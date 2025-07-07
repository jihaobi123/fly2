#!/usr/bin/env python3
import warnings
warnings.simplefilter('ignore', category=FutureWarning)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math
from collections import deque
import json

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from control.DronePositionChecker import DronePositionChecker
from control.AlignmentChecker import AlignmentChecker
from control.ServoControl import ServoControl

class WaterDropMission(Node):
    def __init__(self):
        super().__init__('water_drop_mission')

        # --- 与程序C的通信接口 ---
        # 订阅程序C的投水任务指令
        self.mission_command_sub = self.create_subscription(
            String, '/drop_mission_start', self.mission_command_cb, 10)
        
        # 发布投水任务完成状态给程序C
        self.mission_complete_pub = self.create_publisher(
            Bool, '/drop_mission_complete', 10)
        
        # 发布投水任务状态反馈给程序C
        self.mission_status_pub = self.create_publisher(
            String, '/drop_mission_status', 10)

        # --- Camera + YOLOv8 ---
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None  # 确保初始化
        self.model = YOLO('/home/weights/best.engine')
        self.detecting = False
        self.frame_buffer = deque(maxlen=10)
        self.target_position = None

        # --- Offboard control publishers & subscribers ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_cb, qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.vehicle_local_position = None
        self.vehicle_status = None

        # --- Mission parameters ---
        self.takeoff_height = -1.7     # relative takeoff
        self.forward_x = 2.3           # forward to drop area
        self.align_maxstep = 0.2       # max step for alignment
        self.after_align_dz = 0.5      # descent during second align

        # --- Helpers ---
        self.init_checker   = DronePositionChecker(self.get_logger().info, tolerance=0.17, duration=3.0)
        self.align1_checker = AlignmentChecker(self.get_logger().info, threshold=0.18, time_window=2.0, check_frequency=5)
        self.align2_checker = AlignmentChecker(self.get_logger().info, threshold=0.15, time_window=2.0, check_frequency=10)
        self.servo_control  = ServoControl()
        self.servo_control.initialize_node(self)

        # state flags
        self.is_ready       = False
        self.is_at_height   = False
        self.is_at_droparea = False
        self.first_aligned  = False
        self.second_aligned = False
        self.dropped        = False
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None
        self.init_yaw = 0.0
        self.DropArea_x = None
        self.DropArea_y = None

        # --- 任务控制状态 ---
        self.mission_active = False
        self.mission_sequence = 0
        self.target_bucket_info = None
        self.predefined_target_position = None  # 大致位置（用于初始定位）

        # start timer
        self.timer = self.create_timer(0.03, self.timer_cb)
        self.get_logger().info('WaterDropMission initialized')

    # ========== 与程序C的通信接口 ==========
    def mission_command_cb(self, msg):
        """接收程序C的投水任务指令"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command')
            
            if command_type == 'start_drop_mission':
                self.mission_sequence = command_data.get('sequence', 1)
                self.target_bucket_info = command_data.get('bucket_info', {})
                
                # 检查是否提供了大致位置（用于初始定位）
                if self.target_bucket_info and 'position' in self.target_bucket_info:
                    pos = self.target_bucket_info['position']
                    self.predefined_target_position = Point(
                        x=pos.get('x', 0.0),
                        y=pos.get('y', 0.0),
                        z=pos.get('z', 0.0)
                    )
                    self.get_logger().info(f'Received approximate position: ({self.predefined_target_position.x:.2f}, {self.predefined_target_position.y:.2f}, {self.predefined_target_position.z:.2f})')
                    self.get_logger().info('Will use visual detection for precise alignment')
                else:
                    self.predefined_target_position = None
                    self.get_logger().info('No approximate position provided, using pure visual detection')
                
                self.get_logger().info(f'Received drop mission: sequence {self.mission_sequence}')
                self.get_logger().info(f'Target bucket: {self.target_bucket_info}')
                
                # 激活任务
                self.mission_active = True
                self.reset_mission_state()
                
                # 发布任务开始状态
                self.publish_mission_status('mission_started')
                
            elif command_type == 'cancel_drop_mission':
                self.mission_active = False
                self.publish_mission_status('mission_cancelled')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in mission command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing mission command: {e}')
    
    def publish_mission_status(self, status):
        """发布任务状态给程序C"""
        status_msg = String()
        status_data = {
            'status': status,
            'sequence': self.mission_sequence,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        status_msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(status_msg)
    
    def publish_mission_complete(self):
        """发布任务完成通知给程序C"""
        complete_msg = Bool()
        complete_msg.data = True
        self.mission_complete_pub.publish(complete_msg)
        self.get_logger().info(f'Drop mission {self.mission_sequence} completed')
    
    def reset_mission_state(self):
        """重置任务状态"""
        self.is_ready = False
        self.is_at_height = False
        self.is_at_droparea = False
        self.first_aligned = False
        self.second_aligned = False
        self.dropped = False
        self.detecting = False
        self.frame_buffer.clear()
        
        # 如果有大致位置，可以用于初始定位参考
        if self.predefined_target_position:
            self.get_logger().info('Approximate position available for reference')

    # ========== Camera Callbacks & Detection ==========
    def color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.depth_image is not None:
            self.run_detection()

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if self.color_image is not None:
            self.run_detection()

    def run_detection(self):
        if not self.mission_active:
            return
            
        if self.color_image is None or self.depth_image is None:
            return
            
        img = self.color_image; h,w = img.shape[:2]
        results = self.model(img, imgsz=640, verbose=False)
        candidates = []
        cx0, cy0 = w/2, h/2
        for r in results:
            for box in r.boxes.cpu().numpy():
                x1,y1,x2,y2 = box.xyxy[0]
                conf = float(box.conf[0])
                if conf<0.85: continue
                cx,cy = (x1+x2)/2,(y1+y2)/2
                if cy< h*0.25 or cy>h*0.75: continue
                depth = float(self.depth_image[int(cy),int(cx)])*0.001
                if depth<=0: continue
                d2c = math.hypot(cx-cx0, cy-cy0)
                candidates.append((x1,y1,x2,y2,conf,cx,cy,depth,d2c))
        if not candidates:
            self.detecting = False
            return
            
        # 如果有大致位置，优先选择距离大致位置最近的桶
        if self.predefined_target_position:
            best_candidate = None
            min_distance = float('inf')
            
            for candidate in candidates:
                _,_,_,_,_, cx,cy,depth,_ = candidate
                X,Y,Z = self.pixel2world(int(cx), int(cy), depth)
                # 计算与大致位置的距离
                distance = math.sqrt(
                    (X - self.predefined_target_position.x)**2 +
                    (Y - self.predefined_target_position.y)**2 +
                    (Z - self.predefined_target_position.z)**2
                )
                if distance < min_distance:
                    min_distance = distance
                    best_candidate = candidate
            
            if best_candidate:
                _,_,_,_,_, cx,cy,depth,_ = best_candidate
                X,Y,Z = self.pixel2world(int(cx), int(cy), depth)
                self.target_position = Point(x=X, y=Y, z=Z)
                self.get_logger().info(f'Bucket detected near approximate position (distance: {min_distance:.2f}m)')
        else:
            # 没有大致位置，选择距离画面中心最近的桶
            candidates.sort(key=lambda x:x[8])  # 按距离画面中心排序
            _,_,_,_,_, cx,cy,depth,_ = candidates[0]
            X,Y,Z = self.pixel2world(int(cx), int(cy), depth)
            self.target_position = Point(x=X, y=Y, z=Z)
            self.get_logger().info('Bucket detected -> begin alignment')
            
        if not self.detecting:
            self.publish_mission_status('bucket_detected')
            self.detecting = True

    def pixel2world(self, u,v,d):
        fx,fy=605.78,605.47
        cx,cy=326.35,242.88
        X=(u-cx)*d/fx; Y=(v-cy)*d/fy
        return X,Y,d

    # ========== PX4 callbacks & publishers ==========
    def pos_cb(self, msg):    self.vehicle_local_position=msg
    def status_cb(self, msg): self.vehicle_status=msg
    def publish_offboard_mode(self):
        m=OffboardControlMode(); m.position=True; m.velocity=False; m.attitude=False
        m.timestamp=int(self.get_clock().now().nanoseconds/1000)
        self.offboard_mode_pub.publish(m)
    def publish_setpoint(self,x,y,z):
        m=TrajectorySetpoint(); m.position=[x,y,z]; m.yaw=self.init_yaw
        m.timestamp=int(self.get_clock().now().nanoseconds/1000)
        self.setpoint_pub.publish(m)
    def publish_command(self,cmd, **p):
        m=VehicleCommand(); m.command=cmd
        for i,(k,v) in enumerate(p.items(),1): setattr(m,f'param{i}',v)
        m.target_system=m.source_system=1
        m.timestamp=int(self.get_clock().now().nanoseconds/1000)
        self.command_pub.publish(m)

    # ========== Mission Timer =========================
    def timer_cb(self):
        # 只有在任务激活时才执行
        if not self.mission_active:
            return
            
        # heartbeat
        self.publish_offboard_mode()
        # switch to offboard
        if not self.vehicle_status or self.vehicle_status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            return
        # 1) stable -> arm
        if not self.is_ready and self.vehicle_local_position:
            self.init_checker.update_position((
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ))
            if self.init_checker.is_stable():
                self.is_ready=True
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Armed, starting mission')
                self.publish_mission_status('armed')
            return
        # 2) takeoff
        if self.is_ready and not self.is_at_height and self.vehicle_local_position:
            if self.initial_z is None:
                self.initial_z=self.vehicle_local_position.z
            target_z=self.initial_z + self.takeoff_height
            self.publish_setpoint(self.vehicle_local_position.x,
                                  self.vehicle_local_position.y,
                                  target_z)
            if abs(self.vehicle_local_position.z - target_z)<0.1:
                self.is_at_height=True
                self.get_logger().info('Reached takeoff height')
                self.publish_mission_status('at_height')
            return
        # 3) forward to drop area
        if self.is_at_height and not self.is_at_droparea and self.vehicle_local_position:
            if self.DropArea_x is None:
                self.DropArea_x = self.vehicle_local_position.x + self.forward_x * math.cos(self.init_yaw)
                self.DropArea_y = self.vehicle_local_position.y + self.forward_x * math.sin(self.init_yaw)
            self.publish_setpoint(self.DropArea_x, self.DropArea_y, self.vehicle_local_position.z)
            dist=math.hypot(self.vehicle_local_position.x - self.DropArea_x,
                            self.vehicle_local_position.y - self.DropArea_y)
            if dist<0.1:
                self.is_at_droparea=True
                self.get_logger().info('Arrived drop area')
                self.publish_mission_status('at_drop_area')
            return
        # 4) align & drop
        if self.is_at_droparea and not self.dropped:
            self.perform_alignment_and_drop()
            return

    def perform_alignment_and_drop(self):
        # target_position must exist
        tp = self.target_position
        if not tp or not self.vehicle_local_position: 
            return
        cur = self.vehicle_local_position
        # FRD->NED transform
        dx_frd = tp.y   # lateral
        dy_frd = -tp.x  # forward
        # scale
        dist = math.hypot(tp.x, tp.y)
        scale = min(1, self.align_maxstep/dist)
        step_frd_x = dx_frd*scale
        step_frd_y = dy_frd*scale
        # compute NED
        x_ned = cur.x + step_frd_x*math.cos(self.init_yaw) - step_frd_y*math.sin(self.init_yaw)
        y_ned = cur.y + step_frd_x*math.sin(self.init_yaw) + step_frd_y*math.cos(self.init_yaw)
        # first align
        if not self.first_aligned:
            self.publish_setpoint(x_ned, y_ned, cur.z)
            if self.align1_checker.check(cur.x,cur.y, x_ned,y_ned):
                self.first_aligned=True
                self.get_logger().info('First alignment done')
                self.publish_mission_status('first_aligned')
            return
        # second align
        if not self.second_aligned:
            self.publish_setpoint(x_ned, y_ned, cur.z - self.after_align_dz)
            if self.align2_checker.check(cur.x,cur.y, x_ned,y_ned):
                self.second_aligned=True
                self.get_logger().info('Second alignment done')
                self.publish_mission_status('second_aligned')
            return
        # drop payload
        self.servo_control.open_servo(-1,1)
        self.get_logger().info('Water dropped')
        self.dropped = True
        self.publish_mission_status('water_dropped')
        
        # 任务完成
        self.mission_active = False
        self.publish_mission_complete()

def main():
    rclpy.init()
    node = WaterDropMission()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main() 