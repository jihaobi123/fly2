import warnings
warnings.simplefilter('ignore', category=FutureWarning)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from std_msgs.msg import Float32, Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from collections import deque
import math


class BucketDetectorNode(Node):
    def __init__(self):
        super().__init__('bucket_detector')

        # 订阅图像与深度图
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_cb,
            qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_cb,
            qos_profile_sensor_data
        )
        # 订阅 PX4 发布的本地位置消息
        self.pose_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pose_cb,
            qos_profile_sensor_data
        )

        # 发布给成员 A 的通知话题
        self.trigger_pub = self.create_publisher(Bool, '/nav_trigger', 10)
        self.offset_pub = self.create_publisher(Point, '/nav_offset', 10)
        
        # 发布给成员 C 的圆筒数据话题
        self.bucket_report_pub = self.create_publisher(String, '/buckets_report', 10)

        # 模型加载（YOLOv8）
        self.model = YOLO('/home/cqu/weights/best.pt')

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.detecting = False
        self.frame_buffer = deque(maxlen=10)
        self.found_positions = []
        self.bucket_data = []
        self.recording = False
        self.center_threshold = 50  # 默认像素阈值，将被动态阈值替代

        # 处理完成标志：三桶报告发出后就不再识别
        self.completed = False

        self.get_logger().info("YOLOv8 Bucket detector initialized")
        cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)

    def pose_cb(self, msg: VehicleLocalPosition):
        # VehicleLocalPosition 中直接包含 x,y,z
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        #self.get_logger().info(
            #f"[pose_cb] 位姿更新: x={self.current_x:.2f}, "
            #f"y={self.current_y:.2f}, z={self.current_z:.2f}"
        #)

    def color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.depth_image is not None:
            self.process()

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def process(self):
        # 一旦发送完三桶报告后，直接跳过所有后续处理
        if self.completed:
            return
        # 保证彩色图、深度图和无人机位置都已就绪
        if self.color_image is None or self.depth_image is None:
            self.get_logger().info("等待图像或位姿输入...")
            return
        detections = self.detect()
        h, w = self.color_image.shape[:2]
        fx = 605.7783

        # 过滤并排序检测结果（按距离画面中心排序）
        valid_detections = []
        cx0 = w / 2  # 画面中心x坐标
        cy0 = h / 2  # 画面中心y坐标
        
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            if conf < 0.85:
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            if cy < h * 0.25 or cy > h * 0.75:
                continue

            # 检查深度图是否有效
            if cy >= self.depth_image.shape[0] or cx >= self.depth_image.shape[1]:
                continue
            depth = self.depth_image[cy, cx] * 0.001
            if depth <= 0:
                continue

            # 检查是否已经处理过这个位置的桶
            if self.current_x is not None:
                px = self.current_x
                py = self.current_y
                skip = False
                for old_x, old_y in self.found_positions:
                    if math.hypot(px - old_x, py - old_y) < 1.0:
                        skip = True
                        break
                if skip:
                    self.get_logger().info(f"Skipping already processed bucket at position ({px:.2f}, {py:.2f})")
                    continue

            # 计算到画面中心的距离
            dx = cx - cx0
            dy = cy - cy0
            distance_to_center = math.sqrt(dx*dx + dy*dy)
            
            valid_detections.append((x1, y1, x2, y2, conf, cls, cx, cy, depth, distance_to_center))
        
        # 按距离画面中心排序，处理最近的桶
        if valid_detections:
            valid_detections.sort(key=lambda x: x[9])  # 按距离升序（最近优先）
            x1, y1, x2, y2, conf, cls, cx, cy, depth, distance_to_center = valid_detections[0]
            
            # 提前计算桶宽度
            box_width_px = x2 - x1
            
            # 显示检测框
            cv2.rectangle(self.color_image, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(self.color_image, f"{conf:.2f}",
                        (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # 显示检测到的桶数量和当前桶信息
            if len(valid_detections) > 1:
                cv2.putText(self.color_image, f"Detected: {len(valid_detections)} buckets", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                # 显示所有桶的距离信息
                for i, det in enumerate(valid_detections[:3]):  # 只显示前3个
                    _, _, _, _, _, _, _, _, _, dist = det
                    cv2.putText(self.color_image, f"#{i+1}: {dist:.1f}px", 
                               (10, 120 + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 显示当前桶的尺寸信息
            cv2.putText(self.color_image, f"Bucket: {box_width_px:.0f}px width", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 计算中心偏差（距离已在前面计算过）
            dx = cx - cx0
            dy = cy0 - cy  # 修正y轴方向（图像坐标系y向下，导航坐标系y向上）
            
            # 动态阈值：使用桶的半宽作为中心阈值，保证至少20px
            dynamic_threshold = max(box_width_px * 0.5, 20)  # 半宽以内算靠中，最小阈值20px
            
            # 检查是否接近画面中心
            if distance_to_center <= dynamic_threshold:
                self.trigger_pub.publish(Bool(data=True))  # 通知A进入校准
                # 圆筒靠中 —— 开始／持续记录，不再发偏差
                if not self.recording:
                    bucket_rank = "1st" if len(valid_detections) == 1 else f"1st of {len(valid_detections)}"
                    self.get_logger().info(f"Bucket near center ({bucket_rank}, distance: {distance_to_center:.1f}px, threshold: {dynamic_threshold:.1f}px), start recording")
                    self.recording = True
                    self.detecting = True
                self.frame_buffer.append((x2 - x1, depth))
                self.get_logger().info(f"📸 Recording frame {len(self.frame_buffer)}/10")
                
                # 显示中心状态
                cv2.circle(self.color_image, (int(cx0), int(cy0)), int(dynamic_threshold), (0, 0, 255), 2)
                cv2.putText(self.color_image, f"Recording: {len(self.frame_buffer)}/10", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                if len(self.frame_buffer) == self.frame_buffer.maxlen:
                    self.finalize_bucket(cx, cy, depth)
            else:
                # 圆筒不在中心 —— 只暂停录制，不清空帧缓存
                if self.recording:
                    self.get_logger().info(f"🎞️ Bucket lost center - keep buffer ({len(self.frame_buffer)}/10)")
                # 显示偏差状态
                cv2.circle(self.color_image, (int(cx0), int(cy0)), int(dynamic_threshold), (255, 0, 0), 2)
                cv2.putText(self.color_image, f"Adjust: {distance_to_center:.1f}px (thr {dynamic_threshold:.1f})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # 如果没有检测到圆筒，且之前正在检测，则重置状态
        if not detections and self.detecting:
            self.get_logger().info(f"No bucket detected, resetting detection state and buffer (buffer length: {len(self.frame_buffer)})")
            self.detecting = False
            self.recording = False
            self.frame_buffer.clear()
        
        # 显示已处理桶的数量
        if self.found_positions:
            cv2.putText(self.color_image, f"Processed: {len(self.found_positions)} buckets", 
                       (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # ────── 一定要和上面同级缩进 ──────
        cv2.imshow("Color Detection", self.color_image)
        cv2.waitKey(1)


    def finalize_bucket(self, cx, cy, depth):
        self.get_logger().info("🎯 finalize_bucket() called")
        # 必须先有位置数据才能记录并发布 False
        if self.current_x is None:
            self.get_logger().warn("[finalize_bucket] 缺少位姿，跳过发布 False")
            return
        px = self.current_x
        py = self.current_y
        for old in self.found_positions:
            if math.hypot(px - old[0], py - old[1]) < 1.0:
                self.get_logger().info(f"Duplicate bucket skipped at position ({px:.2f}, {py:.2f}) - too close to previous bucket")
                return

        self.found_positions.append((px, py))
        self.get_logger().info(f"New bucket recorded at position ({px:.2f}, {py:.2f}) - total processed: {len(self.found_positions)}")

        widths, depths = zip(*self.frame_buffer)
        avg_width = sum(widths) / len(widths)
        avg_depth = sum(depths) / len(depths)
        real_diameter = avg_width * avg_depth / 605.7783

        self.bucket_data.append({
            'pixel_width': avg_width,
            'depth': avg_depth,
            'diameter': real_diameter,
            'x': px,
            'y': py,
            'z': self.current_z,
            'height': depth  # 记录当前高度
        })

        self.get_logger().info(f"Bucket recorded: diameter={real_diameter:.3f}m, depth={avg_depth:.2f}m, height={depth:.2f}m")

        # 重置状态
        self.detecting = False
        self.recording = False
        self.frame_buffer.clear()

        # 通知成员A继续执行航线程序
        self.get_logger().info("📣 Publish to Member A: /nav_trigger -> False (continue waypoint)")
        self.trigger_pub.publish(Bool(data=False))

        if len(self.bucket_data) == 3:
            self.get_logger().info("3 buckets recorded, ready for Member C")
            # 创建圆筒报告消息
            report_msg = self.create_bucket_report()
            self.get_logger().info(f"📣 Publish to Member C: /buckets_report -> {report_msg.data}")
            self.bucket_report_pub.publish(report_msg)
            # 发布完报表后清空数据，避免重复发送
            self.bucket_data.clear()
            # —— 三桶报告发出后，标记完成，不再继续识别 —— #
            self.completed = True

    def create_bucket_report(self):
        """创建圆筒报告消息"""
        import json
        
        # 更精细的时间戳（纳秒级）
        t = self.get_clock().now().to_msg()
        report_data = {
            'timestamp': t.sec + t.nanosec * 1e-9,
            'bucket_count': len(self.bucket_data),
            'buckets': []
        }
        
        for i, bucket in enumerate(self.bucket_data):
            bucket_info = {
                'id': i + 1,
                'diameter': round(bucket['diameter'], 3),
                'depth': round(bucket['depth'], 2),
                'height': round(bucket['height'], 2),
                'position': {
                    'x': round(bucket['x'], 2),
                    'y': round(bucket['y'], 2),
                    'z': round(bucket['z'], 2)
                }
            }
            report_data['buckets'].append(bucket_info)
        
        report_json = json.dumps(report_data, ensure_ascii=False)
        return String(data=report_json)

    def detect(self):
        results = self.model(self.color_image, imgsz=640, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes.cpu().numpy():
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0]
                cls = box.cls[0]
                detections.append([x1, y1, x2, y2, conf, cls])
        return detections


def main():
    rclpy.init()
    node = BucketDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
