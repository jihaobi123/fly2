import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time
import math

class OffboardControlNode(Node):

    def __init__(self):
        super().__init__('offboard_control_node')

        # 订阅车辆状态和定位信息
        self.vehicle_status_sub_ = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sensor_data)
        
        self.vehicle_local_position_sub_ = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile_sensor_data)

        # 创建命令发布者
        self.offboard_control_mode_pub_ = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile_sensor_data)
        
        self.trajectory_setpoint_pub_ = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)
        
        self.vehicle_command_pub_ = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile_sensor_data)

        # 定时器 (20Hz)
        self.send_position_timer_ = self.create_timer(0.05, self.send_position)
        self.control_sequence_timer_ = self.create_timer(0.5, self.control_sequence)

        # 变量初始化
        self.timer_count_ = 0
        self.stage = 0  # 0~5: 任务点, 6: 悬停, 7: 降落, 8: 完成
        self.target_altitude_ = -2.0  # NED坐标系：2米高度
        self.target_x_ = 0.0
        self.target_y_ = 0.0
        self.target_yaw_ = 0.0  # 朝向，弧度
        self.current_height_ = 0.0
        self.current_x_ = 0.0
        self.current_y_ = 0.0
        self.current_yaw_ = 0.0
        self.armed_ = False
        self.offboard_mode_ = False
        self.offboard_entered_ = False
        self.arm_command_sent_ = False
        self.offboard_command_sent_ = False
        self.hover_start_time = None  # 悬停计时
        self.turn_wait_start = None

        # 误差阈值
        self.pos_eps = 0.15  # 位置误差8厘米
        self.yaw_eps = 0.03  # 朝向误差约17.2°

        # 新增：初始化nav_trigger_hold
        self.nav_trigger_hold = False

        # 新增：初始化延迟计数器
        self.initialization_delay_counter = 0  # 用于IMU稳定等待
        self.post_arm_hold_counter = 0  # 解锁后延迟进入offboard
        self.just_armed = False  # 标记刚解锁

        # 每段路径在机体坐标系中的相对移动
        self.relative_points = [
            (0.0, 2.92),   # 向上飞2.92米
            (1.3, 2.92),   # 向前飞1.3米
            (1.3, 0.0),
            (1.3, -2.92), # 向下飞5.84米
            (0.0, -2.92),  # 向后飞1.3米
            (0.0, 0.0)     # 回到起点
        ]
        
        # 起飞时自动记录初始位置和朝向
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.ned_points = None
        self.initialized = False

        # 添加日志以验证初始位置和朝向
        self.get_logger().info(f"初始位置: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}")

        # 初始化时不进行航点转换，避免NoneType错误
        self.ned_points = []

        # 订阅/nav_trigger（Bool型）和/nav_offset（Point型）
        self.nav_trigger_sub = self.create_subscription(
            Bool,
            '/nav_trigger',
            self.nav_trigger_callback,
            10
        )
        self.nav_offset_sub = self.create_subscription(
            Point,
            '/nav_offset',
            self.nav_offset_callback,
            10
        )

        # 初始化校准相关变量
        self.is_calibrating = False  # True时正在校准
        self.last_stage_before_calibration = None  # 校准前保存的阶段
        self.nav_offset_x = 0.0  # 校准的水平偏差x
        self.nav_offset_y = 0.0  # 校准的垂直偏差y

        # 新增：订阅下一个桶目标点
        self.next_bucket_point_sub = self.create_subscription(
            Point,
            '/next_bucket_point',
            self.next_bucket_point_callback,
            10
        )
        self.next_target_received = False
        # 用来区分收到的是第几次桶目标
        self.bucket_points_received = 0

        # —— 新增：记录投水完成次数及后续飞行阶段 —— 
        self.drop_count = 0
        self.do_final_phase = False    # 是否进入"后续区域飞行"阶段
        self.final_phase = 0           # 0=旋转朝向,1=前进20m,2=保持
        self.final_start_x = 0.0
        self.final_start_y = 0.0
        # 订阅投水完成通知（同 BarrelManager → DropMission 的 /drop_mission_complete）
        self.drop_complete_sub = self.create_subscription(
            Bool,
            '/drop_mission_complete',
            self.drop_complete_callback,
            10
        )

        # 新增：发布到位通知给程序C
        self.bucket_reached_pub = self.create_publisher(
            Bool,
            '/bucket_reached',
            10
        )

        self.get_logger().info("Offboard Control Node initialized")

    def send_position(self):
        """连续发送位置设定点 (20Hz)"""
        if not self.initialized:
            return  # 位置还没初始化不能发指令

        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_control_mode.position = True
        offboard_control_mode.velocity = False
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False

        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # —— 优先处理"后续区域飞行" —— 
        if self.do_final_phase:
            # —— 1) 旋转朝向起飞时初始航向 —— 
            if self.final_phase == 0:
                trajectory_setpoint.position[0] = self.current_x_
                trajectory_setpoint.position[1] = self.current_y_
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.initial_yaw
                # 到位后进入前进
                if self._reach_yaw(self.initial_yaw):
                    self.final_phase = 1
                    self.get_logger().info('完成朝向调整，开始前进20m')
                    self.get_logger().info(f'目标位置: ({self.final_start_x + 20 * math.cos(self.initial_yaw):.2f}, {self.final_start_y + 20 * math.sin(self.initial_yaw):.2f})')
            # —— 2) 向起飞前方前进20m —— 
            elif self.final_phase == 1:
                fx = self.final_start_x + 20 * math.cos(self.initial_yaw)
                fy = self.final_start_y + 20 * math.sin(self.initial_yaw)
                trajectory_setpoint.position[0] = fx
                trajectory_setpoint.position[1] = fy
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.initial_yaw
                if self._reach_pos(fx, fy, self.target_altitude_):
                    self.final_phase = 2
                    self.get_logger().info('完成20m前进，任务结束')
                    self.get_logger().info(f'最终位置: ({self.current_x_:.2f}, {self.current_y_:.2f})')
            # —— 3) 保持当前位置悬停 —— 
            else:
                trajectory_setpoint.position[0] = self.current_x_
                trajectory_setpoint.position[1] = self.current_y_
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.initial_yaw

            # 发布并退出，不执行原航线逻辑
            self.offboard_control_mode_pub_.publish(offboard_control_mode)
            self.trajectory_setpoint_pub_.publish(trajectory_setpoint)
            self.timer_count_ += 1
            return

        # 原有：offboard模式刚进入的前2秒也只发当前位置
        if not self.offboard_mode_ or self.timer_count_ < 40:
            trajectory_setpoint.position[0] = self.current_x_
            trajectory_setpoint.position[1] = self.current_y_
            trajectory_setpoint.position[2] = self.target_altitude_
            trajectory_setpoint.yaw = self.current_yaw_
        else:
            if self.next_target_received:
                # —— 新增：直接飞向下一个桶的目标点 —— 
                trajectory_setpoint.position[0] = self.target_x_
                trajectory_setpoint.position[1] = self.target_y_
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.current_yaw_
            elif self.is_calibrating:
                trajectory_setpoint.position[0] = self.current_x_ + self.nav_offset_x
                trajectory_setpoint.position[1] = self.current_y_ + self.nav_offset_y
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.current_yaw_
            else:
                # 保留原有的相对航点流程
                trajectory_setpoint.position[0] = self.target_x_
                trajectory_setpoint.position[1] = self.target_y_
                trajectory_setpoint.position[2] = self.target_altitude_
                trajectory_setpoint.yaw = self.target_yaw_

        # 发布控制模式与目标点
        self.offboard_control_mode_pub_.publish(offboard_control_mode)
        self.trajectory_setpoint_pub_.publish(trajectory_setpoint)
        self.timer_count_ += 1

        # 新增：检查是否到达桶目标点，发布到位通知
        if (self.next_target_received and 
            not self.do_final_phase and  # 确保只在"桶模式"下发到位通知
            self._reach_pos(self.target_x_, self.target_y_, self.target_altitude_)):
            # 发布到位通知给程序C
            reached_msg = Bool()
            reached_msg.data = True
            self.bucket_reached_pub.publish(reached_msg)
            self.get_logger().info(f"已到达桶目标点，发布到位通知: ({self.target_x_:.2f}, {self.target_y_:.2f})")
            # 清除目标接收状态，避免重复发布
            self.next_target_received = False

    def nav_trigger_callback(self, msg):
        self.nav_trigger_hold = msg.data
        self.get_logger().info(f"收到/nav_trigger: {msg.data}，{'开始校准' if msg.data else '恢复航线'}")
        if msg.data:
            # 开始校准时保存当前阶段
            self.last_stage_before_calibration = self.stage
            self.is_calibrating = True
        else:
            # 恢复航线时回到之前的阶段
            if self.last_stage_before_calibration is not None:
                self.stage = self.last_stage_before_calibration
                self.last_stage_before_calibration = None
            self.is_calibrating = False

    def nav_offset_callback(self, msg):
        self.nav_offset_x = msg.x
        self.nav_offset_y = msg.y
        self.get_logger().info(f"收到/nav_offset: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}，开始校准")

    def next_bucket_point_callback(self, msg):
        """收到 BarrelManager 下发的下一个桶目标点"""
        
        self.get_logger().info(f"【DEBUG】 next_bucket_point_callback 被调用了！ msg=({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})")
        # 累计一次，并打印是小桶还是中桶
        self.bucket_points_received += 1
        if self.bucket_points_received == 1:
            self.get_logger().info(
                f"【小桶位置】收到 /next_bucket_point: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )
        elif self.bucket_points_received == 2:
            self.get_logger().info(
                f"【中桶位置】收到 /next_bucket_point: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )
        else:
            self.get_logger().warn(
                f"收到多余的桶目标 #{self.bucket_points_received}: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )

        self.target_x_ = msg.x
        self.target_y_ = msg.y
        self.target_altitude_ = msg.z
        # 一旦收到，就直接进入"直接飞向目标"模式
        self.next_target_received = True

    def drop_complete_callback(self, msg):
        """处理每次投水完成，累计两次后进入后续飞行阶段"""
        # 防止重复触发：如果已经进入最终阶段，则忽略后续消息
        if self.do_final_phase:
            return
            
        if msg.data:
            self.drop_count += 1
            self.get_logger().info(f"已完成第 {self.drop_count} 次投水")
            if self.drop_count >= 2:
                # 清除桶目标接收状态，防止逻辑冲突
                self.next_target_received = False
                # 重置桶目标计数器，为下次任务做准备
                self.bucket_points_received = 0
                # 记录开始后续飞行时的当前位置和朝向
                self.final_start_x = self.current_x_
                self.final_start_y = self.current_y_
                self.do_final_phase = True
                self.final_phase = 0
                self.get_logger().info("进入完成投水后的区域飞行阶段")
                self.get_logger().info(f"记录起始位置: ({self.final_start_x:.2f}, {self.final_start_y:.2f})")
                self.get_logger().info(f"目标航向: {self.initial_yaw:.3f} rad")

    def convert_frd_to_ned(self, frd_points, current_x, current_y, current_yaw):
        """将FRD坐标系的航点转换为NED坐标系的航点"""
        ned_points = []
        for dx_body, dy_body in frd_points:
            dx_world = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            dy_world = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)
            ned_points.append((current_x + dx_world, current_y + dy_world))
        return ned_points

    def control_sequence(self):
        # —— 打印当前模式 ——
        if self.next_target_received:
            self.get_logger().info('[MODE] Bucket（桶目标模式）')
            # 正在飞桶目标，完全跳过原航线推进
            return
        if self.do_final_phase:
            self.get_logger().info('[MODE] Final（投水后续区域飞行模式）')
            return
        self.get_logger().info('[MODE] Line（原航线模式）')
        # —— 3）原有解锁/Offboard/校准逻辑 —— 
        if not self.initialized or self.timer_count_ < 10:
            return
        if not self.armed_:
            self.send_arm_command()
            self.arm_command_sent_ = True
            self.get_logger().info("发送解锁命令")
            return
        if not self.offboard_mode_:
            self.send_offboard_command()
            self.offboard_command_sent_ = True
            self.get_logger().info("发送Offboard模式命令")
            return
        if self.is_calibrating or self.nav_trigger_hold:
            self.get_logger().info("校准中，暂停航线推进")
            return

        # 到这里才是按 stage 推进原航线
        if self.stage < len(self.relative_points):
            ned_point = self.ned_points[self.stage]
            if ned_point:
                self.target_x_, self.target_y_ = ned_point
                self.target_yaw_ = self.current_yaw_

                # 判断是否到达目标点
                if self._reach_pos(self.target_x_, self.target_y_, self.target_altitude_):
                    self.stage += 1  # 进入下一阶段
                    self.get_logger().info(f"阶段{self.stage - 1}完成，进入阶段{self.stage}")
        else:
            self.get_logger().info("所有任务点已完成")
            self.stage = len(self.relative_points)  # 停止推进

    def _reach_pos(self, x, y, z):
        return (abs(self.current_x_ - x) < self.pos_eps and
                abs(self.current_y_ - y) < self.pos_eps and
                abs(self.current_height_ - z) < self.pos_eps)

    def _reach_yaw(self, yaw):
        dyaw = abs(self._normalize_angle(self.current_yaw_ - yaw))
        return dyaw < self.yaw_eps

    def _normalize_angle(self, angle):
        """将角度归一化到-pi~pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def send_arm_command(self):
        """发送解锁命令"""
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.param1 = 1.0  # 1=解锁, 0=锁定
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True

        self.vehicle_command_pub_.publish(vehicle_command)

    def send_offboard_command(self):
        """发送Offboard模式命令"""
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        vehicle_command.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        vehicle_command.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        vehicle_command.param3 = 0.0  # PX4_CUSTOM_SUB_MODE_OFFBOARD
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True

        self.vehicle_command_pub_.publish(vehicle_command)

    def vehicle_status_callback(self, msg):
        """车辆状态回调"""
        was_armed = getattr(self, 'armed_', False)
        self.armed_ = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        
        # 优化1：刚解锁时，进入just_armed状态
        if self.armed_ and not was_armed:
            self.get_logger().info("✓ 无人机已解锁")
            self.just_armed = True
            self.post_arm_hold_counter = 0

        was_offboard = getattr(self, 'offboard_mode_', False)
        self.offboard_mode_ = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        if self.offboard_mode_ and not was_offboard:
            self.offboard_entered_ = True
            self.get_logger().info("✓ 已进入Offboard模式")

    def vehicle_local_position_callback(self, msg):
        """位置回调"""
        self.current_height_ = msg.z
        self.current_x_ = msg.x
        self.current_y_ = msg.y
        self.current_yaw_ = msg.heading  # 获取无人机当前朝向

        # 优化2：IMU初始化等待，计数未到前不记录初始点
        if self.initialization_delay_counter < 40:  # 2秒等待
            self.initialization_delay_counter += 1
            return

        # 自动记录初始位置和朝向，只记录一次
        if not self.initialized and self.current_x_ is not None and self.current_y_ is not None:
            # 确保初始值初始化
            if self.initial_x is None and self.initial_y is None:
                self.initial_x = self.current_x_
                self.initial_y = self.current_y_

            # 在记录初始位置和航向前，增加稳定性检查
            if abs(self.current_x_ - self.initial_x) < self.pos_eps and abs(self.current_y_ - self.initial_y) < self.pos_eps:
                self.initial_x = self.current_x_
                self.initial_y = self.current_y_
                self.initial_yaw = self.current_yaw_
                self.ned_points = self.convert_frd_to_ned(self.relative_points, self.initial_x, self.initial_y, self.initial_yaw)
                self.get_logger().info(f"自动记录初始位置: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}")
                self.get_logger().info(f"转换后的NED航点: {self.ned_points}")
                self.initialized = True

    def transform_body_to_world(self, dx_body, dy_body, yaw, current_x, current_y):
        dx_world = dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
        dy_world = dx_body * math.sin(yaw) + dy_body * math.cos(yaw)
        return current_x + dx_world, current_y + dy_world

def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = OffboardControlNode()

    try:
        rclpy.spin(offboard_control_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        offboard_control_node.get_logger().info('Shutting down...')
        offboard_control_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
