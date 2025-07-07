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
        self.pos_eps = 0.08  # 位置误差8厘米
        self.yaw_eps = 0.03  # 朝向误差约17.2°

        # 新增：初始化nav_trigger_hold
        self.nav_trigger_hold = False

        # 每段路径在机体坐标系中的相对移动
        self.relative_points = [
            (0.0, 2.92),  # 向上飞2.92米
            (1.3, 2.92),  # 向前飞1.3米
            (1.3, -2.92), # 向下飞5.84米
            (0.0, -2.92), # 向后飞1.3米
            (0.0, 0.0)    # 回到起点
        ]
        
        # 起飞时自动记录初始位置和朝向
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.ned_points = None
        self.initialized = False

        # 添加日志以验证初始位置和朝向
        self.get_logger().info(f"初始位置: x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}")

        # 修正坐标系转换逻辑，确保转换正确
        def convert_relative_to_ned(relative_points, initial_x, initial_y, initial_yaw):
            if initial_x is None or initial_y is None or initial_yaw is None:
                raise ValueError("初始位置或航向未初始化")
            ned_points = []
            for dx_body, dy_body in relative_points:
                dx_world = dx_body * math.cos(initial_yaw) - dy_body * math.sin(initial_yaw)
                dy_world = dx_body * math.sin(initial_yaw) + dy_body * math.cos(initial_yaw)
                ned_points.append((initial_x + dx_world, initial_y + dy_world))
            return ned_points

        # 初始化时不进行航点转换，避免NoneType错误
        self.ned_points = []

        # 在位置回调中记录初始位置和航向
        def vehicle_local_position_callback(self, msg):
            self.current_height_ = msg.z
            self.current_x_ = msg.x
            self.current_y_ = msg.y
            self.current_yaw_ = msg.heading

            # 自动记录初始位置和航向，只记录一次
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

        # 重写位置回调函数
        self.vehicle_local_position_callback = vehicle_local_position_callback

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

        self.get_logger().info("Offboard Control Node initialized")

    def send_position(self):
        """连续发送位置设定点 (20Hz)"""
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_control_mode.position = True
        offboard_control_mode.velocity = False
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False

        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.is_calibrating:
            # 校准模式下，位置设定为当前位置加上校准偏移
            trajectory_setpoint.position[0] = self.current_x_ + self.nav_offset_x
            trajectory_setpoint.position[1] = self.current_y_ + self.nav_offset_y
            trajectory_setpoint.position[2] = self.target_altitude_
            trajectory_setpoint.yaw = self.current_yaw_
        else:
            # 非校准模式下，按任务点飞行
            trajectory_setpoint.position[0] = self.target_x_
            trajectory_setpoint.position[1] = self.target_y_
            trajectory_setpoint.position[2] = self.target_altitude_
            trajectory_setpoint.yaw = self.target_yaw_

        self.offboard_control_mode_pub_.publish(offboard_control_mode)
        self.trajectory_setpoint_pub_.publish(trajectory_setpoint)

        self.timer_count_ += 1

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

    def convert_frd_to_ned(self, frd_points, current_x, current_y, current_yaw):
        """将FRD坐标系的航点转换为NED坐标系的航点"""
        ned_points = []
        for dx_body, dy_body in frd_points:
            dx_world = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            dy_world = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)
            ned_points.append((current_x + dx_world, current_y + dy_world))
        return ned_points

    def control_sequence(self):
        """控制序列：解锁 -> Offboard模式 -> 飞行"""
        if not self.initialized:
            return  # 未初始化前不推进状态机

        if self.timer_count_ < 10:
            return

        # 解锁
        if not self.armed_ and not self.arm_command_sent_:
            self.send_arm_command()
            self.arm_command_sent_ = True
            self.get_logger().info("发送解锁命令")
            return

        # 步骤2：进入Offboard模式
        if self.armed_ and not self.offboard_mode_ and not self.offboard_command_sent_:
            self.send_offboard_command()
            self.offboard_command_sent_ = True
            self.get_logger().info("发送Offboard模式命令")
            return

        # 校准时暂停航线推进
        if self.is_calibrating:
            self.get_logger().info("校准中，暂停航线推进")
            return

        # nav_trigger为True时悬停，为False时恢复
        if self.nav_trigger_hold:
            self.target_x_ = self.current_x_
            self.target_y_ = self.current_y_
            self.target_yaw_ = self.current_yaw_
            self.get_logger().info("因/nav_trigger悬停，等待恢复")
            return

        # 状态机：每个拐角分为"转头"和"前进"两个阶段
        if self.offboard_mode_:
            # 根据当前阶段设置目标点
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
        
        if self.armed_ and not was_armed:
            self.get_logger().info("✓ 无人机已解锁")

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
