import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
import time


class SimpleServoController(Node):
    """极简舵机控制器 - 只有开关功能"""
   
    def __init__(self):
        super().__init__('simple_servo_controller')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # 发布器
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
       
        self.get_logger().info("极简舵机控制器已启动")
   
    def publish_vehicle_command(self, command, **params) -> None:
        """发布MAVLink命令 - 通用函数（参考主飞控）"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
   
    def set_servo(self, servo_number: int):
        """Send a disarm command to the vehicle."""
        if servo_number == 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1=1.0, param7=1.0)
            self.get_logger().info('Servo 1 set')
        elif servo_number == 2:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1=1.0, param7=2.0)
            self.get_logger().info('Servo 2 set')


def main():
    """主函数"""
    rclpy.init()
   
    controller = SimpleServoController()
   
    def interactive_control():
        time.sleep(1)
        print("\n=== 极简舵机控制 ===")
        print("命令: 1o=舵机1开 1c=舵机1关 2o=舵机2开 2c=舵机2关 ao=全开 ac=全关 q=退出")
       
        while rclpy.ok():
            try:
                cmd = input("命令: ").strip().lower()
               
                if cmd == 'q':
                    break
                elif cmd == '1o':
                    controller.set_servo(1)
                elif cmd == '1c':
                    controller.set_servo(1)
                elif cmd == '2o':
                    controller.set_servo(2)
                elif cmd == '2c':
                    controller.set_servo(2)
                elif cmd == 'ao':
                    controller.set_servo(1)
                    controller.set_servo(2)
                elif cmd == 'ac':
                    controller.set_servo(1)
                    controller.set_servo(2)
                elif cmd == '':
                    continue
                else:
                    print("错误命令")
               
            except (KeyboardInterrupt, EOFError):
                break
   
    try:
        # 启动交互控制
        control_thread = threading.Thread(target=interactive_control, daemon=True)
        control_thread.start()
       
        # 运行节点
        rclpy.spin(controller)
   
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

