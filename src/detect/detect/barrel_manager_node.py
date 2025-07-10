import rclpy

from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray, Bool

from geometry_msgs.msg import PoseStamped, Point

import json

import math

import time

from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition



class BarrelManager(Node):

    def __init__(self):

        super().__init__('barrel_manager')

        self.barrels = {}  # id: {'diameter':..., 'height':..., 'real_size':...}

        self.processed = False  # 标记是否已经处理过

        self.current_drone_position = (0.0, 0.0, 2.0)  # 无人机当前位置 (x, y, z)

        self.current_target_index = 0  # 当前目标桶的索引

        self.sorted_barrel_ids = []  # 按大小排序的桶ID列表

        self.waiting_for_drop_success = False  # 是否在等待投水成功消息

        self.mission_completed = False  # 任务是否已完成

        self.current_mission_sequence = 0  # 当前任务序列号

        
        self.get_logger().info("YOLOv8 Bucket detector initialized")
        # 订阅器

        self.subscription = self.create_subscription(

            String,  # 订阅JSON格式的桶识别报告（来自程序B）

            '/buckets_report',

            self.barrel_callback,

            10)

        self.reset_subscription = self.create_subscription(

            String,

            '/reset_barrel_manager',

            self.reset_callback,

            10)

        # 订阅程序D的任务完成状态

        self.drop_mission_complete_subscription = self.create_subscription(

            Bool,

            '/drop_mission_complete',

            self.drop_mission_complete_callback,

            10)

        # 订阅程序D的任务状态反馈

        self.drop_mission_status_subscription = self.create_subscription(

            String,

            '/drop_mission_status',

            self.drop_mission_status_callback,

            10)

        # 订阅无人机当前位置（PX4 → VehicleLocalPosition）
        self.drone_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.drone_position_callback,
            qos_profile_sensor_data)

        

        # 发布器

        self.order_publisher = self.create_publisher(Int32MultiArray, '/bucket_order', 10)

        self.finish_publisher = self.create_publisher(Bool, '/finish_scan', 10)

        self.target_coords_publisher = self.create_publisher(String, '/target_coordinates', 10)

        # 发布投水任务指令给程序D

        self.drop_mission_start_publisher = self.create_publisher(String, '/drop_mission_start', 10)

        # 发布任务完成信号

        self.mission_complete_publisher = self.create_publisher(Bool, '/mission_complete', 10)

        # 新增：下一个桶目标点发布器

        self.next_bucket_point_pub = self.create_publisher(

            Point,

            '/next_bucket_point',

            10

        )

        # 新增：订阅程序A的到位通知

        self.reached_sub = self.create_subscription(

            Bool, '/bucket_reached', self.on_reached, 10)



    def drone_position_callback(self, msg):

        """更新无人机当前位置 (直接从 VehicleLocalPosition 读取)"""

        self.current_drone_position = (msg.x, msg.y, msg.z)

        self.get_logger().debug(

            f"无人机位置更新: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")



    def drop_mission_complete_callback(self, msg):

        """处理程序D的投水任务完成消息"""

        if msg.data and self.waiting_for_drop_success and not self.mission_completed:

            self.get_logger().info("收到程序D的投水任务完成消息，准备移动到下一个桶")

            self.waiting_for_drop_success = False

            

            # 移动到下一个桶

            self.current_target_index += 1

            

            if self.current_target_index >= len(self.sorted_barrel_ids):

                # 所有桶都处理完成

                self.get_logger().info("所有桶都处理完成，任务结束")

                mission_complete_msg = Bool()

                mission_complete_msg.data = True

                self.mission_complete_publisher.publish(mission_complete_msg)

                self.mission_completed = True

                return

            

            # 获取下一个目标桶

            next_barrel_id = self.sorted_barrel_ids[self.current_target_index]

            self.get_logger().info(f"移动到下一个桶：桶{next_barrel_id}")

            

            # 发布下一个桶的坐标给程序A，等待程序A到位后再发送投水指令

            if not self.mission_completed:

                self.send_next_bucket_coordinate(next_barrel_id)



    def drop_mission_status_callback(self, msg):

        """处理程序D的任务状态反馈"""

        try:

            status_data = json.loads(msg.data)

            status = status_data.get('status', 'unknown')

            sequence = status_data.get('sequence', 0)

            self.get_logger().info(f"程序D任务状态: {status} (序列: {sequence})")

        except json.JSONDecodeError as e:

            self.get_logger().error(f'解析程序D状态消息失败: {e}')



    def calculate_distance(self, point1, point2):

        """计算两点间的距离"""

        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)



    def find_nearest_barrel_from_list(self, barrel_list):

        """从指定桶列表中找到距离无人机最近的桶"""

        if not barrel_list:

            return None

            

        min_distance = float('inf')

        nearest_barrel_id = None

        

        for barrel_id in barrel_list:

            barrel_position = self.barrels[barrel_id]['position']

            # 计算到桶正上方的距离（z坐标设为无人机当前高度）

            target_position = (barrel_position[0], barrel_position[1], self.current_drone_position[2])

            distance = self.calculate_distance(self.current_drone_position, target_position)

            

            if distance < min_distance:

                min_distance = distance

                nearest_barrel_id = barrel_id

        

        return nearest_barrel_id



    def send_next_bucket_coordinate(self, barrel_id):

        """发送下一个桶的坐标给程序A，但不立即发送投水指令"""

        if barrel_id not in self.barrels:

            self.get_logger().error(f"桶{barrel_id}不存在")

            return False

            

        barrel_data = self.barrels[barrel_id]

        barrel_position = barrel_data['position']

        

        # 发布桶的坐标给程序A

        point = Point()

        point.x = barrel_position[0]

        point.y = barrel_position[1]

        point.z = self.current_drone_position[2]  # 以当前无人机高度为目标高度

        self.next_bucket_point_pub.publish(point)

        self.get_logger().info(

            f"发布下一个桶目标点到 /next_bucket_point："

            f" x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}"

        )

        self.get_logger().info(f"等待程序A到达桶{barrel_id}上方后再发送投水指令")

        return True

    def send_drop_mission_to_program_d(self, barrel_id):

        """发送投水任务指令给程序D"""

        if barrel_id not in self.barrels:

            self.get_logger().error(f"桶{barrel_id}不存在")

            return False

            

        barrel_data = self.barrels[barrel_id]

        barrel_position = barrel_data['position']

        

        # 构建投水任务指令（JSON格式）

        mission_command = {

            'command': 'start_drop_mission',

            'sequence': self.current_mission_sequence + 1,

            'bucket_info': {

                'id': barrel_id,

                'diameter': barrel_data['diameter'],

                'height': barrel_data['height'],

                'position': {

                    'x': barrel_position[0],

                    'y': barrel_position[1],

                    'z': self.current_drone_position[2]  # 使用当前无人机高度

                }

            }

        }

        

        # 发布任务指令

        mission_msg = String()

        mission_msg.data = json.dumps(mission_command)

        self.drop_mission_start_publisher.publish(mission_msg)

        

        self.current_mission_sequence += 1

        self.waiting_for_drop_success = True

        

        self.get_logger().info(f"发送投水任务给程序D：桶{barrel_id}，位置({barrel_position[0]:.2f}, {barrel_position[1]:.2f})")

        return True

    def on_reached(self, msg):

        """收到程序A的到位通知，发送投水指令"""

        if not self.waiting_for_drop_success and not self.mission_completed:

            # 这里用 current_target_index 发当前桶的投水指令

            barrel_id = self.sorted_barrel_ids[self.current_target_index]

            self.get_logger().info(f"收到程序A到位通知，开始发送投水指令给桶{barrel_id}")

            self.send_drop_mission_to_program_d(barrel_id)



    def start_mission(self):

        """开始执行任务"""

        if not self.sorted_barrel_ids:

            self.get_logger().error("没有可用的桶数据")

            return



        # 只处理中小桶（索引1和2），不处理大桶（索引0）

        if len(self.sorted_barrel_ids) >= 3:

            # 获取中小桶的ID

            medium_barrel_id = self.sorted_barrel_ids[1]  # 中桶

            small_barrel_id = self.sorted_barrel_ids[2]   # 小桶

            

            # 重新排序：以最近的桶为起点，后面是另一个桶

            nearest_barrel_id = self.find_nearest_barrel_from_list([medium_barrel_id, small_barrel_id])

            if nearest_barrel_id == medium_barrel_id:

                self.sorted_barrel_ids = [medium_barrel_id, small_barrel_id]

            else:

                self.sorted_barrel_ids = [small_barrel_id, medium_barrel_id]

            

            self.current_target_index = 0

            self.get_logger().info(f"只处理中小桶，以最近的桶{nearest_barrel_id}为起点，桶顺序: {self.sorted_barrel_ids}")

            

            # 发送第一个桶的坐标给程序A，等待到位后再发送投水指令

            self.send_next_bucket_coordinate(nearest_barrel_id)

        else:

            self.get_logger().error("没有足够的中小桶数据")



    def reset_callback(self, msg):

        """重置桶管理器状态"""

        self.barrels = {}

        self.processed = False

        self.current_target_index = 0

        self.sorted_barrel_ids = []

        self.waiting_for_drop_success = False

        self.mission_completed = False

        self.current_mission_sequence = 0

        self.get_logger().info("桶管理器已重置，可以重新接收和处理桶数据")



    def barrel_callback(self, msg):

        try:

            # 如果已经处理过，直接返回

            if self.processed:

                return

                

            # 解析JSON数据（来自程序B）

            data = json.loads(msg.data)

            buckets = data['buckets']

            

            # 处理三个桶的数据

            for bucket in buckets:

                bucket_id = bucket['id']

                diameter = bucket['diameter']  # 真实直径，单位m

                height = bucket['height']      # 当前高度，单位m

                position = bucket['position']  # 位置坐标

                

                # 计算实际面积（从直径计算）

                real_area = math.pi * (diameter / 2) ** 2  # 面积 = π * r²

                

                self.barrels[bucket_id] = {

                    'diameter': diameter,

                    'position': (position['x'], position['y']),

                    'height': height,

                    'real_size': real_area

                }

                self.get_logger().info(f"桶{bucket_id}识别，位置：({position['x']:.2f}, {position['y']:.2f})，直径：{diameter:.3f}m，实际面积：{real_area:.6f}m²")

            

            # 检查是否收集到所有三个桶的数据

            if len(self.barrels) == 3:

                self.get_logger().info("已收集到所有三个桶的数据，开始处理...")

                self.process_barrels()

                self.processed = True  # 标记为已处理

                

                            # 开始执行投水任务（发送第一个桶的坐标给程序A）
                self.start_mission()

            else:

                self.get_logger().info(f"已收集到 {len(self.barrels)} 个桶的数据，等待更多数据...")

            

        except json.JSONDecodeError as e:

            self.get_logger().error(f'JSON解析错误: {e}')

        except KeyError as e:

            self.get_logger().error(f'JSON数据缺少字段: {e}')

        except Exception as e:

            self.get_logger().error(f'处理桶数据时出错: {e}')



    def process_barrels(self):

        # 按实际面积排序（从大到小）

        sorted_barrels = sorted(self.barrels.items(), key=lambda x: x[1]['real_size'], reverse=True)

        self.sorted_barrel_ids = [b[0] for b in sorted_barrels]  # 桶id按大小排序

        self.get_logger().info(f"桶大小顺序: {self.sorted_barrel_ids}")



        # 显示每个桶的详细信息

        for barrel_id, barrel_data in sorted_barrels:

            self.get_logger().info(f"桶{barrel_id}: 直径={barrel_data['diameter']:.3f}m, 面积={barrel_data['real_size']:.6f}m²")



        # 获取小桶和中桶的坐标（排序后索引1和2）

        small_barrel_id = self.sorted_barrel_ids[2]  # 最小的桶（索引2）

        medium_barrel_id = self.sorted_barrel_ids[1]  # 中等的桶（索引1）

        

        small_coords = self.barrels[small_barrel_id]['position']

        medium_coords = self.barrels[medium_barrel_id]['position']

        

        # 发布小桶和中桶的坐标

        coords_msg = String()

        coords_msg.data = f"small:{small_coords},medium:{medium_coords}"

        self.target_coords_publisher.publish(coords_msg)

        

        self.get_logger().info(f"小桶坐标: {small_coords}, 中桶坐标: {medium_coords}")



        # 发布桶顺序

        order_msg = Int32MultiArray()

        order_msg.data = self.sorted_barrel_ids

        self.order_publisher.publish(order_msg)



        # 保存结果（可写入文件或数据库）

        with open('/tmp/barrel_order.txt', 'w') as f:

            f.write(str(self.sorted_barrel_ids))



        # 发布扫描完成指令（true）

        self.finish_publisher.publish(Bool(data=True))



def main(args=None):

    rclpy.init(args=args)

    node = BarrelManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main() 