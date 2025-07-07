import warnings
warnings.simplefilter('ignore', category=FutureWarning)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32, Bool
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
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)

        # 发布给成员 A 的通知话题
        self.trigger_pub = self.create_publisher(Bool, '/nav_trigger', 10)

        # 模型加载（YOLOv8）
        self.model = YOLO('/home/weights/best.engine')

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.current_pose = None

        self.detecting = False
        self.frame_buffer = deque(maxlen=10)
        self.found_positions = []
        self.bucket_data = []
        self.recording = False

        self.get_logger().info("YOLOv8 Bucket detector initialized")
        cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.depth_image is not None:
            self.process()

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def process(self):
        detections = self.detect()
        h, w = self.color_image.shape[:2]
        fx = 605.7783

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            if conf < 0.85:
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            if cy < h * 0.25 or cy > h * 0.75:
                continue

            depth = self.depth_image[cy, cx] * 0.001
            if depth <= 0:
                continue

            # 显示检测框
            cv2.rectangle(self.color_image, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(self.color_image, f"{conf:.2f}",
                        (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if not self.detecting:
                self.get_logger().info("High conf bucket detected, notifying Member A")
                self.trigger_pub.publish(Bool(data=True))
                self.detecting = True

            self.frame_buffer.append((x2 - x1, depth))
            if len(self.frame_buffer) == self.frame_buffer.maxlen:
                self.finalize_bucket(cx, cy)
                break

        # ────── 一定要和上面同级缩进 ──────
        cv2.imshow("Color Detection", self.color_image)
        cv2.waitKey(1)


    def finalize_bucket(self, cx, cy):
        if not self.current_pose:
            return
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        for old in self.found_positions:
            if math.hypot(px - old[0], py - old[1]) < 1.0:
                self.get_logger().info("Duplicate bucket skipped")
                return

        self.found_positions.append((px, py))

        widths, depths = zip(*self.frame_buffer)
        avg_width = sum(widths) / len(widths)
        avg_depth = sum(depths) / len(depths)
        real_diameter = avg_width * avg_depth / 605.7783

        self.bucket_data.append({
            'pixel_width': avg_width,
            'depth': avg_depth,
            'diameter': real_diameter,
            'pose': self.current_pose
        })

        self.get_logger().info(f"Bucket recorded: diameter={real_diameter:.3f}, depth={avg_depth:.2f}")

        self.detecting = False
        self.frame_buffer.clear()

        if len(self.bucket_data) == 3:
            self.get_logger().info("3 buckets recorded, ready for Member C")
            # TODO: 发布消息 self.bucket_pub.publish(...) 可自定义消息
        else:
            self.trigger_pub.publish(Bool(data=True))  # 继续飞行

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
