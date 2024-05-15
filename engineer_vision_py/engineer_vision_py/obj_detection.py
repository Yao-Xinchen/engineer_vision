import rclpy
import os
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from geometry_msgs.msg import Vector3

# Load YOLO model
model = YOLO(os.path.expanduser("~/Proj/runs/detect/train21/weights/best.pt"))

# Camera intrinsic parameters (from the YAML file)
fx = 1552.7
fy = 1537.9
cx = 640.0
cy = 360.0
EDGE_SIZE = 0.2  # Known cube size in meters


class ObjDetector(Node):
    def __init__(self):
        super().__init__("camera_subscriber_obj_detection")
        self.publisher_ = self.create_publisher(Vector3, "ore_position", 10)
        self.subscription = self.create_subscription(
            Image, "image_raw", self.listener_callback, 10
        )
        self.get_logger().info("Start listening messages from 'image_raw'")

    def listener_callback(self, msg):
        img_arr = np.array(msg.data).reshape((msg.height, msg.width, 3))
        result = model(img_arr)[0]

        # Return if no object is detected
        if result.boxes.xyxy.numel() <= 0:
            return

        self.get_logger().info("-" * 80)
        self.get_logger().info(f"result.boxes.xyxy: {result.boxes.xyxy}")

        for box in result.boxes.xyxy:
            x1, y1, x2, y2 = box
            pixel_width = x2 - x1
            pixel_height = y2 - y1

            # Assuming cube is facing camera squarely
            pixel_size = max(pixel_width, pixel_height)

            # Calculate real-world distance Z using similar triangles
            Z = (EDGE_SIZE * fx) / pixel_size

            # Calculate real-world coordinates X, Y (center of the bounding box)
            X = ((x1 + x2) / 2 - cx) * Z / fx
            Y = ((y1 + y2) / 2 - cy) * Z / fy

            self.get_logger().info(f"X: {X:.2f}m, Y: {Y:.2f}m, Z: {Z:.2f}m")

            msg = Vector3()
            msg.x = float(X);
            msg.y = float(Y);
            msg.z = float(Z);
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    subscriber = ObjDetector()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
