import rclpy
import os
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# Load YOLO model
model = YOLO(os.path.expanduser("~/Proj/runs/detect/train20/weights/best.pt"))
detected_cnt = 0  # count of obj is recognition


class Subscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber_obj_detection")
        self.subscription = self.create_subscription(
            Image, "image_raw", self.listener_callback, 10
        )
        self.get_logger().info("Start listening messages from 'image_raw'")

    def listener_callback(self, msg):
        global model
        global detected_cnt

        # # TEST
        # global img_saved
        # if img_saved:
        #     return
        # img_saved = True

        # # TEST
        # self.get_logger().info("---------------------------------------------")
        # self.get_logger().info(f"image height: {msg.height}")
        # self.get_logger().info(f"image width: {msg.width}")
        # self.get_logger().info(f"image step: {msg.step}")
        # self.get_logger().info(f"image data length: {len(msg.data)}")

        # # TEST: save the array as image
        # img_arr = np.array(msg.data).reshape((msg.height, msg.width, 3))
        # img = pil.Image.fromarray(img_arr, "RGB")
        # img.save("img.png")

        img_arr = np.array(msg.data).reshape((msg.height, msg.width, 3))
        result = model(img_arr)[0]

        # TEST
        # Save the result when an object is detected
        # 'Boxes' object can be used to index, manipulate,and convert bounding
        # boxes to different formats
        # 'boxes.xyxy' return the boxes in xyxy format
        # 'Probs' object contains the scores of classification
        if result.probs or result.boxes.xyxy.numel() > 0:
            print("-" * 80)
            self.get_logger().debug(f"detection {detected_cnt}:")
            self.get_logger().debug(f"result.probs: {result.probs}")
            self.get_logger().debug(f"result.boxes: {result.boxes}")

            result.save(f"detection_{detected_cnt}.png")
            detected_cnt += 1


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
