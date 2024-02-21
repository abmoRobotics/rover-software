#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pseudo_camera')
        self.publisher_ = self.create_publisher(Image, 'camera_raw', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.img = cv2.imread('rover-software/perception_stack/scripts/markertest_delete.jpg', cv2.IMREAD_COLOR)
        self.bridge = CvBridge()
    def timer_callback(self):
        msg = Image()

        msg= self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()