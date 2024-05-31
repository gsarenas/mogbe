#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])
        
        self.get_logger().info("Line follower node has been started.")
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert from ROS Image message to CV Image: {e}')
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        h, w, _ = cv_image.shape
        search_top = int(3 * h / 4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        M = cv2.moments(mask)
        
        twist = Twist()
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            err = cx - w / 2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.publisher_.publish(twist)
        
        cv2.imshow("Original Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
