#! /usr/bin/env python3

'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        timer_period = 1.0 /30.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.1  # Adjust linear speed as needed
        twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info('Publishing: {}'.format(twist))

    
    def joint_states_callback(self, msg):
        if 'left_wheel_joint' in msg.name and 'right_wheel_joint' in msg.name:

            # Checking order of wheels in msg.name (e.g.: msg.name = ['left_wheel_joint', 'right_wheel_joint'])
            left_wheel_index = msg.name.index('left_wheel_joint')
            right_wheel_index = msg.name.index('right_wheel_joint')
            
            left_wheel_position = msg.position[left_wheel_index]
            right_wheel_position = msg.position[right_wheel_index]
            
            self.get_logger().info("Left Wheel Position: {:.15f} Right Wheel Position: {:.15f}".format(left_wheel_position, right_wheel_position))


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()

    rclpy.spin(move_robot)

    move_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.subscription  # prevent unused variable warning
        self.current_distance_1, self.current_distance_2 = 0.0, 0.0
        self.initial_position_1, self.initial_position_2 = 0.0, 0.0
        self.target_distance_1, self.target_distance_2 = 2.0 / 0.02075, 2.0 / 0.02075 # Relative distance to travel
        self.linear_speed = 0.1  # Fixed linear speed
        self.is_started = False

    def joint_states_callback(self, msg):
        # Assuming msg.position[] contains the positions of the encoders for the two wheels
        # You may need to adjust this based on your actual message structure
        if len(msg.position) >= 2:
            wheel1_position = msg.position[0]
            wheel2_position = msg.position[1]
            # Calculate current distance traveled based on encoder positions
            self.current_distance_1, self.current_distance_2 = wheel1_position, wheel2_position
            if not self.is_started:
                self.initial_position_1 = self.current_distance_1
                self.initial_position_2 = self.current_distance_2
                self.target_distance_1 += self.initial_position_1
                self.target_distance_2 += self.initial_position_2
                self.is_started = True

    def run_motors(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        while self.current_distance_1 < self.target_distance_1 and self.current_distance_2 < self.target_distance_2:
            self.publisher_.publish(twist_msg)
            #self.get_logger().info(f"Current Distance: {self.current_distance}")
            self.get_logger().info("Moving...")
            #self.get_logger().info("Target Distance: {}".format(self.target_distance))
            rclpy.spin_once(self)

        self.get_logger().info("Target Distance Reached!")
        # Stop the motors
        twist_msg.linear.x = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    motor_control_node.run_motors()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
