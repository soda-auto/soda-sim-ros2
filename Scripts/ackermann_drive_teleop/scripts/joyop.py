#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import sys

class AckermannDriveJoyop(Node):

    def __init__(self, args):
        super().__init__('ackermann_drive_joyop_node')


        self.publish_frequency = 100.0  
        self.max_speed = 25.0
        self.max_steering_angle = 0.7
        cmd_topic = '/wheel_loader/cmd_drive'

        self.speed = 0.0
        self.steering_angle = 0.0

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            QoSProfile(depth=10))
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            cmd_topic,
            QoSProfile(depth=10))


        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)

        self.get_logger().info('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg):
        self.speed = (1 - joy_msg.axes[5]) / 2 * self.max_speed
        self.steering_angle = joy_msg.axes[0] * self.max_steering_angle * -1

    def timer_callback(self):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        self.drive_publisher.publish(ackermann_cmd_msg)
        self.print_state()

    def print_state(self):
        self.get_logger().info(
            'Speed: {:.2f} m/s, Steering Angle: {:.2f} rad'.format(
                self.speed, self.steering_angle))

    def finalize(self):
        self.get_logger().info('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.drive_publisher.publish(ackermann_cmd_msg)
        sys.exit()

def main(args=None):
    rclpy.init(args=args)

    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])

    try:
        rclpy.spin(joyop)
    except KeyboardInterrupt:
        pass
    finally:
        joyop.finalize()
        joyop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
