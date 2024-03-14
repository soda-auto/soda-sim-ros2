#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from pynput import keyboard
from numpy import clip

class AckermannDriveKeyop(Node):

    def __init__(self):
        super().__init__('ackermann_drive_keyop_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_speed', 25.0),
                ('max_steering_angle', 0.7),
                ('cmd_topic', '/wheel_loader/cmd_drive')
            ]
        )

        max_speed = self.get_parameter('max_speed').value
        max_steering_angle = self.get_parameter('max_steering_angle').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self.key_bindings = {
            'w': (1.0, 0.0),   # Forward
            's': (-1.0, 0.0),  # Backward
            'a': (0.0, -1.0),   # Left
            'd': (0.0, 1.0),  # Right
            keyboard.Key.space: (0.0, 'stop'), # Stop (reset speed)
            keyboard.Key.tab: ('reset', 0.0)   # Tab (reset steering angle)
        }

        for key in self.key_bindings:
            if isinstance(self.key_bindings[key][0], float):
                self.key_bindings[key] = (self.key_bindings[key][0] * max_speed / 5,
                                          self.key_bindings[key][1])
            if isinstance(self.key_bindings[key][1], float):
                self.key_bindings[key] = (self.key_bindings[key][0],
                                          self.key_bindings[key][1] * max_steering_angle / 5)

        self.speed_range = [-max_speed, max_speed]
        self.steering_angle_range = [-max_steering_angle, max_steering_angle]

        self.speed = 0.0
        self.steering_angle = 0.0
        self.motors_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self.timer = self.create_timer(0.2, self.pub_callback)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.print_state()

    def pub_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.motors_pub.publish(msg)

    def print_state(self):
        self.get_logger().info(
            f'Speed: {self.speed:.2f} m/s, Steering Angle: {self.steering_angle:.2f} rad')

    def on_press(self, key):
        try:
            if key.char in self.key_bindings:
                self.update_movement(key.char)
        except AttributeError:
            if key in self.key_bindings:
                self.update_movement(key)
            elif key == keyboard.Key.esc:
                self.listener.stop()
                self.finalize()

    def update_movement(self, key):
        if key == keyboard.Key.space:
            self.speed = 0.0
        elif key == keyboard.Key.tab:
            self.steering_angle = 0.0
        elif key in self.key_bindings:
            speed_change, angle_change = self.key_bindings[key]
            if isinstance(speed_change, float):
                self.speed = clip(self.speed + speed_change, self.speed_range[0], self.speed_range[1])
            if isinstance(angle_change, float):
                self.steering_angle = clip(self.steering_angle + angle_change, self.steering_angle_range[0], self.steering_angle_range[1])
        self.print_state()

    def finalize(self):
        self.get_logger().info('Exiting...')
        msg = AckermannDriveStamped()
        msg.drive.speed = 0
        msg.drive.steering_angle = 0
        self.motors_pub.publish(msg)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveKeyop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()

if __name__ == '__main__':
    main()
