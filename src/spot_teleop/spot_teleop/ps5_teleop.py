#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import time

class SpotPs5Teleop(Node):

    def __init__(self):
        super().__init__('spot_ps5_teleop')

        self.axis_left_horizontal = 0
        self.axis_left_vertical = 1
        self.axis_right_horizontal = 3

        self.button_enable = 4 # L1

        # Command limits
        self.max_vx = 0.65
        self.max_vy = 0.40
        self.max_wz = 0.60

        self.deadzone = 0.08

        self.latest_joy = None

        self.latest_joy_time = None
        self.joy_timeout = 0.25

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        # Publish commands at the same rate as the PPO policy.
        self.timer = self.create_timer(
            1.0 / 50.0,
            self.publish_command,
        )

        self.get_logger().info(
            "PS5 Spot teleop ready. Hold L1 to enable motion."
        )

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0

        return float(value)

    def joy_callback(self, msg):
        self.latest_joy = msg
        self.latest_joy_time = time.monotonic()

    def publish_command(self):
        cmd = Twist()

        if self.latest_joy is None or self.latest_joy_time is None:
            self.publisher.publish(cmd)
            return

        if time.monotonic() - self.latest_joy_time > self.joy_timeout:
            self.publisher.publish(cmd)
            return

        if len(self.latest_joy.buttons) <= self.button_enable:
            self.publisher.publish(cmd)
            return

        enabled = bool(
            self.latest_joy.buttons[self.button_enable]
        )

        if not enabled:
            self.publisher.publish(cmd)
            return

        axes = self.latest_joy.axes

        required_axis = max(
            self.axis_left_horizontal,
            self.axis_left_vertical,
            self.axis_right_horizontal,
        )

        if len(axes) <= required_axis:
            self.publisher.publish(cmd)
            return

        left_horizontal = self.apply_deadzone(
            axes[self.axis_left_horizontal]
        )

        left_vertical = self.apply_deadzone(
            axes[self.axis_left_vertical]
        )

        right_horizontal = self.apply_deadzone(
            axes[self.axis_right_horizontal]
        )

        cmd.linear.x = self.max_vx * left_vertical
        cmd.linear.y = self.max_vy * left_horizontal
        cmd.angular.z = self.max_wz * right_horizontal

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = SpotPs5Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        node.publisher.publish(zero)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()