#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelOdometry(Node):
    """Kinematika maju: kecepatan roda (/wheel/speed) -> pose robot (/odom)."""

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('rate', 100.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_time = None
        self.has_encoder = False

        self.create_subscription(
            Float64MultiArray, '/wheel/speed', self.wheel_speed_callback, 10
        )

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self.update)

        self.get_logger().info('Wheel Odometry aktif.')

    def wheel_speed_callback(self, msg):
        if len(msg.data) < 2:
            return

        left_speed, right_speed = msg.data[0], msg.data[1]
        r = self.wheel_radius

        self.v = r * (right_speed + left_speed) / 2.0
        self.omega = r * (right_speed - left_speed) / self.wheel_separation
        self.has_encoder = True

    def update(self):
        if not self.has_encoder:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # Integrasi midpoint agar lebih akurat saat robot berbelok
        mid_theta = self.theta + self.omega * dt / 2.0
        self.x += self.v * math.cos(mid_theta) * dt
        self.y += self.v * math.sin(mid_theta) * dt
        self.theta = math.atan2(
            math.sin(self.theta + self.omega * dt),
            math.cos(self.theta + self.omega * dt),
        )

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.omega
        self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
