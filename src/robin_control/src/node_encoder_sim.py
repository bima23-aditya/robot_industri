#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class SensorEncoder(Node):
    def __init__(self):
        super().__init__('sensor_encoder')

        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.wheel_speed_publisher = self.create_publisher(
            Float64MultiArray, '/wheel/speed', 10
        )

        self.get_logger().info('Sensor Encoder aktif.')

    def joint_state_callback(self, msg):
        speed = dict(zip(msg.name, msg.velocity))

        right_speed = speed.get('base_right_wheel_joint')
        left_speed = speed.get('base_left_wheel_joint')

        if right_speed is None or left_speed is None:
            self.get_logger().warn(
                'Data kecepatan roda kiri atau kanan tidak ditemukan.'
            )
            return

        msg = Float64MultiArray()
        msg.data = [float(left_speed), float(right_speed)]
        self.wheel_speed_publisher.publish(msg)

        # self.get_logger().info(
        #     f'Kiri: {left_speed:.3f} rad/s, '
        #     f'kanan: {right_speed:.3f} rad/s'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = SensorEncoder()

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