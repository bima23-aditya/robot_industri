#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class InverseKinematic(Node):
    """Kinematika balik differential drive.

    Input  : /target_velocity (Twist) -> v [m/s] dan omega [rad/s] robot.
    Output : /wheel/target_speed [kiri, kanan] dalam rad/s,
             /cmd_vel untuk plugin DiffDrive Gazebo.
    """

    def __init__(self):
        super().__init__('inverse_kinematic')

        # Geometri robot asli (sesuai URDF robin_description)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.17)
        # Geometri yang dipakai plugin DiffDrive (mobile_base_gazebo.xacro)
        self.declare_parameter('plugin_wheel_radius', 0.1)
        self.declare_parameter('plugin_wheel_separation', 0.45)
        self.declare_parameter('max_wheel_speed', 15.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.plugin_wheel_radius = self.get_parameter(
            'plugin_wheel_radius'
        ).value
        self.plugin_wheel_separation = self.get_parameter(
            'plugin_wheel_separation'
        ).value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value

        self.create_subscription(
            Twist, '/target_velocity', self.target_velocity_callback, 10
        )

        self.wheel_target_publisher = self.create_publisher(
            Float64MultiArray, '/wheel/target_speed', 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Inverse Kinematic aktif.')

    def compute_wheel_speed(self, v, omega):
        r = self.wheel_radius
        half_l = self.wheel_separation / 2.0

        left_speed = (v - omega * half_l) / r
        right_speed = (v + omega * half_l) / r

        # Saturasi dengan menjaga rasio kiri/kanan agar arah gerak tetap
        peak = max(abs(left_speed), abs(right_speed))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            left_speed *= scale
            right_speed *= scale

        return left_speed, right_speed

    def target_velocity_callback(self, msg):
        left_speed, right_speed = self.compute_wheel_speed(
            msg.linear.x, msg.angular.z
        )

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [float(left_speed), float(right_speed)]
        self.wheel_target_publisher.publish(wheel_msg)

        # Plugin DiffDrive hanya menerima Twist dan menghitung ulang kecepatan
        # roda dengan geometrinya sendiri. Twist dibentuk dari kecepatan roda
        # hasil IK sehingga roda di Gazebo berputar tepat sesuai hasil IK.
        rp = self.plugin_wheel_radius
        cmd = Twist()
        cmd.linear.x = rp * (right_speed + left_speed) / 2.0
        cmd.angular.z = (
            rp * (right_speed - left_speed) / self.plugin_wheel_separation
        )
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematic()

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
