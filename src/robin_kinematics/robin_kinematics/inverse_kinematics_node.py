from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def inverse_kinematics(v_y, omega, wheel_separation, wheel_radius):
    """Hitung kecepatan sudut roda kiri dan kanan (rad/s)."""
    r = wheel_radius
    s = wheel_separation

    # Matriks inverse kinematics J^-1
    ik_matrix = np.array([
        [1.0 / r, -s / (2.0 * r)],
        [1.0 / r, s / (2.0 * r)],
    ])
    # Vektor kecepatan robot [Vy, w]^T
    robot_vel = np.array([v_y, omega])

    phi_l, phi_r = ik_matrix @ robot_vel
    return phi_l, phi_r


class InverseKinematicsNode(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_node')

        self.declare_parameter('wheel_separation', 0.17)  # s (m)
        self.declare_parameter('wheel_radius', 0.03)  # r (m)
        # Roda dihentikan jika cmd_vel tidak diterima selama waktu ini
        self.declare_parameter('cmd_timeout', 0.5)  # detik

        self.wheel_separation = self.get_parameter(
            'wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.left_pub = self.create_publisher(
            Float64, '/left_wheel/command', 10)
        self.right_pub = self.create_publisher(
            Float64, '/right_wheel/command', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.last_cmd_time = None
        self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f'Inverse kinematics aktif: s = {self.wheel_separation} m, '
            f'r = {self.wheel_radius} m')

    def cmd_vel_callback(self, msg):
        v_y = msg.linear.x
        omega = msg.angular.z
        phi_l, phi_r = inverse_kinematics(
            v_y, omega, self.wheel_separation, self.wheel_radius)

        self.publish_wheels(phi_l, phi_r)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().debug(
            f'Vy = {v_y:.3f} m/s, w = {omega:.3f} rad/s -> '
            f'phi_L = {phi_l:.3f} rad/s, phi_R = {phi_r:.3f} rad/s')

    def watchdog_callback(self):
        if self.last_cmd_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds
        if elapsed * 1e-9 > self.cmd_timeout:
            self.publish_wheels(0.0, 0.0)
            self.last_cmd_time = None
            self.get_logger().info('cmd_vel timeout, roda dihentikan.')

    def publish_wheels(self, phi_l, phi_r):
        self.left_pub.publish(Float64(data=float(phi_l)))
        self.right_pub.publish(Float64(data=float(phi_r)))


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
