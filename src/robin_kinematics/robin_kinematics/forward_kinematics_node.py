"""
Forward kinematics differential drive: (phi_L, phi_R) -> (Vy, w).

Rumus (koordinat lokal robot, Vy = arah maju):
    Vy = (r / 2) * (phi_L + phi_R)
    w  = (r / s) * (-phi_L + phi_R)

Kecepatan roda dibaca dari /joint_states, hasilnya dipublish sebagai
geometry_msgs/Twist (linear.x = Vy, angular.z = w) pada topic robot_velocity.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def forward_kinematics(phi_l, phi_r, wheel_separation, wheel_radius):
    """Hitung kecepatan linear (m/s) dan angular (rad/s) robot."""
    v_y = (wheel_radius / 2.0) * (phi_l + phi_r)
    omega = (wheel_radius / wheel_separation) * (-phi_l + phi_r)
    return v_y, omega


class ForwardKinematicsNode(Node):

    def __init__(self):
        super().__init__('forward_kinematics_node')

        self.declare_parameter('wheel_separation', 0.17)  # s (m)
        self.declare_parameter('wheel_radius', 0.03)  # r (m)
        self.declare_parameter('left_wheel_joint', 'base_left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'base_right_wheel_joint')

        self.wheel_separation = self.get_parameter(
            'wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.left_joint = self.get_parameter('left_wheel_joint').value
        self.right_joint = self.get_parameter('right_wheel_joint').value

        self.velocity_pub = self.create_publisher(
            Twist, 'robot_velocity', 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        # Untuk menghitung kecepatan dari selisih posisi jika field
        # velocity pada JointState kosong
        self.prev_positions = None
        self.prev_stamp = None

        self.get_logger().info(
            f'Forward kinematics aktif: s = {self.wheel_separation} m, '
            f'r = {self.wheel_radius} m')

    def joint_states_callback(self, msg):
        if self.left_joint not in msg.name or self.right_joint not in msg.name:
            return
        i_l = msg.name.index(self.left_joint)
        i_r = msg.name.index(self.right_joint)

        if len(msg.velocity) == len(msg.name):
            phi_l = msg.velocity[i_l]
            phi_r = msg.velocity[i_r]
        else:
            wheel_speeds = self.speeds_from_positions(msg, i_l, i_r)
            if wheel_speeds is None:
                return
            phi_l, phi_r = wheel_speeds

        v_y, omega = forward_kinematics(
            phi_l, phi_r, self.wheel_separation, self.wheel_radius)

        twist = Twist()
        twist.linear.x = v_y
        twist.angular.z = omega
        self.velocity_pub.publish(twist)

        self.get_logger().debug(
            f'phi_L = {phi_l:.3f} rad/s, phi_R = {phi_r:.3f} rad/s -> '
            f'Vy = {v_y:.3f} m/s, w = {omega:.3f} rad/s')

    def speeds_from_positions(self, msg, i_l, i_r):
        positions = (msg.position[i_l], msg.position[i_r])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        wheel_speeds = None
        if self.prev_positions is not None:
            dt = stamp - self.prev_stamp
            if dt > 0.0:
                wheel_speeds = (
                    (positions[0] - self.prev_positions[0]) / dt,
                    (positions[1] - self.prev_positions[1]) / dt,
                )

        self.prev_positions = positions
        self.prev_stamp = stamp
        return wheel_speeds


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
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
