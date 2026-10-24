#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Motion(Node):
    """Mengikuti daftar waypoint dan mengirim v, omega ke /target_velocity.

    Kecepatan roda dihitung oleh node inverse_kinematic.
    """

    def __init__(self):
        super().__init__('motion')

        # Daftar waypoint datar [x1, y1, x2, y2, ...] dalam frame odom
        self.declare_parameter('waypoints', [0.0, 0.0])
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('min_angular_speed', 0.25)
        self.declare_parameter('k_linear', 1.0)
        self.declare_parameter('k_angular', 3.2)
        self.declare_parameter('goal_tolerance', 0.04)
        # Putar di tempat jika heading error > rotate_threshold, lalu tetap
        # berputar sampai error < align_tolerance sebelum maju lagi
        self.declare_parameter('rotate_threshold', 0.70)
        self.declare_parameter('align_tolerance', 0.08)
        self.declare_parameter('max_linear_accel', 0.8)
        self.declare_parameter('max_angular_accel', 4.5)
        # Segmen lurus (panjang >= straight_min_length) memakai batas sendiri
        self.declare_parameter('straight_linear_speed', 1.1)
        self.declare_parameter('straight_min_length', 1.0)
        self.declare_parameter('lookahead_distance', 0.40)
        self.declare_parameter('rate', 50.0)

        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = list(zip(flat[0::2], flat[1::2]))

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.k_linear = self.get_parameter('k_linear').value
        self.k_angular = self.get_parameter('k_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.rotate_threshold = self.get_parameter('rotate_threshold').value
        self.align_tolerance = self.get_parameter('align_tolerance').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.straight_linear_speed = self.get_parameter(
            'straight_linear_speed'
        ).value
        self.straight_min_length = self.get_parameter(
            'straight_min_length'
        ).value
        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value

        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate

        self.pose = None
        self.v = 0.0
        self.omega = 0.0
        self.index = 0
        self.segment_start = None
        self.rotating = False
        self.finished = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.velocity_publisher = self.create_publisher(
            Twist, '/target_velocity', 10
        )

        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Motion aktif dengan {len(self.waypoints)} waypoint.'
        )

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)
        if self.segment_start is None:
            self.segment_start = self.pose[:2]

    def publish_velocity(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.velocity_publisher.publish(msg)

    def command_velocity(self, v, omega):
        # Batasi percepatan agar roda tidak slip (slip merusak odometri roda)
        dv = self.max_linear_accel * self.dt
        dw = self.max_angular_accel * self.dt
        self.v += clamp(v - self.v, dv)
        self.omega += clamp(omega - self.omega, dw)
        self.publish_velocity(self.v, self.omega)

    def control_loop(self):
        if self.pose is None or self.finished:
            return

        if self.index >= len(self.waypoints):
            self.command_velocity(0.0, 0.0)
            if self.v != 0.0 or self.omega != 0.0:
                return
            self.finished = True
            x, y, _ = self.pose
            self.get_logger().info(
                f'Semua waypoint selesai. Posisi akhir ({x:.2f}, {y:.2f}).'
            )
            return

        x, y, theta = self.pose
        goal_x, goal_y = self.waypoints[self.index]
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)

        seg_x = goal_x - self.segment_start[0]
        seg_y = goal_y - self.segment_start[1]
        segment_length = math.hypot(seg_x, seg_y)

        # Kecepatan saat melewati waypoint: tetap melaju jika belokan ke
        # segmen berikutnya landai (misalnya rintangan setengah lingkaran < 0.70 rad).
        # Belokan siku (90 deg) berhenti penuh dan putar di tempat agar tidak menabrak tembok.
        corner_speed = 0.0
        if self.index + 1 < len(self.waypoints) and segment_length > 0.0:
            next_x, next_y = self.waypoints[self.index + 1]
            turn = normalize_angle(
                math.atan2(next_y - goal_y, next_x - goal_x)
                - math.atan2(seg_y, seg_x)
            )
            if abs(turn) < self.rotate_threshold:
                corner_speed = self.max_linear_speed

        # Waypoint lintasan dianggap tercapai begitu robot melewatinya (untuk belokan mengalir)
        passed = corner_speed > 0.0 and dx * seg_x + dy * seg_y <= 0.0

        if distance < self.goal_tolerance or passed:
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(self.waypoints)} '
                f'({goal_x:.2f}, {goal_y:.2f}) tercapai.'
            )
            self.segment_start = (goal_x, goal_y)
            self.index += 1
            return

        # Path tracking: gunakan lookahead point di sepanjang garis segmen
        # agar robot selalu tertarik ke tengah lorong (mencegah jalan miring)
        if segment_length > 0.05:
            rx = x - self.segment_start[0]
            ry = y - self.segment_start[1]
            s = (rx * seg_x + ry * seg_y) / segment_length
            target_s = min(segment_length, max(0.0, s) + self.lookahead_distance)
            target_x = self.segment_start[0] + (target_s / segment_length) * seg_x
            target_y = self.segment_start[1] + (target_s / segment_length) * seg_y
            target_heading = math.atan2(target_y - y, target_x - x)
        else:
            target_heading = math.atan2(dy, dx)

        heading_error = normalize_angle(target_heading - theta)
        omega = clamp(self.k_angular * heading_error, self.max_angular_speed)

        if abs(heading_error) > self.rotate_threshold:
            self.rotating = True
        elif abs(heading_error) < self.align_tolerance:
            self.rotating = False

        if self.rotating:
            # Putar di tempat tepat di atas waypoint (v = 0.0 agar tidak maju menabrak tembok)
            if abs(omega) < self.min_angular_speed:
                omega = math.copysign(self.min_angular_speed, omega)
            self.command_velocity(0.0, omega)
            return

        max_speed = self.max_linear_speed
        if segment_length >= self.straight_min_length:
            max_speed = self.straight_linear_speed

        # Batas pengereman agar robot sempat melambat ke corner_speed
        # Menggunakan perlambatan terukur aman dari slip roda
        safe_decel = self.max_linear_accel * 0.75
        brake_speed = math.sqrt(
            corner_speed ** 2 + 2.0 * safe_decel * distance
        )
        approach_speed = max(self.k_linear * distance, corner_speed)
        v = min(max_speed, approach_speed, brake_speed)

        # Perlambatan proporsional jika heading belum lurus sempurna (anti-tabrak)
        heading_factor = max(0.2, math.cos(heading_error))
        v *= heading_factor

        v = max(v, self.min_linear_speed)
        self.command_velocity(v, omega)


def main(args=None):
    rclpy.init(args=args)
    node = Motion()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
