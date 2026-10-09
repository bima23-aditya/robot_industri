import math
import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class RectangleMoverNode(Node):

  def __init__(self):
    super().__init__('rectangle_mover_node')

    self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    # Definisi Kecepatan
    self.linear_speed = 0.5  # m/s
    self.angular_speed = 0.5  # rad/s

    # Perhitungan Durasi
    self.t_move_4 = 4.0 / self.linear_speed  # 8.0 detik
    self.t_move_2 = 2.0 / self.linear_speed  # 4.0 detik
    self.t_turn_90 = (math.pi / 2.0) / self.angular_speed  # ~3.1416 detik

    # Urutan Fase Gerakan (Kumulatif Waktu)
    self.t1 = self.t_move_4
    self.t2 = self.t1 + self.t_turn_90
    self.t3 = self.t2 + self.t_move_2
    self.t4 = self.t3 + self.t_turn_90
    self.t5 = self.t4 + self.t_move_4
    self.t6 = self.t5 + self.t_turn_90
    self.t7 = self.t6 + self.t_move_2
    self.t8 = self.t7 + self.t_turn_90

    self.start_time = time.time()
    self.timer = self.create_timer(0.1, self.timer_callback)

  def timer_callback(self):
    msg = Twist()
    elapsed_time = time.time() - self.start_time

    if elapsed_time < self.t1:
      msg.linear.x = self.linear_speed
      msg.angular.z = 0.0
      self.get_logger().info('Fase 1: Maju 4 petak...')
    elif elapsed_time < self.t2:
      msg.linear.x = 0.0
      msg.angular.z = self.angular_speed
      self.get_logger().info('Fase 2: Rotasi Kiri 90°...')
    elif elapsed_time < self.t3:
      msg.linear.x = self.linear_speed
      msg.angular.z = 0.0
      self.get_logger().info('Fase 3: Maju 2 petak...')
    elif elapsed_time < self.t4:
      msg.linear.x = 0.0
      msg.angular.z = self.angular_speed
      self.get_logger().info('Fase 4: Rotasi Kiri 90°...')
    elif elapsed_time < self.t5:
      msg.linear.x = self.linear_speed
      msg.angular.z = 0.0
      self.get_logger().info('Fase 5: Maju 4 petak...')
    elif elapsed_time < self.t6:
      msg.linear.x = 0.0
      msg.angular.z = self.angular_speed
      self.get_logger().info('Fase 6: Rotasi Kiri 90°...')
    elif elapsed_time < self.t7:
      msg.linear.x = self.linear_speed
      msg.angular.z = 0.0
      self.get_logger().info('Fase 7: Maju 2 petak...')
    elif elapsed_time < self.t8:
      msg.linear.x = 0.0
      msg.angular.z = self.angular_speed
      self.get_logger().info('Fase 8: Rotasi Kiri 90°...')
    else:
      # Berhenti total dan matikan node
      msg.linear.x = 0.0
      msg.angular.z = 0.0
      self.publisher_.publish(msg)
      self.get_logger().info('Selesai membuat persegi panjang. Berhenti.')

      self.timer.cancel()
      rclpy.shutdown()
      return

    self.publisher_.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = RectangleMoverNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if rclpy.ok():
      node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
  main()
