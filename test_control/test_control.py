import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np

import time
import random

def euler_to_quaternion(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

class TestNode(Node):
  def __init__(self):
    super().__init__('test_node')

    # create tracks publisher
    self._publisher = self.create_publisher(
      PoseStamped,
      'cmd_abs_move',
      10
    )

    self._timer = self.create_timer(5, self.send_command) # update every 0.5 sec

  def send_command(self):
    msg = PoseStamped()
    msg.header.stamp = self.get_clock().now().to_msg() # new timestamp

    pan = random.uniform(0, 2*np.pi)
    tilt = random.uniform(0, np.pi/2)

    self.get_logger().info('%f, %f'%(pan, tilt))

    q = euler_to_quaternion(0, tilt, pan)

    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    self._publisher.publish(msg)

def main():
  rclpy.init()
  try:
    node = TestNode()
    try:
      rclpy.spin(node)
    finally:
      node.destroy_node()
  finally:
    rclpy.shutdown()

if __name__ == '__main__':
  main()