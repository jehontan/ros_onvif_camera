import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
import socket
import struct

import time
import random

from geometry_helper.geometry_helper import *

class TestNode(Node):
  def __init__(self):
    super().__init__('test_node')

    # create tracks publisher
    self._publisher = self.create_publisher(
      PoseStamped,
      'cmd_abs_move',
      10
    )

    self.serve()

  def send_command(self, pan, tilt, zoom):
    """
    Send PTZ command.
    
    Params:
      pan - Pan angle in degrees
      tilt - Tilt angle in degrees
      zoom - Zoom  [0.0, 1.0]
    """
    msg = PoseStamped()
    msg.header.stamp = self.get_clock().now().to_msg() # new timestamp

    p = pan/180*np.pi
    t = tilt/180*np.pi
    self.get_logger().info('%f, %f, %f'%(pan, tilt, zoom))

    q = euler_to_quaternion(0, t, p)

    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    msg.pose.position.z = max(min(zoom, 1.0), 0.0)

    self._publisher.publish(msg)

  def serve(self):
    HOST = '192.168.137.216'
    PORT = 2306

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.bind((HOST, PORT))
      try:
        s.listen()
        
        while True:
          self.get_logger().info('Listening on %s:%d'%(HOST, PORT))
          conn, addr = s.accept()
          with conn:
            self.get_logger().info('Connected to %s:%d'%(addr[0], addr[1]))
            while True:
              cmd = conn.recv(12) # read 12 bytes (float32 pan, float32 tilt, float32 zoom)
              if cmd:
                pan, tilt, zoom = struct.unpack('fff', cmd)
                self.send_command(pan, tilt, zoom)
      finally:
        s.close()

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