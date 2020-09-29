import argparse

import rclpy
from rclpy.node import Node
from onvif import ONVIFCamera

import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, QuarternionStamped

class CameraNode(Node):
  def __init__(self, name:str, addr:str, port:int, user:str, pwd:str):
    super().__init__(name, namespace=name)

    self.name = name

    # setup PTZ
    self._camera = ONVIFCamera(addr, port, user, pwd)
    self._media = self._camera.create_media_service()
    self._ptz = self._camera.create_ptz_service()

    media_profile = self._media.GetProfiles()[0]
    
    # get PTZ move range
    request = self._ptz.create_type('GetConfigurationOptions')
    request.ConfigurationToken = media_profile.PTZConfiguration.token
    ptz_configuration_options = self._ptz.GetConfigurationOptions(request)

    XMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Max
    XMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Min
    YMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Max
    YMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Min
    # TODO: set limits as ROS params

    

    # setup move_request template
    self._move_request = self._ptz.create_type('ContinuousMove')
    self._move_request.ProfileToken = media_profile.token

    # setup transform boradcaster
    self._tf_br = tf2_ros.TransformBoradcaster()
    self._timer = self.create_timer(0.5, self.update_pose)

    # create control subscriber
    self._subscriber = self.create_subscription(
      QuarternionStamped,
      'orientation_in',
      self.handle_control,
      10
    )

  def update_pose(self):
    '''
    Get camera orientation and broadcast transform
    '''
    pos = self._ptz.GetStatus().Position.PanTilt

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'robot'
    t.child_frame_id = self.name
    q = tf_conversions.transformations.quaternion_from_euler(0, pos.y, pos.x) # roll, pitch, yaw
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    self._tf_br.sendTransform(t)
  
  def handle_control(self, msg:QuarternionStamped):
    # TODO
    raise NotImplementedError()

  def do_move(self):
    if self.active:
      self._ptz.Stop()

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('address', help='IP address of camera')
  parser.add_argument('-p', '--port', type=int, help='Port of camera', default=80)
  parser.add_argument('-u', '--user', help='Username for authentication', default='')
  parser.add_argument('-k', '--password', help='Password for authentication', default='')
  parser.add_argument('-n', '--name', help='Camera name for identification', default='onvif_camera')

  args = parser.parse_args()

  rclpy.init()
  try:
    camera_node = CameraNode(name=args.name, addr=args.address, port=args.port, user=args.user, pwd=args.password)
    try:
      rclpy.spin(camera_node)
    finally:
      camera_node.destroy_node()
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  main()