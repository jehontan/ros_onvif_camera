import argparse

import rclpy
from rclpy.node import Node
from onvif import ONVIFCamera

import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

class CameraNode(Node):
  def __init__(self, name:str, addr:str, port:int, user:str, pwd:str):
    super().__init__(name, namespace=name)

    self.name = name

    # setup PTZ
    self._camera = ONVIFCamera(addr, port, user, pwd)
    self._media = self._camera.create_media_service()
    self._ptz = self._camera.create_ptz_service()

    media_profile = self._media.GetProfiles()[0]
    self._profile_token = media_profile.token
    
    # get PTZ move range and store in parameters
    request = self._ptz.create_type('GetConfigurationOptions')
    request.ConfigurationToken = media_profile.PTZConfiguration.token
    ptz_configuration_options = self._ptz.GetConfigurationOptions(request)

    XMAX = rclpy.parameter.Parameter(
      'pan_max',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].XRange.Max
    )

    XMIN = rclpy.parameter.Parameter(
      'pan_min',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].XRange.Min
    )

    YMAX = rclpy.parameter.Parameter(
      'tilt_max',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].YRange.Max
    )

    YMIN = rclpy.parameter.Parameter(
      'tilt_min',
      rclpy.Parameter.Type.DOUBLE,
      ptz_configuration_options.Spaces.AbsolutePanTiltPositionSpace[0].YRange.Min
    )

    self.set_parameters([XMAX, XMIN, YMAX, YMIN])

    # initialize self.pos
    self.update_pose()

    # setup transform broadcaster
    self._tf_br = tf2_ros.TransformBoradcaster()
    self._timer = self.create_timer(0.5, self.update_pose) # update every 0.5 sec

    # create control subscriber
    self._subscriber = self.create_subscription(
      PoseStamped,
      'cmd_abs_move',
      self.handle_cmd_abs_move,
      10
    )

  def update_pose(self):
    '''
    Get camera position and broadcast transform
    '''
    self.pos = {
      'pan': self._ptz.GetStatus().Position.PanTilt.x,
      'tilt': self._ptz.GetStatus().Position.PanTilt.y,
      'zoom': self._ptz.GetStatus().Position.Zoom.x
    }
    
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'robot'
    t.child_frame_id = self.name
    q = tf_conversions.transformations.quaternion_from_euler(0, self.pos.tilt, self.pos.pan) # roll, pitch, yaw
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    self._tf_br.sendTransform(t)
  

  def handle_cmd_abs_move(self, msg:PoseStamped):
    '''
    Handle absolute move commands.
    '''
    q = msg.pose.orientation
    zoom = msg.pose.position.z
    pos = tf_conversions.transformations.euler_from_quarternion(q.x, q.y, q.z, q.w) # roll, pitch, yaw
    self.absolute_move(pan=pos[2], tilt=pos[1], zoom=zoom)
    

  def absolute_move(self, pan=None, tilt=None, zoom=None):
    # setup move_request template
    request = self._ptz.create_type('AbsoluteMove')
    request.ProfileToken = self._profile_token
    request.Position.PanTilt.x = pan if pan is not None else self.pos.pan
    request.Position.PanTilt.y = tilt if tilt is not None else self.pos.tilt
    request.Position.Zoom.x = zoom if zoom is not None else self.pos.zoom
    self._ptz.Stop({'ProfileToken': self._profile_token})
    self._ptz.AbsoluteMove(request)

  def continuous_move(self, pan_speed, tilt_speed, zoom_speed):
    # TODO
    raise NotImplementedError

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