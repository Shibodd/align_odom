import rclpy.node
import param_parser
import rclpy.qos
import rclpy.subscription
import rclpy.time
import odometry

class QuitException(Exception):
  pass

class OdometryAlignerNode(rclpy.node.Node):
  def __init__(self):
    super().__init__('odometry_aligner_node')
    self.parse_odoms()

  def parse_odoms(self):
    pp = param_parser.ParameterParser(self)

    self.main_odom = odometry.Odometry(self, 'main_odom', pp.expected('main_odom.input_topic', param_parser.ParameterType.PARAMETER_STRING), None)
    self.aux_odoms = [
      odometry.Odometry(self, prefix, topic, self.main_odom)
      for prefix, topic in pp.iter_named_list('aux_odoms._%d', 'input_topic', param_parser.ParameterType.PARAMETER_STRING)
    ]

    if len(self.aux_odoms) == 0:
      self.get_logger().warning("No aux odoms configured. Exiting prematurely.")
      raise QuitException()
    
    topics = ', '.join(f'"{x.subscription.topic}"' for x in self.aux_odoms)
    self.get_logger().info(f'Running. Registered aux odoms: {topics}')

if __name__ == '__main__':
  rclpy.init()
  try:
    rclpy.spin(OdometryAlignerNode())
  except QuitException:
    pass
  rclpy.shutdown()