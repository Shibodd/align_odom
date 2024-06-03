import numpy as np

import rclpy.node
import rclpy.publisher
import rclpy.subscription
import rclpy.qos
import nav_msgs.msg

import param_parser
import overlap
import point_set_registration
import msg_helpers
import odom_samples

class Odometry:
  def __init__(self, node: rclpy.node.Node, param_prefix, input_topic, main_odom: 'Odometry'):
    """
    main_odom: if this is None, then this odometry is the main odometry
    """
    pp = param_parser.ParameterParser(node, param_prefix)

    self.logger = node.get_logger()
    self.use_position = pp.expected('use_position', param_parser.ParameterType.PARAMETER_BOOL)
    self.use_orientation = pp.expected('use_orientation', param_parser.ParameterType.PARAMETER_BOOL)
    self.min_samples = pp.expected('min_samples', param_parser.ParameterType.PARAMETER_INTEGER)
    self.buffer = odom_samples.OdometrySamples(True, self.use_position, self.use_orientation, max(self.min_samples, 64))
    self.subscription = node.create_subscription(nav_msgs.msg.Odometry, input_topic, self.odometry_cb, rclpy.qos.qos_profile_sensor_data)
    self.main_odom = main_odom

    if self.main_odom is not None:
      self.has_transform = False
      self.registrator = getattr(point_set_registration, pp.expected('registrator', param_parser.ParameterType.PARAMETER_STRING))
      self.publisher = node.create_publisher(
        nav_msgs.msg.Odometry,
        pp.expected('output_topic', param_parser.ParameterType.PARAMETER_STRING),
        rclpy.qos.qos_profile_sensor_data
      )

  def __echo_aligned_odom(self, msg: nav_msgs.msg.Odometry):
    position = msg_helpers.vector_from_msg(msg.pose.pose.position)[:2]
    position = np.dot(self.transform_matrix, np.pad(position, (0,1), constant_values=1))[:2]

    msg.pose.pose.position = msg_helpers.point(np.pad(position, (0,1), constant_values=0))
    self.publisher.publish(msg)

  def __try_compute_transform(self):
    assert(self.main_odom is not None)

    if self.has_transform:
      return
    
    # Check for global min samples before computing overlap
    if self.buffer.size() < self.min_samples or self.main_odom.buffer.size() < self.main_odom.min_samples:
      return
    
    # Retrieve OVerLap between this odom and the main odom
    ovl_main_range, ovl_self_range = overlap.get_overlapping_ranges(
      self.main_odom.buffer.as_view().timestamps(),
      self.buffer.as_view().timestamps()
    )

    # Check for overlap min samples
    if len(ovl_main_range) < self.main_odom.min_samples or len(ovl_self_range) < self.min_samples:
      return
    
    # Retrieve the overlap samples
    ovl_main_samples = self.main_odom.buffer.retrieve()[:, ovl_main_range]
    ovl_self_samples = self.buffer.retrieve()[:, ovl_self_range]

    # Get main odom samples at this odom's timestamps by interpolation
    main_view = odom_samples.ArrayOdomSamplesView(False, self.use_position, self.use_orientation, 
      np.vstack(tuple(
        np.interp(ovl_self_samples[0,:], ovl_main_samples[0,:], ovl_main_samples[i,:])
        for i in range(1, self.buffer.as_view().dim())
      ))
    )
    self_view = odom_samples.ArrayOdomSamplesView(False, self.use_position, self.use_orientation,
      ovl_self_samples[1:,:]
    )

    # Compute the transform
    self.has_transform = True
    self.transform_matrix = self.registrator(self_view, main_view)
    self.logger.info(f'Transform obtained for odometry {self.subscription.topic}:\n{self.transform_matrix}')
    
  def odometry_cb(self, msg: nav_msgs.msg.Odometry):
    if self.buffer.size() == 0:
      self.logger.info(f'Odometry "{self.subscription.topic}" is alive!')

    self.buffer.add_sample(
      rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 10**9,
      msg_helpers.vector_from_msg(msg.pose.pose.position) if self.use_position else None,
      msg_helpers.vector_from_msg(msg.pose.pose.orientation) if self.use_orientation else None
    )

    # Proceed only if this is an aux odom
    if self.main_odom is None:
      return
    
    # Try to compute the transform, if it hadn't already been computed
    self.__try_compute_transform()

    # If we have a transformation, then transform this odometry
    if self.has_transform:
      self.__echo_aligned_odom(msg)
    