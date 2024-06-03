import rcl_interfaces.msg
import rclpy.parameter
from rclpy.parameter import ParameterType
import rclpy.node
import itertools
import typing


class MissingParameterException(Exception):
  pass

class ParameterParser:
  def __init__(self, node: rclpy.node.Node, prefix: str = ''):
    self.node = node
    self.prefix = prefix

  def __fmt_param(self, name):
    return f'{self.prefix}.{name}' if self.prefix else name

  def expected(self, name, type: ParameterType):
    ans = self.optional(name, None, type=type)
    if ans is None:
      self.node.get_logger().fatal(f'Missing expected parameter "{self.__fmt_param(name)}".')
      raise MissingParameterException()
    return ans

  def optional(self, name, default, type=None):
    assert default is not None or type is not None

    parameter = self.__fmt_param(name)
    if self.node.has_parameter(parameter):
      return self.node.get_parameter(parameter).value

    return self.node.declare_parameter(
      parameter,
      default,
      descriptor=rcl_interfaces.msg.ParameterDescriptor(type=type) if type is not None else None
    ).value
  
  def iter_named_list(self, list_member_fmt: str, field_name: str, field_type: ParameterType):
    # fmt0, fmt1, fmt2...
    prefixes = (list_member_fmt % i for i in itertools.count()) 

    # (fmt0, fmt0.field.value?), (fmt1, fmt1.field.value?), ...
    prefix_expected_pairs = ((self.__fmt_param(prefix), self.optional(f"{prefix}.{field_name}", None, field_type)) for prefix in prefixes)

    # (fmt0, fmt0.field.value), (fmt1, fmt1.field.value), ..., (fmtN, fmtN.field.value)
    return itertools.takewhile(lambda x: x[1] is not None, prefix_expected_pairs)