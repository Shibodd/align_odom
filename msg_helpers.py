import rclpy.time
import std_msgs.msg
import geometry_msgs.msg
from numpy.typing import ArrayLike
import numpy as np

def vector_from_msg(msg):
  if hasattr(msg, 'w'):
    return np.array((msg.x, msg.y, msg.z, msg.w))
  return np.array((msg.x, msg.y, msg.z))

def point(v):
  return geometry_msgs.msg.Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

def vector3(v):
  return geometry_msgs.msg.Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))

def transform_stamped(t: rclpy.time.Time, frame_id: str, child_frame_id: str, translation: ArrayLike, rotation: ArrayLike):
  return geometry_msgs.msg.TransformStamped(
    header=std_msgs.msg.Header(
      frame_id=frame_id,
      stamp=t.to_msg()
    ),
    child_frame_id=child_frame_id,
    transform=geometry_msgs.msg.Transform(
      translation=vector3(translation),
      rotation=geometry_msgs.msg.Quaternion(x=float(rotation[0]), y=float(rotation[1]), z=float(rotation[2]), w=float(rotation[3]))
    )
  )