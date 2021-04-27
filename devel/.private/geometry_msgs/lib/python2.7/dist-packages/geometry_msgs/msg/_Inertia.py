# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from geometry_msgs/Inertia.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Inertia(genpy.Message):
  _md5sum = "1d26e4bb6c83ff141c5cf0d883c2b0fe"
  _type = "geometry_msgs/Inertia"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['m','com','ixx','ixy','ixz','iyy','iyz','izz']
  _slot_types = ['float64','geometry_msgs/Vector3','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       m,com,ixx,ixy,ixz,iyy,iyz,izz

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Inertia, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.m is None:
        self.m = 0.
      if self.com is None:
        self.com = geometry_msgs.msg.Vector3()
      if self.ixx is None:
        self.ixx = 0.
      if self.ixy is None:
        self.ixy = 0.
      if self.ixz is None:
        self.ixz = 0.
      if self.iyy is None:
        self.iyy = 0.
      if self.iyz is None:
        self.iyz = 0.
      if self.izz is None:
        self.izz = 0.
    else:
      self.m = 0.
      self.com = geometry_msgs.msg.Vector3()
      self.ixx = 0.
      self.ixy = 0.
      self.ixz = 0.
      self.iyy = 0.
      self.iyz = 0.
      self.izz = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_10d().pack(_x.m, _x.com.x, _x.com.y, _x.com.z, _x.ixx, _x.ixy, _x.ixz, _x.iyy, _x.iyz, _x.izz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.com is None:
        self.com = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 80
      (_x.m, _x.com.x, _x.com.y, _x.com.z, _x.ixx, _x.ixy, _x.ixz, _x.iyy, _x.iyz, _x.izz,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_10d().pack(_x.m, _x.com.x, _x.com.y, _x.com.z, _x.ixx, _x.ixy, _x.ixz, _x.iyy, _x.iyz, _x.izz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.com is None:
        self.com = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 80
      (_x.m, _x.com.x, _x.com.y, _x.com.z, _x.ixx, _x.ixy, _x.ixz, _x.iyy, _x.iyz, _x.izz,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10d = None
def _get_struct_10d():
    global _struct_10d
    if _struct_10d is None:
        _struct_10d = struct.Struct("<10d")
    return _struct_10d
