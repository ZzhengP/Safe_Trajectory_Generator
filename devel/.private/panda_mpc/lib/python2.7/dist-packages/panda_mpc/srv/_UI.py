# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from panda_mpc/UIRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class UIRequest(genpy.Message):
  _md5sum = "c014034d544e11b0948e3346a66ba7c7"
  _type = "panda_mpc/UIRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool play_traj
bool jog_robot
bool publish_traj 
bool build_traj
bool positioning_
geometry_msgs/Twist p_gains_
geometry_msgs/Twist d_gains_
bool move_signal_
bool tune_gain
float64 amplitude
int64 axis
bool exit_
float64 distance_to_contact_
bool fake_distance_

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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
  __slots__ = ['play_traj','jog_robot','publish_traj','build_traj','positioning_','p_gains_','d_gains_','move_signal_','tune_gain','amplitude','axis','exit_','distance_to_contact_','fake_distance_']
  _slot_types = ['bool','bool','bool','bool','bool','geometry_msgs/Twist','geometry_msgs/Twist','bool','bool','float64','int64','bool','float64','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       play_traj,jog_robot,publish_traj,build_traj,positioning_,p_gains_,d_gains_,move_signal_,tune_gain,amplitude,axis,exit_,distance_to_contact_,fake_distance_

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(UIRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.play_traj is None:
        self.play_traj = False
      if self.jog_robot is None:
        self.jog_robot = False
      if self.publish_traj is None:
        self.publish_traj = False
      if self.build_traj is None:
        self.build_traj = False
      if self.positioning_ is None:
        self.positioning_ = False
      if self.p_gains_ is None:
        self.p_gains_ = geometry_msgs.msg.Twist()
      if self.d_gains_ is None:
        self.d_gains_ = geometry_msgs.msg.Twist()
      if self.move_signal_ is None:
        self.move_signal_ = False
      if self.tune_gain is None:
        self.tune_gain = False
      if self.amplitude is None:
        self.amplitude = 0.
      if self.axis is None:
        self.axis = 0
      if self.exit_ is None:
        self.exit_ = False
      if self.distance_to_contact_ is None:
        self.distance_to_contact_ = 0.
      if self.fake_distance_ is None:
        self.fake_distance_ = False
    else:
      self.play_traj = False
      self.jog_robot = False
      self.publish_traj = False
      self.build_traj = False
      self.positioning_ = False
      self.p_gains_ = geometry_msgs.msg.Twist()
      self.d_gains_ = geometry_msgs.msg.Twist()
      self.move_signal_ = False
      self.tune_gain = False
      self.amplitude = 0.
      self.axis = 0
      self.exit_ = False
      self.distance_to_contact_ = 0.
      self.fake_distance_ = False

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
      buff.write(_get_struct_5B12d2BdqBdB().pack(_x.play_traj, _x.jog_robot, _x.publish_traj, _x.build_traj, _x.positioning_, _x.p_gains_.linear.x, _x.p_gains_.linear.y, _x.p_gains_.linear.z, _x.p_gains_.angular.x, _x.p_gains_.angular.y, _x.p_gains_.angular.z, _x.d_gains_.linear.x, _x.d_gains_.linear.y, _x.d_gains_.linear.z, _x.d_gains_.angular.x, _x.d_gains_.angular.y, _x.d_gains_.angular.z, _x.move_signal_, _x.tune_gain, _x.amplitude, _x.axis, _x.exit_, _x.distance_to_contact_, _x.fake_distance_))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.p_gains_ is None:
        self.p_gains_ = geometry_msgs.msg.Twist()
      if self.d_gains_ is None:
        self.d_gains_ = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 129
      (_x.play_traj, _x.jog_robot, _x.publish_traj, _x.build_traj, _x.positioning_, _x.p_gains_.linear.x, _x.p_gains_.linear.y, _x.p_gains_.linear.z, _x.p_gains_.angular.x, _x.p_gains_.angular.y, _x.p_gains_.angular.z, _x.d_gains_.linear.x, _x.d_gains_.linear.y, _x.d_gains_.linear.z, _x.d_gains_.angular.x, _x.d_gains_.angular.y, _x.d_gains_.angular.z, _x.move_signal_, _x.tune_gain, _x.amplitude, _x.axis, _x.exit_, _x.distance_to_contact_, _x.fake_distance_,) = _get_struct_5B12d2BdqBdB().unpack(str[start:end])
      self.play_traj = bool(self.play_traj)
      self.jog_robot = bool(self.jog_robot)
      self.publish_traj = bool(self.publish_traj)
      self.build_traj = bool(self.build_traj)
      self.positioning_ = bool(self.positioning_)
      self.move_signal_ = bool(self.move_signal_)
      self.tune_gain = bool(self.tune_gain)
      self.exit_ = bool(self.exit_)
      self.fake_distance_ = bool(self.fake_distance_)
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
      buff.write(_get_struct_5B12d2BdqBdB().pack(_x.play_traj, _x.jog_robot, _x.publish_traj, _x.build_traj, _x.positioning_, _x.p_gains_.linear.x, _x.p_gains_.linear.y, _x.p_gains_.linear.z, _x.p_gains_.angular.x, _x.p_gains_.angular.y, _x.p_gains_.angular.z, _x.d_gains_.linear.x, _x.d_gains_.linear.y, _x.d_gains_.linear.z, _x.d_gains_.angular.x, _x.d_gains_.angular.y, _x.d_gains_.angular.z, _x.move_signal_, _x.tune_gain, _x.amplitude, _x.axis, _x.exit_, _x.distance_to_contact_, _x.fake_distance_))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.p_gains_ is None:
        self.p_gains_ = geometry_msgs.msg.Twist()
      if self.d_gains_ is None:
        self.d_gains_ = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 129
      (_x.play_traj, _x.jog_robot, _x.publish_traj, _x.build_traj, _x.positioning_, _x.p_gains_.linear.x, _x.p_gains_.linear.y, _x.p_gains_.linear.z, _x.p_gains_.angular.x, _x.p_gains_.angular.y, _x.p_gains_.angular.z, _x.d_gains_.linear.x, _x.d_gains_.linear.y, _x.d_gains_.linear.z, _x.d_gains_.angular.x, _x.d_gains_.angular.y, _x.d_gains_.angular.z, _x.move_signal_, _x.tune_gain, _x.amplitude, _x.axis, _x.exit_, _x.distance_to_contact_, _x.fake_distance_,) = _get_struct_5B12d2BdqBdB().unpack(str[start:end])
      self.play_traj = bool(self.play_traj)
      self.jog_robot = bool(self.jog_robot)
      self.publish_traj = bool(self.publish_traj)
      self.build_traj = bool(self.build_traj)
      self.positioning_ = bool(self.positioning_)
      self.move_signal_ = bool(self.move_signal_)
      self.tune_gain = bool(self.tune_gain)
      self.exit_ = bool(self.exit_)
      self.fake_distance_ = bool(self.fake_distance_)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5B12d2BdqBdB = None
def _get_struct_5B12d2BdqBdB():
    global _struct_5B12d2BdqBdB
    if _struct_5B12d2BdqBdB is None:
        _struct_5B12d2BdqBdB = struct.Struct("<5B12d2BdqBdB")
    return _struct_5B12d2BdqBdB
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from panda_mpc/UIResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class UIResponse(genpy.Message):
  _md5sum = "eb13ac1f1354ccecb7941ee8fa2192e8"
  _type = "panda_mpc/UIResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool result

"""
  __slots__ = ['result']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(UIResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = False
    else:
      self.result = False

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
      _x = self.result
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
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
      _x = self.result
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class UI(object):
  _type          = 'panda_mpc/UI'
  _md5sum = '2e1bdac3ef57deb56fae70ed4385f616'
  _request_class  = UIRequest
  _response_class = UIResponse