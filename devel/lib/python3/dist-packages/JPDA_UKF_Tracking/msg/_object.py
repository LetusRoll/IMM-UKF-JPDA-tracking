# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from JPDA_UKF_Tracking/object.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

class object(genpy.Message):
  _md5sum = "75b5aef8e67be81912fbb3bf0345714b"
  _type = "JPDA_UKF_Tracking/object"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """###物体类型
std_msgs/Header             header   ##坐标系和时间戳
uint32                      id       ##物体的序号
geometry_msgs/Pose          pose     ##位姿
geometry_msgs/Vector3       dimensions##长宽高
geometry_msgs/Vector3       variance  ##
sensor_msgs/PointCloud2     pointcloud
geometry_msgs/PolygonStamped     convex_hull

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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
float64 z
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['header','id','pose','dimensions','variance','pointcloud','convex_hull']
  _slot_types = ['std_msgs/Header','uint32','geometry_msgs/Pose','geometry_msgs/Vector3','geometry_msgs/Vector3','sensor_msgs/PointCloud2','geometry_msgs/PolygonStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,pose,dimensions,variance,pointcloud,convex_hull

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(object, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = 0
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      if self.variance is None:
        self.variance = geometry_msgs.msg.Vector3()
      if self.pointcloud is None:
        self.pointcloud = sensor_msgs.msg.PointCloud2()
      if self.convex_hull is None:
        self.convex_hull = geometry_msgs.msg.PolygonStamped()
    else:
      self.header = std_msgs.msg.Header()
      self.id = 0
      self.pose = geometry_msgs.msg.Pose()
      self.dimensions = geometry_msgs.msg.Vector3()
      self.variance = geometry_msgs.msg.Vector3()
      self.pointcloud = sensor_msgs.msg.PointCloud2()
      self.convex_hull = geometry_msgs.msg.PolygonStamped()

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_I13d3I().pack(_x.id, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.variance.x, _x.variance.y, _x.variance.z, _x.pointcloud.header.seq, _x.pointcloud.header.stamp.secs, _x.pointcloud.header.stamp.nsecs))
      _x = self.pointcloud.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.pointcloud.height, _x.pointcloud.width))
      length = len(self.pointcloud.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pointcloud.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.pointcloud.is_bigendian, _x.pointcloud.point_step, _x.pointcloud.row_step))
      _x = self.pointcloud.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_B3I().pack(_x.pointcloud.is_dense, _x.convex_hull.header.seq, _x.convex_hull.header.stamp.secs, _x.convex_hull.header.stamp.nsecs))
      _x = self.convex_hull.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.convex_hull.polygon.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.convex_hull.polygon.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      if self.variance is None:
        self.variance = geometry_msgs.msg.Vector3()
      if self.pointcloud is None:
        self.pointcloud = sensor_msgs.msg.PointCloud2()
      if self.convex_hull is None:
        self.convex_hull = geometry_msgs.msg.PolygonStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 120
      (_x.id, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.variance.x, _x.variance.y, _x.variance.z, _x.pointcloud.header.seq, _x.pointcloud.header.stamp.secs, _x.pointcloud.header.stamp.nsecs,) = _get_struct_I13d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pointcloud.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.pointcloud.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pointcloud.height, _x.pointcloud.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pointcloud.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.pointcloud.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pointcloud.is_bigendian, _x.pointcloud.point_step, _x.pointcloud.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.pointcloud.is_bigendian = bool(self.pointcloud.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.pointcloud.data = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.pointcloud.is_dense, _x.convex_hull.header.seq, _x.convex_hull.header.stamp.secs, _x.convex_hull.header.stamp.nsecs,) = _get_struct_B3I().unpack(str[start:end])
      self.pointcloud.is_dense = bool(self.pointcloud.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.convex_hull.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.convex_hull.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.convex_hull.polygon.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.convex_hull.polygon.points.append(val1)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_I13d3I().pack(_x.id, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.variance.x, _x.variance.y, _x.variance.z, _x.pointcloud.header.seq, _x.pointcloud.header.stamp.secs, _x.pointcloud.header.stamp.nsecs))
      _x = self.pointcloud.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.pointcloud.height, _x.pointcloud.width))
      length = len(self.pointcloud.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pointcloud.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.pointcloud.is_bigendian, _x.pointcloud.point_step, _x.pointcloud.row_step))
      _x = self.pointcloud.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_B3I().pack(_x.pointcloud.is_dense, _x.convex_hull.header.seq, _x.convex_hull.header.stamp.secs, _x.convex_hull.header.stamp.nsecs))
      _x = self.convex_hull.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.convex_hull.polygon.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.convex_hull.polygon.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      if self.variance is None:
        self.variance = geometry_msgs.msg.Vector3()
      if self.pointcloud is None:
        self.pointcloud = sensor_msgs.msg.PointCloud2()
      if self.convex_hull is None:
        self.convex_hull = geometry_msgs.msg.PolygonStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 120
      (_x.id, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.variance.x, _x.variance.y, _x.variance.z, _x.pointcloud.header.seq, _x.pointcloud.header.stamp.secs, _x.pointcloud.header.stamp.nsecs,) = _get_struct_I13d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pointcloud.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.pointcloud.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pointcloud.height, _x.pointcloud.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pointcloud.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.pointcloud.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pointcloud.is_bigendian, _x.pointcloud.point_step, _x.pointcloud.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.pointcloud.is_bigendian = bool(self.pointcloud.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.pointcloud.data = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.pointcloud.is_dense, _x.convex_hull.header.seq, _x.convex_hull.header.stamp.secs, _x.convex_hull.header.stamp.nsecs,) = _get_struct_B3I().unpack(str[start:end])
      self.pointcloud.is_dense = bool(self.pointcloud.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.convex_hull.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.convex_hull.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.convex_hull.polygon.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.convex_hull.polygon.points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_B2I = None
def _get_struct_B2I():
    global _struct_B2I
    if _struct_B2I is None:
        _struct_B2I = struct.Struct("<B2I")
    return _struct_B2I
_struct_B3I = None
def _get_struct_B3I():
    global _struct_B3I
    if _struct_B3I is None:
        _struct_B3I = struct.Struct("<B3I")
    return _struct_B3I
_struct_I13d3I = None
def _get_struct_I13d3I():
    global _struct_I13d3I
    if _struct_I13d3I is None:
        _struct_I13d3I = struct.Struct("<I13d3I")
    return _struct_I13d3I
_struct_IBI = None
def _get_struct_IBI():
    global _struct_IBI
    if _struct_IBI is None:
        _struct_IBI = struct.Struct("<IBI")
    return _struct_IBI