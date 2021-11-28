// Generated by gencpp from file JPDA_UKF_Tracking/object_array.msg
// DO NOT EDIT!


#ifndef JPDA_UKF_TRACKING_MESSAGE_OBJECT_ARRAY_H
#define JPDA_UKF_TRACKING_MESSAGE_OBJECT_ARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <JPDA_UKF_Tracking/object.h>

namespace JPDA_UKF_Tracking
{
template <class ContainerAllocator>
struct object_array_
{
  typedef object_array_<ContainerAllocator> Type;

  object_array_()
    : hearder()
    , objects()  {
    }
  object_array_(const ContainerAllocator& _alloc)
    : hearder(_alloc)
    , objects(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _hearder_type;
  _hearder_type hearder;

   typedef std::vector< ::JPDA_UKF_Tracking::object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >::other >  _objects_type;
  _objects_type objects;





  typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> const> ConstPtr;

}; // struct object_array_

typedef ::JPDA_UKF_Tracking::object_array_<std::allocator<void> > object_array;

typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_array > object_arrayPtr;
typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_array const> object_arrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator1> & lhs, const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator2> & rhs)
{
  return lhs.hearder == rhs.hearder &&
    lhs.objects == rhs.objects;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator1> & lhs, const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace JPDA_UKF_Tracking

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e91d385083a6f18bcdb564c8ce15cc81";
  }

  static const char* value(const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe91d385083a6f18bULL;
  static const uint64_t static_value2 = 0xcdb564c8ce15cc81ULL;
};

template<class ContainerAllocator>
struct DataType< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JPDA_UKF_Tracking/object_array";
  }

  static const char* value(const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header     hearder\n"
"object[]     objects\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: JPDA_UKF_Tracking/object\n"
"###物体类型\n"
"std_msgs/Header             header   ##坐标系和时间戳\n"
"uint32                      id       ##物体的序号\n"
"geometry_msgs/Pose          pose     ##位姿\n"
"geometry_msgs/Vector3       dimensions##长宽高\n"
"geometry_msgs/Vector3       variance  ##\n"
"sensor_msgs/PointCloud2     pointcloud\n"
"geometry_msgs/PolygonStamped     convex_hull\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PolygonStamped\n"
"# This represents a Polygon with reference coordinate frame and timestamp\n"
"Header header\n"
"Polygon polygon\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.hearder);
      stream.next(m.objects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct object_array_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::JPDA_UKF_Tracking::object_array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::JPDA_UKF_Tracking::object_array_<ContainerAllocator>& v)
  {
    s << indent << "hearder: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.hearder);
    s << indent << "objects[]" << std::endl;
    for (size_t i = 0; i < v.objects.size(); ++i)
    {
      s << indent << "  objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >::stream(s, indent + "    ", v.objects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JPDA_UKF_TRACKING_MESSAGE_OBJECT_ARRAY_H
