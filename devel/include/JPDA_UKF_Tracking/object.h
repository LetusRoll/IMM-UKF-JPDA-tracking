// Generated by gencpp from file JPDA_UKF_Tracking/object.msg
// DO NOT EDIT!


#ifndef JPDA_UKF_TRACKING_MESSAGE_OBJECT_H
#define JPDA_UKF_TRACKING_MESSAGE_OBJECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>

namespace JPDA_UKF_Tracking
{
template <class ContainerAllocator>
struct object_
{
  typedef object_<ContainerAllocator> Type;

  object_()
    : header()
    , id(0)
    , pose()
    , dimensions()
    , variance()
    , pointcloud()
    , convex_hull()  {
    }
  object_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , pose(_alloc)
    , dimensions(_alloc)
    , variance(_alloc)
    , pointcloud(_alloc)
    , convex_hull(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _id_type;
  _id_type id;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _dimensions_type;
  _dimensions_type dimensions;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _variance_type;
  _variance_type variance;

   typedef  ::sensor_msgs::PointCloud2_<ContainerAllocator>  _pointcloud_type;
  _pointcloud_type pointcloud;

   typedef  ::geometry_msgs::PolygonStamped_<ContainerAllocator>  _convex_hull_type;
  _convex_hull_type convex_hull;





  typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object_<ContainerAllocator> const> ConstPtr;

}; // struct object_

typedef ::JPDA_UKF_Tracking::object_<std::allocator<void> > object;

typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object > objectPtr;
typedef boost::shared_ptr< ::JPDA_UKF_Tracking::object const> objectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::JPDA_UKF_Tracking::object_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::JPDA_UKF_Tracking::object_<ContainerAllocator1> & lhs, const ::JPDA_UKF_Tracking::object_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.pose == rhs.pose &&
    lhs.dimensions == rhs.dimensions &&
    lhs.variance == rhs.variance &&
    lhs.pointcloud == rhs.pointcloud &&
    lhs.convex_hull == rhs.convex_hull;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::JPDA_UKF_Tracking::object_<ContainerAllocator1> & lhs, const ::JPDA_UKF_Tracking::object_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace JPDA_UKF_Tracking

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::JPDA_UKF_Tracking::object_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::JPDA_UKF_Tracking::object_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::JPDA_UKF_Tracking::object_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "75b5aef8e67be81912fbb3bf0345714b";
  }

  static const char* value(const ::JPDA_UKF_Tracking::object_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x75b5aef8e67be819ULL;
  static const uint64_t static_value2 = 0x12fbb3bf0345714bULL;
};

template<class ContainerAllocator>
struct DataType< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JPDA_UKF_Tracking/object";
  }

  static const char* value(const ::JPDA_UKF_Tracking::object_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "###物体类型\n"
"std_msgs/Header             header   ##坐标系和时间戳\n"
"uint32                      id       ##物体的序号\n"
"geometry_msgs/Pose          pose     ##位姿\n"
"geometry_msgs/Vector3       dimensions##长宽高\n"
"geometry_msgs/Vector3       variance  ##\n"
"sensor_msgs/PointCloud2     pointcloud\n"
"geometry_msgs/PolygonStamped     convex_hull\n"
"\n"
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

  static const char* value(const ::JPDA_UKF_Tracking::object_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.pose);
      stream.next(m.dimensions);
      stream.next(m.variance);
      stream.next(m.pointcloud);
      stream.next(m.convex_hull);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct object_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::JPDA_UKF_Tracking::object_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::JPDA_UKF_Tracking::object_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "dimensions: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.dimensions);
    s << indent << "variance: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.variance);
    s << indent << "pointcloud: ";
    s << std::endl;
    Printer< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::stream(s, indent + "  ", v.pointcloud);
    s << indent << "convex_hull: ";
    s << std::endl;
    Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.convex_hull);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JPDA_UKF_TRACKING_MESSAGE_OBJECT_H
