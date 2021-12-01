;; Auto-generated. Do not edit!


(when (boundp 'JPDA_UKF_Tracking::object_array)
  (if (not (find-package "JPDA_UKF_TRACKING"))
    (make-package "JPDA_UKF_TRACKING"))
  (shadow 'object_array (find-package "JPDA_UKF_TRACKING")))
(unless (find-package "JPDA_UKF_TRACKING::OBJECT_ARRAY")
  (make-package "JPDA_UKF_TRACKING::OBJECT_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude object_array.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass JPDA_UKF_Tracking::object_array
  :super ros::object
  :slots (_hearder _objects ))

(defmethod JPDA_UKF_Tracking::object_array
  (:init
   (&key
    ((:hearder __hearder) (instance std_msgs::Header :init))
    ((:objects __objects) ())
    )
   (send-super :init)
   (setq _hearder __hearder)
   (setq _objects __objects)
   self)
  (:hearder
   (&rest __hearder)
   (if (keywordp (car __hearder))
       (send* _hearder __hearder)
     (progn
       (if __hearder (setq _hearder (car __hearder)))
       _hearder)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _hearder
    (send _hearder :serialization-length)
    ;; JPDA_UKF_Tracking/object[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _hearder
       (send _hearder :serialize s)
     ;; JPDA_UKF_Tracking/object[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _hearder
     (send _hearder :deserialize buf ptr-) (incf ptr- (send _hearder :serialization-length))
   ;; JPDA_UKF_Tracking/object[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance JPDA_UKF_Tracking::object :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get JPDA_UKF_Tracking::object_array :md5sum-) "81187b16050baf6314918c62baf7c77b")
(setf (get JPDA_UKF_Tracking::object_array :datatype-) "JPDA_UKF_Tracking/object_array")
(setf (get JPDA_UKF_Tracking::object_array :definition-)
      "std_msgs/Header     hearder
object[]     objects
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
MSG: JPDA_UKF_Tracking/object
###物体类型
std_msgs/Header             header   ##坐标系和时间戳
uint32                      id       ##物体的序号
geometry_msgs/Pose          pose     ##位姿
geometry_msgs/Vector3       dimensions##长宽高
geometry_msgs/Vector3       variance  ##
sensor_msgs/PointCloud2     pointcloud
geometry_msgs/PolygonStamped     convex_hull
geometry_msgs/Twist      velocity
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
# contents of the \"fields\" array.

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
float32 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :JPDA_UKF_Tracking/object_array "81187b16050baf6314918c62baf7c77b")


