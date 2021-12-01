###物体类型
std_msgs/Header             header   ##坐标系和时间戳
uint32                      id       ##物体的序号
geometry_msgs/Pose          pose     ##位姿
geometry_msgs/Vector3       dimensions##长宽高
geometry_msgs/Vector3       variance  ##
sensor_msgs/PointCloud2     pointcloud
geometry_msgs/PolygonStamped     convex_hull
geometry_msgs/Twist      velocity