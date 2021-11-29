/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-29 10:41:02
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-29 11:34:26
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "rslidar"));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odometry/gps", 100, Callback);
    ros::spin();
    return 0;
}
