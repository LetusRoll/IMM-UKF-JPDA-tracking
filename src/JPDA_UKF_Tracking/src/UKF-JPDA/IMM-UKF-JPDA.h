
/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-15 19:49:36
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-26 21:14:58
 */
#ifndef IMM_UKF_JPDA_H
#define IMM_UKF_JPDA_H
#include <ros/ros.h>
// #include <chrono>
// #include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include "JPDA_UKF_Tracking/object_array.h"
#include "JPDA_UKF_Tracking/object.h"
#include "UKF/UKF.h"
#include <vector>

using namespace std;

class IMM_UKF_JPDA
{
private:
    std_msgs::Header global_header;
    tf::TransformListener tf_listener;
    tf::StampedTransform local2world;
    tf::StampedTransform world2local;
    vector<UKF> targets;
    bool init = false;
    double timestamp_;
    double P_determinant_threshold;
    double dyaw_determinant_threshold;
    double S_determinant_threshold;
    double gating_threshold;

public:
    IMM_UKF_JPDA();
    ~IMM_UKF_JPDA();
    void TransformLocal2World(const JPDA_UKF_Tracking::object_array &objects, JPDA_UKF_Tracking::object_array &transformed_objects);
    geometry_msgs::Pose Gettransformedpose(const geometry_msgs::Pose &in_pose);
    //将观测量作为初始化输入
    void Initialize(const JPDA_UKF_Tracking::object_array &objects);
    void Output();
    void DataAssociation(const JPDA_UKF_Tracking::object_array &objects, UKF target,
                         Eigen::MatrixXd matching_mat, JPDA_UKF_Tracking::object_array matched_object,
                         Eigen::MatrixXd max_Z, Eigen::MatrixXd max_S);
    void findMaxSMaxZ(UKF target, Eigen::MatrixXd max_Z, Eigen::MatrixXd max_S);

    void MeasurementValidation(const JPDA_UKF_Tracking::object_array &objects, Eigen::MatrixXd matching_mat,
                               UKF target, Eigen::MatrixXd max_Z, Eigen::MatrixXd max_S,
                               JPDA_UKF_Tracking::object_array matched_object);

    void SecondInit(UKF target, JPDA_UKF_Tracking::object_array matched_object, double dt); //二次初始化

    void Process(const JPDA_UKF_Tracking::object_array &objects);
};

#endif