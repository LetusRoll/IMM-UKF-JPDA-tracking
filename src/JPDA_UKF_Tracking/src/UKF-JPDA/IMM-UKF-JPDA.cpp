/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-15 21:16:54
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-29 17:29:44
 */

#include "IMM-UKF-JPDA.h"

IMM_UKF_JPDA::IMM_UKF_JPDA()
{
    init = false;
}

void IMM_UKF_JPDA::Output()
{
}

void IMM_UKF_JPDA::TransformLocal2World(const JPDA_UKF_Tracking::object_array &objects, JPDA_UKF_Tracking::object_array &transformed_objects)
{
    transformed_objects.hearder = global_header;

    for (auto const object : objects.objects)
    {
        JPDA_UKF_Tracking::object transformed_object;
        transformed_object = object;
        transformed_object.header = global_header;
        transformed_object.pose = Gettransformedpose(object.pose);
        transformed_objects.objects.push_back(transformed_object);
    }
}

geometry_msgs::Pose IMM_UKF_JPDA::Gettransformedpose(const geometry_msgs::Pose &in_pose)
{
    tf::Transform tf;
    geometry_msgs::Pose out_pose;
    tf.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    tf.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    tf::poseTFToMsg(local2world * tf, out_pose);
    return out_pose;
}

//初始化
void IMM_UKF_JPDA::Initialize(const JPDA_UKF_Tracking::object_array &objects)
{
    JPDA_UKF_Tracking::object_array transformed_objects;
    TransformLocal2World(objects, transformed_objects);
    for (int i = 0; i < transformed_objects.objects.size(); ++i)
    {
        UKF ukf;
        Eigen::VectorXd init_meas(2);
        init_meas << transformed_objects.objects[i].pose.position.x, transformed_objects.objects[i].pose.position.y;
        ukf.Initialize(init_meas);
        targets.push_back(ukf);
    }
}

void IMM_UKF_JPDA::findMaxSMaxZ(UKF target, Eigen::MatrixXd max_Z, Eigen::MatrixXd max_S)
{
    double max_det = target.S_cv.determinant();
    max_det = max_det < target.S_ctrv.determinant() ? target.S_ctrv.determinant() : max_det;
    max_det = max_det < target.S_rm.determinant() ? target.S_rm.determinant() : max_det;
    if (max_det == target.S_cv.determinant())
    {
        max_Z = target.Z_predict_cv;
        max_S = target.S_cv;
    }
    if (max_det == target.S_ctrv.determinant())
    {
        max_Z = target.Z_predict_ctrv;
        max_S = target.S_ctrv;
    }
    if (max_det == target.S_rm.determinant())
    {
        max_Z = target.Z_predict_rm;
        max_S = target.S_rm;
    }
}

void IMM_UKF_JPDA::MeasurementValidation(const JPDA_UKF_Tracking::object_array &objects,
                                         Eigen::MatrixXd matching_mat, UKF target, Eigen::MatrixXd max_Z,
                                         Eigen::MatrixXd max_S, JPDA_UKF_Tracking::object_array matched_object)
{

    double smallest_nis = numeric_limits<double>::max();
    int matched_index = 0; //匹配的最近的物体的索引
    JPDA_UKF_Tracking::object tmp_object;
    for (int i = 0; i < objects.objects.size(); ++i)
    {
        double x = objects.objects[i].pose.position.x;
        double y = objects.objects[i].pose.position.y;
        Eigen::MatrixXd meas = Eigen::MatrixXd(2, 1);
        meas << x, y;
        Eigen::VectorXd diff = meas - max_Z;
        double nis = diff.transpose() * max_S.inverse() * diff;
        if (nis < gating_threshold)
        {
            if (nis < smallest_nis)
            {
                smallest_nis = nis;
                matched_index = i;
                tmp_object = objects.objects[i];
            }
        }
    }
    matched_object.objects.push_back(tmp_object);
    matching_mat[matched_index] = 1;
}

void IMM_UKF_JPDA::UpdateTargetState(UKF target, JPDA_UKF_Tracking::object_array matched_object)
{
    target.life_time++;
    //更新Tracking_num
    if (matched_object.objects.size() > 0)
    {
        if (target.tracking_num < TrackingState::Stable)
        {
            target.tracking_num++;
        }
        else if (target.tracking_num == TrackingState::Stable)
        {
            target.tracking_num = TrackingState::Stable;
        }
        else if (target.tracking_num > TrackingState::Stable && target.tracking_num <= TrackingState::Lost)
        {
            target.tracking_num = TrackingState::Stable;
        }
    }
    else
    {
        if (target.tracking_num < TrackingState::Stable)
        {
            target.tracking_num = TrackingState::Die;
        }
        else if (target.tracking_num >= TrackingState::Stable && target.tracking_num < TrackingState::Lost)
        {
            target.tracking_num++;
        }
        else if (target.tracking_num == TrackingState::Lost)
        {
            target.tracking_num = TrackingState::Die;
        }
    }

    if (target.tracking_num == TrackingState::Stable || target.tracking_num == TrackingState::Occlusion)
    {
        target.is_stable = true;
    }
}

void IMM_UKF_JPDA::DataAssociation(const JPDA_UKF_Tracking::object_array &objects, UKF target,
                                   Eigen::MatrixXd matching_mat, JPDA_UKF_Tracking::object_array matched_object,
                                   Eigen::MatrixXd max_Z, Eigen::MatrixXd max_S)
{

    findMaxSMaxZ(target, max_Z, max_S);

    if ((isnan(max_S.determinant())) || (max_S.determinant() > S_determinant_threshold))
    {
        target.tracking_num = TrackingState::Die;
        return;
    }

    MeasurementValidation(objects, matching_mat, target, max_Z, max_S, matched_object);
}

void IMM_UKF_JPDA::SecondInit(UKF target, JPDA_UKF_Tracking::object_array matched_object, double dt)
{
    if (matched_object.objects.size() == 0) //未匹配则状态设为Die
    {
        target.tracking_num = TrackingState::Die;
    }
    else
    {
        double target_x = matched_object.objects[0].pose.position.x;
        double target_y = matched_object.objects[0].pose.position.y;
        double target_diff_x = target_x - target.X_merge[0];
        double target_diff_y = target_y - target.X_merge[1];
        target.X_merge[0] = target.X_cv[0] = target.X_ctrv[0] = target.X_rm[0] = target_x;
        target.X_merge[1] = target.X_cv[1] = target.X_ctrv[1] = target.X_rm[1] = target_y;
        target.X_merge[2] = target.X_cv[2] = target.X_ctrv[2] = target.X_rm[2] = sqrt(pow(target_diff_x, 2) + pow(target_diff_y, 2)) / dt;
        double yaw = atan2(target_diff_y, target_diff_x);
        if (yaw < -PI)
        {
            yaw += 2 * PI;
        }
        if (yaw > PI)
        {
            yaw -= 2 * PI;
        }
        target.X_merge[3] = target.X_cv[3] = target.X_ctrv[3] = target.X_rm[3] = yaw;
        target.X_merge[4] = target.X_cv[4] = target.X_ctrv[4] = target.X_rm[4] = yaw / dt;
        target.tracking_num++;
    }
}

void IMM_UKF_JPDA::Process(const JPDA_UKF_Tracking::object_array &objects)
{
    double timestamp = objects.hearder.stamp.toSec();

    double dt = timestamp - timestamp_;
    timestamp_ = timestamp;
    if (!init)
    {
        Initialize(objects);

        Output();
        init = true;
    }
    Eigen::MatrixXd matching_mat;
    matching_mat.resize(objects.objects.size(), 1);
    matching_mat.fill(0);

    // JPDA_UKF_Tracking::object_array transformed_objects;
    // TransformLocal2World(objects,transformed_objects);
    for (int i = 0; i < targets.size(); ++i)
    {
        if (targets[i].tracking_num == TrackingState::Die)
        {

            continue;
        }
        if (targets[i].P_merge.determinant() > P_determinant_threshold || targets[i].P_merge(4, 4) > dyaw_determinant_threshold)
        {
            targets[i].tracking_num = TrackingState::Die;
        }
        targets[i].Predict(dt);
        Eigen::MatrixXd max_Z;
        Eigen::MatrixXd max_S;

        JPDA_UKF_Tracking::object_array matched_object; //将匹配的物体提出来，用于后面的SecondInit,
                                                        //   这里必须用array，后面SecondInit需要判断是否为空
        DataAssociation(objects, targets[i], matching_mat, matched_object, max_Z, max_S);

        if (targets[i].tracking_num == TrackingState::Init)
        {
            SecondInit(targets[i], matched_object, dt);
            continue;
        }
        UpdateTargetState(targets[i], matched_object);
        if (targets[i].tracking_num == TrackingState::Die)
        {
            continue;
        }
        targets[i].Update(matched_object, gating_threshold, detection_probability, gate_probability);
    }
}
