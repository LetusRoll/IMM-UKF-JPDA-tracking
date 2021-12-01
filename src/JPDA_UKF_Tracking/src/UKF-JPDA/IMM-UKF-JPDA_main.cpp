/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-30 17:12:19
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-30 23:01:15
 */
#include "../UKF/UKF.h"
#include "IMM-UKF-JPDA.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    IMM_UKF_JPDA imm_ukf_jpda(nh, private_nh);
    return 0;
}