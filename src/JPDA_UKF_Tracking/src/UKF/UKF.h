
/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-09 22:20:50
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-26 21:38:35
 */

#ifndef UKF_H
#define UKF_H

#include <Eigen/Dense>

const double PI = 3.14159;
enum TrackingState : int
{
    Die = 0,       // No longer tracking
    Init = 1,      // Start tracking
    Stable = 4,    // Stable tracking
    Occlusion = 5, // Lost 1 frame possibly by occlusion
    Lost = 10,     // About to lose target
};

enum Mode : int
{
    CV = 1,
    CTRV = 2,
    RM = 3,
};

class UKF
{
public:
    int tracking_num;
    bool is_static;
    bool is_stable;

    int state_num_;
    int meas_num_;
    float alpha;
    float beta;
    float k;
    float lambda;

    Eigen::MatrixXd mode_prob;
    Eigen::MatrixXd mode_trans_prob;
    Eigen::MatrixXd initial_mode_trans_prob;

    //过程噪声
    double std_a_cv_;
    double std_a_ctrv_;
    double std_a_rm_;

    double std_ctrv_yawdd_;
    double std_cv_yawdd_;
    double std_rm_yawdd_;
    //测量噪声
    double std_laspx_;
    double std_laspy_;

    Eigen::VectorXd w_s; //期望
    Eigen::VectorXd w_c; //方差

    Eigen::MatrixXd X_merge; //状态量   x y v yaw dyaw
    Eigen::MatrixXd X_cv;
    Eigen::MatrixXd X_ctrv;
    Eigen::MatrixXd X_rm;

    Eigen::MatrixXd P_merge; //状态量方差
    Eigen::MatrixXd P_cv;
    Eigen::MatrixXd P_ctrv;
    Eigen::MatrixXd P_rm;

    Eigen::MatrixXd Q_cv;
    Eigen::MatrixXd Q_ctrv;
    Eigen::MatrixXd Q_rm;

    Eigen::MatrixXd X_predict_sig_cv; //预测状态量
    Eigen::MatrixXd X_predict_sig_ctrv;
    Eigen::MatrixXd X_predict_sig_rm;

    Eigen::MatrixXd Z_predict_cv; //预测测量量
    Eigen::MatrixXd Z_predict_ctrv;
    Eigen::MatrixXd Z_predict_rm;

    Eigen::MatrixXd S_cv; //预测测量方差
    Eigen::MatrixXd S_ctrv;
    Eigen::MatrixXd S_rm;

    Eigen::MatrixXd R_cv;
    Eigen::MatrixXd R_ctrv;
    Eigen::MatrixXd R_rm;

    Eigen::MatrixXd C_xz_cv; //协方差
    Eigen::MatrixXd C_xz_ctrv;
    Eigen::MatrixXd C_xz_rm;

public:
    UKF();
    ~UKF();
    void Initialize(Eigen::VectorXd &init_meas);
    void Predict(const double dt);
    void InitialCovQ(const double dt);   //初始化Q
    void Interaction();                  //计算交互矩阵
    void PredictMotion(const double dt); //预测各个模型的状态量
    void MixProbability();
    void PredictMotion(const double dt, int mode);
    void CV(const Eigen::MatrixXd &X_sig, const double dt);
    void CTRV(const Eigen::MatrixXd &X_sig, const double dt);
    void RM(const Eigen::MatrixXd &X_sig, const double dt);
    void PredictMeasurement(int mode); //预测测量量
    void Update();
};

#endif
