/*
 * @Description: 
 * @Autor: C-Xingyu
 * @Date: 2021-11-09 22:20:42
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2021-11-26 21:38:38
 */

#include "UKF.h"

UKF::UKF()
{
    state_num_ = 5; //x y v yaw dyaw
    meas_num_ = 2;
    mode_prob.resize(3, 1);
    initial_mode_trans_prob.resize(3, 3);
    mode_trans_prob.resize(3, 3);

    w_c.resize(2 * state_num_ + 1);
    w_s.resize(2 * state_num_ + 1);

    X_merge.resize(state_num_, 1);
    X_cv.resize(state_num_, 1);
    X_ctrv.resize(state_num_, 1);
    X_rm.resize(state_num_, 1);

    X_predict_sig_cv.resize(state_num_, 2 * state_num_ + 1);
    X_predict_sig_ctrv.resize(state_num_, 2 * state_num_ + 1);
    X_predict_sig_rm.resize(state_num_, 2 * state_num_ + 1);

    P_merge.resize(state_num_, state_num_);
    P_cv.resize(state_num_, state_num_);
    P_ctrv.resize(state_num_, state_num_);
    P_rm.resize(state_num_, state_num_);

    Q_cv.resize(state_num_, state_num_);
    Q_ctrv.resize(state_num_, state_num_);
    Q_rm.resize(state_num_, state_num_);

    Z_predict_cv.resize(meas_num_, 1);
    Z_predict_ctrv.resize(meas_num_, 1);
    Z_predict_rm.resize(meas_num_, 1);

    S_cv.resize(meas_num_, meas_num_);
    S_ctrv.resize(meas_num_, meas_num_);
    S_rm.resize(meas_num_, meas_num_);

    R_cv.resize(meas_num_, meas_num_);
    R_ctrv.resize(meas_num_, meas_num_);
    R_rm.resize(meas_num_, meas_num_);

    C_xz_cv.resize(state_num_, meas_num_);
    C_xz_ctrv.resize(state_num_, meas_num_);
    C_xz_rm.resize(state_num_, meas_num_);
}

UKF::~UKF()
{
    //
}

void UKF::Initialize(Eigen::VectorXd &init_meas)
{
    X_merge << 0, 0, 0, 0, 0.1;
    P_merge << 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 1;

    X_merge[0] = init_meas[0];
    X_merge[1] = init_meas[1];
    X_cv = X_ctrv = X_rm = X_merge;

    initial_mode_trans_prob << 0.9, 0.05, 0.05,
        0.05, 0.9, 0.05,
        0.05, 0.05, 0.9;
    mode_prob << 0.33, 0.33, 0.33;
    mode_trans_prob << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //初始化权重
    alpha = 0.001;
    beta = 2.0;
    k = 0.0;
    lambda = alpha * alpha * (state_num_ + k) - state_num_;
    w_s[0] = lambda / (lambda + state_num_);
    w_c[0] = lambda / (lambda + state_num_) + 1 - alpha * alpha + beta;
    for (int i = 1; i < 2 * state_num_ + 1; ++i)
    {
        w_s[i] = 1 / (2 * (state_num_ + lambda));
        w_c[i] = 1 / (2 * (state_num_ + lambda));
    }

    S_cv << 1, 0, 0, 1;
    S_ctrv << 1, 0, 0, 1;
    S_rm << 1, 0, 0, 1;

    Z_predict_cv << init_meas[0], init_meas[1];
    Z_predict_ctrv << init_meas[0], init_meas[1];
    Z_predict_rm << init_meas[0], init_meas[1];

    R_cv << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    R_ctrv << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    R_rm << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

    C_xz_cv << 0, 0,
        0, 0,
        0, 0,
        0, 0,
        0, 0;

    C_xz_ctrv << 0, 0,
        0, 0,
        0, 0,
        0, 0,
        0, 0;

    C_xz_rm << 0, 0,
        0, 0,
        0, 0,
        0, 0,
        0, 0;

    tracking_num = 1;
}

//预测阶段
void UKF::Predict(const double dt)
{
    InitialCovQ(dt);
    MixProbability();
    Interaction();
    PredictMotion(dt, Mode::CV);
    PredictMotion(dt, Mode::CTRV);
    PredictMotion(dt, Mode::RM);

    PredictMeasurement(Mode::CV);
    PredictMeasurement(Mode::CTRV);
    PredictMeasurement(Mode::RM);
}

void UKF::InitialCovQ(const double dt)
{
    if (tracking_num != TrackingState::Init)
    {
        return;
    }
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    double cos_yaw = cos(X_merge(2));
    double sin_yaw = sin(X_merge(2));
    double cos_2_yaw = cos_yaw * cos_yaw;
    double sin_2_yaw = sin_yaw * sin_yaw;
    double cos_sin = cos_yaw * sin_yaw;
    double cv_var_a = std_a_cv_ * std_a_cv_;
    double cv_var_yawdd = std_cv_yawdd_ * std_cv_yawdd_;

    double ctrv_var_a = std_a_ctrv_ * std_a_ctrv_;
    double ctrv_var_yawdd = std_ctrv_yawdd_ * std_ctrv_yawdd_;

    double rm_var_a = std_a_rm_ * std_a_rm_;
    double rm_var_yawdd = std_rm_yawdd_ * std_rm_yawdd_;

    Q_cv << 0.5 * 0.5 * dt_4 * cos_2_yaw * cv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
        0.5 * dt_3 * cos_yaw * cv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
        0.5 * 0.5 * dt_4 * sin_2_yaw * cv_var_a, 0.5 * dt_3 * sin_yaw * cv_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * cv_var_a,
        0.5 * dt_3 * sin_yaw * cv_var_a, dt_2 * cv_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * cv_var_yawdd,
        0.5 * dt_3 * cv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * cv_var_yawdd, dt_2 * cv_var_yawdd;
    Q_ctrv << 0.5 * 0.5 * dt_4 * cos_2_yaw * ctrv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
        0.5 * dt_3 * cos_yaw * ctrv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
        0.5 * 0.5 * dt_4 * sin_2_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, 0, 0,
        0.5 * dt_3 * cos_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, dt_2 * ctrv_var_a, 0, 0, 0, 0, 0,
        0.5 * 0.5 * dt_4 * ctrv_var_yawdd, 0.5 * dt_3 * ctrv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * ctrv_var_yawdd,
        dt_2 * ctrv_var_yawdd;
    Q_rm << 0.5 * 0.5 * dt_4 * cos_2_yaw * rm_var_a, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
        0.5 * dt_3 * cos_yaw * rm_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
        0.5 * 0.5 * dt_4 * sin_2_yaw * rm_var_a, 0.5 * dt_3 * sin_yaw * rm_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * rm_var_a,
        0.5 * dt_3 * sin_yaw * rm_var_a, dt_2 * rm_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * rm_var_yawdd,
        0.5 * dt_3 * rm_var_yawdd, 0, 0, 0, 0.5 * dt_3 * rm_var_yawdd, dt_2 * rm_var_yawdd;
}

//计算交互矩阵
void UKF::MixProbability()
{
    Eigen::VectorXd sum = Eigen::VectorXd(3);

    sum << initial_mode_trans_prob(0, 0) * mode_prob(0) + initial_mode_trans_prob(0, 1) * mode_prob(0) + initial_mode_trans_prob(0, 2) * mode_prob(0),
        initial_mode_trans_prob(1, 0) * mode_prob(1) + initial_mode_trans_prob(1, 1) * mode_prob(1) + initial_mode_trans_prob(1, 2) * mode_prob(1),
        initial_mode_trans_prob(2, 0) * mode_prob(2) + initial_mode_trans_prob(2, 1) * mode_prob(2) + initial_mode_trans_prob(2, 2) * mode_prob(2);

    for (int i = 0; i < mode_trans_prob.rows(); ++i)
    {
        for (int j = 0; j < mode_trans_prob.cols(); ++j)
        {

            mode_trans_prob(i, j) = initial_mode_trans_prob(i, j) * mode_prob(i) / sum(i);
        }
    }
}

//交互融合每个滤波器的X P
void UKF::Interaction()
{
    Eigen::MatrixXd X_tmp_cv = X_cv;
    Eigen::MatrixXd X_tmp_ctrv = X_ctrv;
    Eigen::MatrixXd X_tmp_rm = X_rm;

    Eigen::MatrixXd P_tmp_cv = P_cv;
    Eigen::MatrixXd P_tmp_ctrv = P_ctrv;
    Eigen::MatrixXd P_tmp_rm = P_rm;

    X_cv = mode_trans_prob(0, 0) * X_tmp_cv + mode_trans_prob(0, 1) * X_tmp_ctrv + mode_trans_prob(0, 1) * X_tmp_rm;
    X_ctrv = mode_trans_prob(1, 0) * X_tmp_cv + mode_trans_prob(1, 1) * X_tmp_ctrv + mode_trans_prob(1, 1) * X_tmp_rm;
    X_rm = mode_trans_prob(2, 0) * X_tmp_cv + mode_trans_prob(2, 1) * X_tmp_ctrv + mode_trans_prob(2, 1) * X_tmp_rm;

    P_cv = mode_trans_prob(0, 0) * (P_tmp_cv + (X_tmp_cv - X_cv) * (X_tmp_cv - X_cv).transpose()) +
           mode_trans_prob(0, 1) * (P_tmp_ctrv + (X_tmp_ctrv - X_cv) * (X_tmp_ctrv - X_cv).transpose()) +
           mode_trans_prob(0, 2) * (P_tmp_rm + (X_tmp_rm - X_cv) * (X_tmp_rm - X_cv).transpose());

    P_ctrv = mode_trans_prob(1, 0) * (P_tmp_cv + (X_tmp_cv - X_ctrv) * (X_tmp_cv - X_ctrv).transpose()) +
             mode_trans_prob(1, 1) * (P_tmp_ctrv + (X_tmp_ctrv - X_ctrv) * (X_tmp_ctrv - X_ctrv).transpose()) +
             mode_trans_prob(1, 2) * (P_tmp_rm + (X_tmp_rm - X_ctrv) * (X_tmp_rm - X_ctrv).transpose());

    P_cv = mode_trans_prob(2, 0) * (P_tmp_cv + (X_tmp_cv - X_rm) * (X_tmp_cv - X_rm).transpose()) +
           mode_trans_prob(2, 1) * (P_tmp_ctrv + (X_tmp_ctrv - X_rm) * (X_tmp_ctrv - X_rm).transpose()) +
           mode_trans_prob(2, 2) * (P_tmp_rm + (X_tmp_rm - X_rm) * (X_tmp_rm - X_rm).transpose());
}

void UKF::PredictMotion(const double dt, int mode)
{
    Eigen::MatrixXd X_sig = Eigen::MatrixXd(state_num_, 2 * state_num_ + 1);
    Eigen::MatrixXd X = Eigen::MatrixXd(state_num_, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd(state_num_, state_num_);

    if (mode = Mode::CV)
    {
        X = X_cv;
        P = P_cv;
    }
    else if (mode = Mode::CTRV)
    {
        X = X_ctrv;
        P = P_ctrv;
    }
    else if (mode = Mode::RM)
    {
        X = X_rm;
        P = P_rm;
    }

    X_sig.col(0) = X;
    Eigen::MatrixXd P_sqrt = P.llt().matrixL();

    for (int i = 1; i < state_num_ + 1; ++i)
    {
        X_sig.col(i) = X + sqrt(state_num_ + lambda) * P_sqrt.col(i - 1);

        if (X_sig.col(i)[3] < -PI)
        {
            X_sig.col(i)[3] += 2 * PI;
        }
        if (X_sig.col(i)[3] > PI)
        {
            X_sig.col(i)[3] -= 2 * PI;
        }
    }
    for (int i = state_num_ + 1; i < 2 * state_num_ + 1; ++i)
    {
        X_sig.col(i) = X - sqrt(state_num_ + lambda) * P_sqrt.col(i - state_num_ - 1);
        if (X_sig.col(i)[3] < -PI)
        {
            X_sig.col(i)[3] += 2 * PI;
        }
        if (X_sig.col(i)[3] > PI)
        {
            X_sig.col(i)[3] -= 2 * PI;
        }
    }

    if (mode == Mode::CV)
    {
        CV(X_sig, dt);
        P_cv.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = X_sig.col(i) - X_cv;
            if (diff(3) > PI)
            {
                diff(3) -= 2 * PI;
            }
            if (diff(3) < -PI)
            {
                diff(3) += 2 * PI;
            }
            P_cv += w_c(i) * diff * diff.transpose();
        }
        P_cv += Q_cv;
    }
    else if (mode == Mode::CTRV)
    {
        CTRV(X_sig, dt);
        P_ctrv.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = X_sig.col(i) - X_ctrv;
            if (diff(3) > PI)
            {
                diff(3) -= 2 * PI;
            }
            if (diff(3) < -PI)
            {
                diff(3) += 2 * PI;
            }
            P_ctrv += w_c(i) * diff * diff.transpose();
        }
        P_ctrv += Q_ctrv;
    }
    else if (mode == Mode::RM)
    {
        RM(X_sig, dt);
        P_rm.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = X_sig.col(i) - X_rm;
            if (diff(3) > PI)
            {
                diff(3) -= 2 * PI;
            }
            if (diff(3) < -PI)
            {
                diff(3) += 2 * PI;
            }
            P_rm += w_c(i) * diff * diff.transpose();
        }
        P_rm += Q_rm;
    }
}

void UKF::CV(const Eigen::MatrixXd &X_sig, const double dt)
{
    for (int i = 0; i < X_sig.cols(); ++i)
    {
        X_predict_sig_cv(0, i) = X_sig(0, i) + X_sig(2, i) * dt * sin(X_sig(3, i)); //X
        X_predict_sig_cv(1, i) = X_sig(1, i) + X_sig(2, i) * dt * cos(X_sig(3, i)); //Y
        X_predict_sig_cv(2, i) = X_sig(2, i);                                       //v
        X_predict_sig_cv(3, i) = X_sig(3, i);                                       //yaw
        X_predict_sig_cv(4, i) = 0;                                                 //dyaw

        X_cv += w_s(i) * X_predict_sig_cv.col(i);
    }
    if (X_cv(3) > PI)
    {
        X_cv(3) -= 2 * PI;
    }
    if (X_cv(3) < -PI)
    {
        X_cv(3) += 2 * PI;
    }
}

void UKF::CTRV(const Eigen::MatrixXd &X_sig, const double dt)
{
    for (int i = 0; i < X_sig.cols(); ++i)
    {
        //防止分母为0
        if (fabs(X_sig(4, i)) > 0.001)
        {
            X_predict_sig_ctrv(0, i) = X_sig(0, i) + X_sig(2, i) / X_sig(4, i) * (sin(dt * X_sig(4, i) + X_sig(3, i)) - sin(X_sig(3, i)));  //X
            X_predict_sig_ctrv(1, i) = X_sig(1, i) + X_sig(2, i) / X_sig(4, i) * (-cos(dt * X_sig(4, i) + X_sig(3, i)) + cos(X_sig(3, i))); //Y
        }
        else
        {
            X_predict_sig_ctrv(0, i) = X_sig(0, i) + X_sig(2, i) * dt * sin(X_sig(3, i)); //x
            X_predict_sig_ctrv(1, i) = X_sig(1, i) + X_sig(2, i) * dt * cos(X_sig(3, i)); //y
        }

        X_predict_sig_ctrv(2, i) = X_sig(2, i);                    //v
        X_predict_sig_ctrv(3, i) = X_sig(3, i) + dt * X_sig(4, i); //yaw
        X_predict_sig_ctrv(4, i) = X_sig(4, i);                    //dyaw

        if (X_predict_sig_ctrv(3, i) > PI)
        {
            X_predict_sig_ctrv(3, i) -= 2 * PI;
        }
        if (X_predict_sig_ctrv(3, i) < -PI)
        {
            X_predict_sig_ctrv(3, i) += 2 * PI;
        }

        X_ctrv += w_s(i) * X_predict_sig_ctrv.col(i);
    }
    if (X_ctrv(3) > PI)
    {
        X_ctrv(3) -= 2 * PI;
    }
    if (X_ctrv(3) < -PI)
    {
        X_ctrv(3) += 2 * PI;
    }
}

void UKF::RM(const Eigen::MatrixXd &X_sig, const double dt)
{
    for (int i = 0; i < X_sig.cols(); ++i)
    {
        X_predict_sig_rm(0, i) = X_sig(0, i); //x
        X_predict_sig_rm(1, i) = X_sig(1, i); //y
        X_predict_sig_rm(2, i) = 0;           //v
        X_predict_sig_rm(3, i) = X_sig(3, i); //yaw
        X_predict_sig_rm(4, i) = 0;           //dyaw

        X_rm += w_s(i) * X_predict_sig_rm.col(i);
    }

    if (X_rm(3) > PI)
    {
        X_rm(3) -= 2 * PI;
    }
    if (X_rm(3) < -PI)
    {
        X_rm(3) += 2 * PI;
    }
}

//预测测量量
void UKF::PredictMeasurement(int mode)
{
    Eigen::MatrixXd X(state_num_, 1);
    Eigen::MatrixXd P(state_num_, state_num_);
    Eigen::MatrixXd X_predict_meas_sig = Eigen::MatrixXd(state_num_, 2 * state_num_ + 1);
    if (mode == Mode::CV)
    {
        X = X_cv;
        P = P_cv;
    }
    if (mode == Mode::CTRV)
    {
        X = X_ctrv;
        P = P_ctrv;
    }
    if (mode == Mode::RM)
    {
        X = X_rm;
        P = P_rm;
    }

    X_predict_meas_sig.col(0) = X;

    Eigen::MatrixXd P_sqrt = P.llt().matrixL();
    for (int i = 1; i < state_num_ + 1; ++i)
    {
        X_predict_meas_sig.col(i) = X + sqrt(state_num_ + lambda) * P_sqrt.col(i);
        // if(X_predict_meas_sig.col(i)[3]<-PI)
        // {
        //     X_predict_meas_sig.col(i)[3]+=2*PI;
        // }
        // if(X_predict_meas_sig.col(i)[3]>PI)
        // {
        //     X_predict_meas_sig.col(i)[3]-=2*PI;
        // }
    }
    for (int i = state_num_ + 1; i < 2 * state_num_ + 1; ++i)
    {
        X_predict_meas_sig.col(i) = X - sqrt(state_num_ + lambda) * P_sqrt.col(i - state_num_ - 1);
        // if(X_predict_meas_sig.col(i)[3]<-PI)
        // {
        //     X_predict_meas_sig.col(i)[3]+=2*PI;
        // }
        // if(X_predict_meas_sig.col(i)[3]>PI)
        // {
        //     X_predict_meas_sig.col(i)[3]-=2*PI;
        // }
    }

    Eigen::MatrixXd Z_sig(meas_num_, 2 * state_num_ + 1);
    for (int i = 0; i < 2 * state_num_ + 1; ++i)
    {
        Z_sig(0, i) = X_predict_meas_sig(0, i); //x
        Z_sig(1, i) = X_predict_meas_sig(1, i); //y
    }

    if (mode == Mode::CV)
    {
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Z_predict_cv += w_s(i) * Z_sig.col(i);
        }
        S_cv.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = Z_sig.col(i) - Z_predict_cv;
            S_cv += w_c(i) * diff * diff.transpose();
        }
        S_cv += R_cv;
    }
    if (mode == Mode::CTRV)
    {
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Z_predict_ctrv += w_s(i) * Z_sig.col(i);
        }
        S_ctrv.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = Z_sig.col(i) - Z_predict_ctrv;
            S_ctrv += w_c(i) * diff * diff.transpose();
        }
        S_ctrv += R_ctrv;
    }
    if (mode == Mode::RM)
    {
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Z_predict_rm += w_s(i) * Z_sig.col(i);
        }
        S_rm.fill(0);
        for (int i = 0; i < 2 * state_num_ + 1; ++i)
        {
            Eigen::VectorXd diff = Z_sig.col(i) - Z_predict_rm;
            S_rm += w_c(i) * diff * diff.transpose();
        }
        S_rm += R_rm;
    }
}

void UKF::Update()
{
}