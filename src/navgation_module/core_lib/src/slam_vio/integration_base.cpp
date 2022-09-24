#include "slam_vio/integration_base.h"

IntegrationBase::IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_linearized_ba,
                                 const Eigen::Vector3d &_linearized_bg, double acc_n, double gyr_n, double acc_w, double gyr_w)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0}, linearized_ba{_linearized_ba},
      linearized_bg{_linearized_bg}, jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
      sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

{
    last_un_gyr_dt.setZero();
    midPointIntegration_last_un_gyr_dt.setZero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
    noise.setZero();
    noise.block<3, 3>(0, 0) = (acc_n * acc_n) * I3x3;
    noise.block<3, 3>(3, 3) = (gyr_n * gyr_n) * I3x3;
    noise.block<3, 3>(6, 6) = (acc_n * acc_n) * I3x3;
    noise.block<3, 3>(9, 9) = (gyr_n * gyr_n) * I3x3;
    noise.block<3, 3>(12, 12) = (acc_w * acc_w) * I3x3;
    noise.block<3, 3>(15, 15) = (gyr_w * gyr_w) * I3x3;
    // covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 10;
    // covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.52358331;
}

IntegrationBase::IntegrationBase(std::shared_ptr<IntegrationBase> ori_integrartion) {
    dt = ori_integrartion->dt;
    acc_0 = ori_integrartion->acc_0;
    gyr_0 = ori_integrartion->gyr_0;
    acc_1 = ori_integrartion->acc_1;
    gyr_1 = ori_integrartion->gyr_1;
    linearized_acc = ori_integrartion->linearized_acc;
    linearized_gyr = ori_integrartion->linearized_gyr;
    linearized_ba = ori_integrartion->linearized_ba;
    linearized_bg = ori_integrartion->linearized_bg;

    jacobian = ori_integrartion->jacobian;
    covariance = ori_integrartion->covariance;
    noise = ori_integrartion->noise;
    sum_dt = ori_integrartion->sum_dt;
    delta_p = ori_integrartion->delta_p;
    delta_q = ori_integrartion->delta_q;
    delta_v = ori_integrartion->delta_v;
    last_un_gyr_dt.setZero();
    midPointIntegration_last_un_gyr_dt.setZero();
    for (int i = 0; i < ori_integrartion->dt_buf.size(); i++) {
        dt_buf.push_back(ori_integrartion->dt_buf[i]);
        acc_buf.push_back(ori_integrartion->acc_buf[i]);
        gyr_buf.push_back(ori_integrartion->gyr_buf[i]);
    }
};

void IntegrationBase::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void IntegrationBase::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) {
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setQuaternion(Eigen::Quaterniond::Identity());
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
}

void IntegrationBase::midPointIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1,
                                          const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p, const Sophus::SO3d &delta_q,
                                          const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                          Eigen::Vector3d &result_delta_p, Sophus::SO3d &result_delta_q, Eigen::Vector3d &result_delta_v,
                                          Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{
    //ROS_INFO("midpoint integration");
    Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Eigen::Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Sophus::SO3d(Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2));
    Eigen::Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if(update_jacobian)
    {
        Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Eigen::Vector3d a_0_x = _acc_0 - linearized_ba;
        Eigen::Vector3d a_1_x = _acc_1 - linearized_ba;
        Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

        R_w_x<<0, -w_x(2), w_x(1),
            w_x(2), 0, -w_x(0),
            -w_x(1), w_x(0), 0;
        R_a_0_x<<0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x<<0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        Eigen::Matrix<double, 15, 15> F;
        F.setZero();
        Eigen::Matrix3d deltaR = delta_q.unit_quaternion().toRotationMatrix(), result_delta_R = result_delta_q.unit_quaternion().toRotationMatrix();
        F.block<3, 3>(0, 0) = I3x3;
        F.block<3, 3>(0, 3) = -0.25 * deltaR * R_a_0_x * _dt * _dt +
                              -0.25 * result_delta_R * R_a_1_x * (I3x3 - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = I3x3 * _dt;
        F.block<3, 3>(0, 9) = -0.25 * (deltaR + result_delta_R) * _dt * _dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_R * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = I3x3 - R_w_x * _dt;
        F.block<3, 3>(3, 12) = -1.0 * I3x3 * _dt;
        F.block<3, 3>(6, 3) = -0.5 * deltaR * R_a_0_x * _dt +
                              -0.5 * result_delta_R * R_a_1_x * (I3x3 - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = I3x3;
        F.block<3, 3>(6, 9) = -0.5 * (deltaR + result_delta_R) * _dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_R * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 9) = I3x3;
        F.block<3, 3>(12, 12) = I3x3;
        //cout<<"A"<<endl<<A<<endl;

        Eigen::Matrix<double,15,18> V;
        V.setZero();
        V.block<3, 3>(0, 0) = 0.25 * deltaR * _dt * _dt;
        V.block<3, 3>(0, 3) = 0.25 * -result_delta_R * R_a_1_x * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 6) = 0.25 * result_delta_R * _dt * _dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * I3x3 * _dt;
        V.block<3, 3>(3, 9) = 0.5 * I3x3 * _dt;
        V.block<3, 3>(6, 0) = 0.5 * deltaR * _dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_R * R_a_1_x * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_R * _dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = I3x3 * _dt;
        V.block<3, 3>(12, 15) = I3x3 * _dt;

        jacobian = F * jacobian;
        covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
}

void IntegrationBase::propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1) {
     dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Eigen::Vector3d result_delta_p;
    Sophus::SO3d result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                        linearized_ba, linearized_bg,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg, 1);

    //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
    //                    linearized_ba, linearized_bg);
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
}

// NOTE:: redisual without td term
Eigen::Matrix<double, 15, 1> IntegrationBase::evaluate(const Eigen::Vector3d &Pi, const Sophus::SO3d &Qi, const Eigen::Vector3d &Vi,
                                                       const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                                       const Sophus::SO3d &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj,
                                                       const Eigen::Vector3d &Bgj, const Eigen::Vector3d &Gw) {                                                        
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    Sophus::SO3d corrected_delta_q = delta_q * Sophus::SO3d::exp(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * Gw * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).log();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (Gw * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
}

// NOTE:: redisual with td term
Eigen::Matrix<double, 15, 1> IntegrationBase::evaluate(const Eigen::Vector3d &Pi, const Sophus::SO3d &Qi, const Eigen::Vector3d &Vi,
                                                       const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                                       const Sophus::SO3d &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj,
                                                       const Eigen::Vector3d &Bgj, const double &td_para, const Eigen::Vector3d &Gw,
                                                       const Eigen::Vector3d &w_begin, const Eigen::Vector3d &w_end, const Eigen::Vector3d &acc_begin,
                                                       const Eigen::Vector3d &acc_end, const double &t_begin, const double &t_end,
                                                       const double &td_fix_last) {

    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    Eigen::Vector3d avg_bias_acc = (Bai + Baj) / 2;
    Eigen::Vector3d avg_bias_gyro = (Bai + Baj) / 2;

    Eigen::Vector3d gyro_end_nobias = w_end - linearized_bg;
    Eigen::Vector3d gyro_begin_nobias = w_begin - linearized_bg;
    Eigen::Vector3d acc_end_nobias = acc_end - linearized_ba;
    Eigen::Vector3d acc_begin_nobias = acc_begin - linearized_ba;
    double Td = td_para - td_fix_last;

    Sophus::SO3d Qi_inv = Qi.inverse();
    Sophus::SO3d end_to_begin_deltaq_q = delta_q * Sophus::SO3d::exp(dq_dbg * dbg);
    Eigen::Vector3d end_to_begin_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d end_to_begin_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    //**Note : Update corrected_delta_q/v/p based on Td Factor
    // end and begin represenst start and end points of pre-integration at time
    // of t_end and t_begin
    //    |--------|-------|------|
    //   bk      begin    end    bk+1
    Sophus::SO3d end_to_bk1_deltaq_q = Sophus::SO3d(Eigen::Quaterniond(1, gyro_end_nobias(0) * Td / 2, gyro_end_nobias(1) * Td / 2, gyro_end_nobias(2) * Td / 2));
    Sophus::SO3d bk_to_begin_deltaq_q = Sophus::SO3d(Eigen::Quaterniond(1, gyro_begin_nobias(0) * Td / 2, gyro_begin_nobias(1) * Td / 2, gyro_begin_nobias(2) * Td / 2));

    //**Note : corrected_delta from last_frame_timestamp to
    // cur_frame_timestamp(bk+1 to bk)
    Sophus::SO3d corrected_delta_q = bk_to_begin_deltaq_q * end_to_begin_deltaq_q * end_to_bk1_deltaq_q.inverse();
    Eigen::Vector3d corrected_delta_v = bk_to_begin_deltaq_q * (end_to_begin_delta_v + (acc_begin_nobias * Td) - (end_to_begin_deltaq_q * acc_end_nobias * Td));
    Eigen::Vector3d corrected_delta_p =
        bk_to_begin_deltaq_q * (end_to_begin_delta_p + (acc_begin_nobias * (t_end - t_begin) * Td) - (end_to_begin_delta_v * Td));

    residuals.block<3, 1>(O_P, 0) = Qi_inv * (0.5 * Gw * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = (corrected_delta_q.inverse() * (Qi_inv * Qj)).log();
    residuals.block<3, 1>(O_V, 0) = Qi_inv * (Gw * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
}
