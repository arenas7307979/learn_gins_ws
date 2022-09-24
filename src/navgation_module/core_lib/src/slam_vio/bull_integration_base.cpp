#include "slam_vio/bull_integration_base.h"
#include "ob_gins/common/rotation.h"

BullIntegrationBase::BullIntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_linearized_ba,
                                 const Eigen::Vector3d &_linearized_bg, double acc_n, double gyr_n, double acc_w, double gyr_w)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0}, linearized_ba{_linearized_ba},
      linearized_bg{_linearized_bg}, jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
      sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

{
    const double corr_time = 3600;
    last_un_gyr_dt.setZero();
    midPointIntegration_last_un_gyr_dt.setZero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
    noise.setIdentity();
    noise.block<3, 3>(0, 0) *= gyr_n * gyr_n; // nw
    noise.block<3, 3>(3, 3) *= acc_n * acc_n; // na
    noise.block<3, 3>(6, 6) *= (2 * gyr_w * gyr_w) / corr_time; // nbg
    noise.block<3, 3>(9, 9) *= (2 * acc_w * acc_w) / corr_time; // nba
}

BullIntegrationBase::BullIntegrationBase(std::shared_ptr<BullIntegrationBase> ori_integrartion) {
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

void BullIntegrationBase::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void BullIntegrationBase::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
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

void BullIntegrationBase::NormalEulerIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1,
                                                 const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p, const Sophus::SO3d &delta_q,
                                                 const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                                 Eigen::Vector3d &result_delta_p, Sophus::SO3d &result_delta_q, Eigen::Vector3d &result_delta_v,
                                                 Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{

    // std::cout << "NormalEulerIntegration" <<std::endl;
    //ROS_INFO("midpoint integration");
    Eigen::Vector3d un_gyr = gyr_1 - linearized_bg;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    Eigen::Vector3d imu_cur_dvel = (_acc_1 - linearized_ba) * _dt;
    Eigen::Vector3d imu_pre_dvel = (_acc_0 - linearized_ba) * _dt;
    Eigen::Vector3d imu_cur_dtheta = (un_gyr) * _dt;
    Eigen::Vector3d imu_pre_dtheta = (_gyr_0 - linearized_bg) * _dt;

    //TODO::Reduce repeated operations, dvfb and dtheta result from the outside SLAM system
    // 位置速度
    Eigen::Vector3d dvfb = (imu_cur_dvel) + 0.5 * imu_cur_dtheta.cross(imu_cur_dvel) + 1.0 / 12.0 * (imu_pre_dtheta.cross(imu_cur_dvel) + imu_pre_dvel.cross(imu_cur_dtheta));
    // 姿态
    Eigen::Vector3d dtheta = imu_cur_dtheta + 1.0 / 12.0 * imu_pre_dtheta.cross(imu_cur_dtheta);

    Eigen::Vector3d dvel = delta_q * dvfb;
    // std::cout << "vsolver in dvfb=" << dvfb << std::endl;
    // std::cout << "vsolver in delta_qORI=" << delta_q.unit_quaternion().coeffs() << std::endl;
    // std::cout << "vsolver in dtheta=" << dtheta << std::endl;
    // std::cout << "vsolver in dvel=" << dvel << std::endl;
    // std::cout << "vsolver in _dt=" << _dt << std::endl;

    // 预积分
    result_delta_p = delta_p + _dt * delta_v + 0.5 * _dt * dvel;
    result_delta_v = delta_v  + dvel;
    // 姿态
    result_delta_q = delta_q * Sophus::SO3d::exp(dtheta);
 
    // std::cout << "vsolver delta_state_.p=" << result_delta_p << std::endl;
    // std::cout << "vsolver delta_state_.v=" << result_delta_v << std::endl;
    // std::cout << "vsolver delta_state_.q=" << result_delta_q.unit_quaternion().coeffs() << std::endl;
    // std::cout << "vsolver delta_state_.q OB_GINS_VERSION=" << Sophus::SO3d((delta_q.unit_quaternion() * Rotation::rotvec2quaternion(dtheta)).normalized()).unit_quaternion().coeffs() << std::endl;

    if (update_jacobian)
    {
        // dp, dv, dq, dbg, dba

        Eigen::Matrix<double, 15, 15> phi;
        phi.setZero();


        // jacobian
        Eigen::Matrix3d Rot_result_delta_q = result_delta_q.unit_quaternion().matrix();
        // phi = I + F * dt
        phi.block<3, 3>(B_O_P, B_O_P) = Eigen::Matrix3d::Identity();
        phi.block<3, 3>(B_O_P, B_O_V) = Eigen::Matrix3d::Identity() * _dt;
        phi.block<3, 3>(B_O_V, B_O_V) = Eigen::Matrix3d::Identity();
        phi.block<3, 3>(B_O_V, B_O_R) = -Rot_result_delta_q * Sophus::SO3d::hat(imu_cur_dvel);
        phi.block<3, 3>(B_O_V, B_O_BA) = -Rot_result_delta_q * _dt;
        phi.block<3, 3>(B_O_R, B_O_R) =  Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(imu_cur_dtheta);
        phi.block<3, 3>(B_O_R, B_O_BG) = - Eigen::Matrix3d::Identity() * _dt;
        phi.block<3, 3>(B_O_BG, B_O_BG) =  Eigen::Matrix3d::Identity() * (1 - dt/3600);
        phi.block<3, 3>(B_O_BA, B_O_BA) =  Eigen::Matrix3d::Identity() * (1 - dt/3600);

        jacobian = phi * jacobian;

        // covariance
        Eigen::Matrix<double, 15, 12> gt;
        gt.setZero();
        gt.block<3, 3>(3, 3) = Rot_result_delta_q;
        gt.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
        gt.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
        gt.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
      
        Eigen::Matrix<double, 15, 15> Qk =  0.5 * _dt * (phi * gt * noise * gt.transpose() + gt * noise * gt.transpose() * phi.transpose());
        covariance = phi * covariance * phi.transpose() + Qk;
    }
}

void BullIntegrationBase::propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1) {
    //Update delta_state and integration change matrix covariance and jacobian
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Eigen::Vector3d result_delta_p;
    Sophus::SO3d result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    NormalEulerIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
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
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
}

// NOTE:: redisual without td term
Eigen::Matrix<double, 15, 1> BullIntegrationBase::evaluate(const Eigen::Vector3d &Pi, const Sophus::SO3d &Qi, const Eigen::Vector3d &Vi,
                                                       const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                                       const Sophus::SO3d &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj,
                                                       const Eigen::Vector3d &Bgj, const Eigen::Vector3d &Gw) {                                                        
    Eigen::Matrix<double, 15, 1> residuals;

    //-------OB_GINS
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(B_O_P, B_O_BG);
    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(B_O_P, B_O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(B_O_V, B_O_BG);
    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(B_O_V, B_O_BA);
    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(B_O_R, B_O_BG);

    // 零偏误差
    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    // 积分校正
    Sophus::SO3d corrected_delta_q = delta_q * Sophus::SO3d::exp(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    // Residuals
    residuals.block<3, 1>(B_O_P, 0) = Qi.inverse() * (0.5 * Gw * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    // residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.unit_quaternion().inverse() * Qi.unit_quaternion().inverse() * Qj.unit_quaternion()).vec();
    residuals.block<3, 1>(B_O_R, 0) = (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).log();
    residuals.block<3, 1>(B_O_V, 0) = Qi.inverse() * (Gw * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(B_O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(B_O_BG, 0) = Bgj - Bgi;
    return residuals;
}
