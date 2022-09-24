#include <iostream>

#include "slam_vio/Vsolver/vertex_pose.h"
// #include "Vsolver/vertex_speedbias.h"
#include <sophus/se3.hpp>

#include "slam_vio/Vsolver/edge_imu.h"
#include "slam_vio/sophus_extra.h"

namespace Vsolver {
using Sophus::SO3d;
// Vec3 EdgeImu::gravity_ = Vec3(0, 0, 9.8);

void EdgeImu::ComputeResidual() {
    VecX param_0 = verticies_[0]->Parameters(); // VertexPose_i
    VecX param_1 = verticies_[1]->Parameters(); // VertexSpeed_i
    VecX param_2 = verticies_[2]->Parameters(); // VertexBias_i
    VecX param_3 = verticies_[3]->Parameters(); // VertexPose_j
    VecX param_4 = verticies_[4]->Parameters(); // VertexSpeed_j
    VecX param_5 = verticies_[5]->Parameters(); // VertexBias_j

    Eigen::Map<const Sophus::SO3d> Qwi(param_0.data()), Qwj(param_3.data());
    Eigen::Map<const Eigen::Vector3d> Pwi(param_0.data() + 4), Pwj(param_3.data() + 4),
        Vi(param_1.data()), Bai(param_2.data()), Bgi(param_2.data() + 3), Vj(param_4.data()),
        Baj(param_5.data()), Bgj(param_5.data() + 3);

    residual_ = pre_integration_->evaluate(Pwi, Qwi, Vi, Bai, Bgi, Pwj, Qwj, Vj, Baj, Bgj, Gw);
}

void EdgeImu::ComputeJacobians() {
    VecX param_0 = verticies_[0]->Parameters(); // VertexPose_i
    VecX param_1 = verticies_[1]->Parameters(); // VertexSpeed_i
    VecX param_2 = verticies_[2]->Parameters(); // VertexBias_i
    VecX param_3 = verticies_[3]->Parameters(); // VertexPose_j
    VecX param_4 = verticies_[4]->Parameters(); // VertexSpeed_j
    VecX param_5 = verticies_[5]->Parameters(); // VertexBias_j
    Eigen::Map<const Sophus::SO3d> Qwi(param_0.data()), Qwj(param_3.data());
    Eigen::Map<const Eigen::Vector3d> Pwi(param_0.data() + 4), Pwj(param_3.data() + 4),
        Vi(param_1.data()), Bai(param_2.data()), Bgi(param_2.data() + 3), Vj(param_4.data()),
        Baj(param_5.data()), Bgj(param_5.data() + 3);

    double sum_dt = pre_integration_->sum_dt;
    Eigen::Matrix3d dp_dba = pre_integration_->jacobian.template block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.template block<3, 3>(O_P, O_BG);
    Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.template block<3, 3>(O_R, O_BG);
    Eigen::Matrix3d dv_dba = pre_integration_->jacobian.template block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.template block<3, 3>(O_V, O_BG);
    Sophus::SO3d Qiw = Qwi.inverse(), Qjw = Qwj.inverse();
    Eigen::Matrix3d Riw = Qiw.matrix(), I3x3 = Eigen::Matrix3d::Identity();
    Sophus::SO3d Qij = Qiw * Qwj;
    Sophus::SO3d Qji = Qjw * Qwi;

    Eigen::Vector3d dba = Bai - pre_integration_->linearized_ba;
    Eigen::Vector3d dbg = Bgi - pre_integration_->linearized_bg;

    Sophus::SO3d end_to_begin_deltaq_q =
        pre_integration_->delta_q * Sophus::SO3d::exp(dq_dbg * dbg);
    Eigen::Vector3d end_to_begin_delta_v = pre_integration_->delta_v + dv_dba * dba + dv_dbg * dbg;

    //**Note : corrected_delta from last_frame_timestamp to
    // cur_frame_timestamp(bk+1 to bk)
    Sophus::SO3d corrected_delta_q = end_to_begin_deltaq_q;

    if (pre_integration_->jacobian.maxCoeff() > 1e8 ||
        pre_integration_->jacobian.minCoeff() < -1e8) {
        std::cout << "imu numerical unstable in preintegration" << '\n';
        // std::cout << pre_integration->jacobian << '\n';
    }

    {
        // d_residual / dpose_i
        Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_i;
        jacobian_pose_i.setZero();
        jacobian_pose_i.block<3, 3>(O_P, O_P) = -Riw;
        jacobian_pose_i.block<3, 3>(O_P, O_R) =
            Sophus::SO3d::hat(Qiw * (0.5 * Gw * sum_dt * sum_dt + Pwj - Pwi - Vi * sum_dt));
        jacobian_pose_i.block<3, 3>(O_R, O_R) =
            -(Sophus::Qleft(Qji) * Sophus::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
        jacobian_pose_i.block<3, 3>(O_V, O_R) = Sophus::SO3d::hat(Qiw * (Gw * sum_dt + Vj - Vi));
        jacobians_[0] = jacobian_pose_i;
    }

    // d_residual / dv_i
    {
        Eigen::Matrix<double, 15, 3, Eigen::RowMajor> jacobian_speed;
        jacobian_speed.setZero();
        jacobian_speed.block<3, 3>(O_P, O_V - O_V) = -Riw * sum_dt;
        jacobian_speed.block<3, 3>(O_V, O_V - O_V) = -Riw;
        jacobians_[1] = jacobian_speed;
    }

    // d_residual / dba_bg
    {
        Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_bias_i;
        jacobian_bias_i.setZero();
        jacobian_bias_i.block<3, 3>(O_P, O_BA - O_V - 3) = -dp_dba;
        jacobian_bias_i.block<3, 3>(O_P, O_BG - O_V - 3) = -dp_dbg;
        jacobian_bias_i.block<3, 3>(O_R, O_BG - O_V - 3) =
            -Sophus::Qleft(Qji * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
        jacobian_bias_i.block<3, 3>(O_V, O_BA - O_V - 3) = -dv_dba;
        jacobian_bias_i.block<3, 3>(O_V, O_BG - O_V - 3) = -dv_dbg;
        jacobian_bias_i.block<3, 3>(O_BA, O_BA - O_V - 3) = -I3x3;
        jacobian_bias_i.block<3, 3>(O_BG, O_BG - O_V - 3) = -I3x3;
        jacobians_[2] = jacobian_bias_i;
    }

    // d_residual / dpose_j
    {
        Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_j;
        jacobian_pose_j.setZero();
        jacobian_pose_j.block<3, 3>(O_P, O_P) = Riw;
        jacobian_pose_j.block<3, 3>(O_R, O_R) =
            Sophus::Qleft(corrected_delta_q.inverse() * Qij).bottomRightCorner<3, 3>();
        jacobians_[3] = jacobian_pose_j;
    }

    // d_residual / dv_j
    {
        Eigen::Matrix<double, 15, 3, Eigen::RowMajor> jacobian_speed_j;
        jacobian_speed_j.setZero();
        jacobian_speed_j.block<3, 3>(O_V, O_V - O_V) = Riw;
        jacobians_[4] = jacobian_speed_j;
    }
    // d_residual / baj, bgj
    {
        Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_bias_j;
        jacobian_bias_j.setZero();
        jacobian_bias_j.block<3, 3>(O_BA, O_BA - O_V - 3) = I3x3;
        jacobian_bias_j.block<3, 3>(O_BG, O_BG - O_V - 3) = I3x3;
        jacobians_[5] = jacobian_bias_j;
    }
}

} // namespace Vsolver
