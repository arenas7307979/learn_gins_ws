#include <iostream>

#include "slam_vio/Vsolver/vertex_pose.h"
// #include "Vsolver/vertex_speedbias.h"
#include <sophus/se3.hpp>

#include "slam_vio/Vsolver/edge_imu_last_bias.h"
#include "slam_vio/sophus_extra.h"

namespace Vsolver {
void EdgeLastBias::ComputeResidual()
{
    VecX param_0 = verticies_[0]->Parameters(); // VertexPose_i
    Eigen::Map<const Eigen::Vector3d> Ba(param_0.data()), Bg(param_0.data() + 3);
    // bg, ba
    residual_[0] = Ba.x() / IMU_ACC_BIAS_STD;
    residual_[1] = Ba.y() / IMU_ACC_BIAS_STD;
    residual_[2] = Ba.z() / IMU_ACC_BIAS_STD;
    residual_[3] = Bg.x() / IMU_GRY_BIAS_STD;
    residual_[4] = Bg.y() / IMU_GRY_BIAS_STD;
    residual_[5] = Bg.z() / IMU_GRY_BIAS_STD;
}

void EdgeLastBias::ComputeJacobians()
{
    //J[0] = d[residual_gnss] / d[bai, qgi]
    {
        //d_residual / dpose_i
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian_pose_i;
        jacobian_pose_i.setZero();
        jacobian_pose_i(0,0) = 1.0 / IMU_ACC_BIAS_STD;
        jacobian_pose_i(1,1) = 1.0 / IMU_ACC_BIAS_STD;
        jacobian_pose_i(2,2) = 1.0 / IMU_ACC_BIAS_STD;
        jacobian_pose_i(3,3) = 1.0 / IMU_GRY_BIAS_STD;
        jacobian_pose_i(4,4) = 1.0 / IMU_GRY_BIAS_STD;
        jacobian_pose_i(5,5) = 1.0 / IMU_GRY_BIAS_STD;
        jacobians_[0] = jacobian_pose_i;
    }
}

} // namespace Vsolver
