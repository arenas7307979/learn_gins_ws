#include <iostream>

#include "slam_vio/Vsolver/vertex_pose.h"
// #include "Vsolver/vertex_speedbias.h"
#include <sophus/se3.hpp>

#include "slam_vio/Vsolver/edge_gnss.h"
#include "slam_vio/sophus_extra.h"

namespace Vsolver {
using Sophus::SO3d;
// Vec3 EdgeGNSS::gravity_ = Vec3(0, 0, 9.8);

void EdgeGNSS::ComputeResidual() {
    VecX param_0 = verticies_[0]->Parameters(); // VertexPose_i
    Eigen::Map<const Sophus::SO3d> Qwi(param_0.data());
    Eigen::Map<const Eigen::Vector3d> Pwi(param_0.data() + 4);
    residual_.head<3>() = Pwi + (Qwi * t_bi_gnss) - local_gnss_pwgps_info;
}

void EdgeGNSS::ComputeJacobians() {
    VecX param_0 = verticies_[0]->Parameters();
    Eigen::Map<const Sophus::SO3d> Qwi(param_0.data());
    Eigen::Map<const Eigen::Vector3d> Pwi(param_0.data() + 4);

    //J[0] = d[residual_gnss] / d[pwi, qwi]
    {
        //d_residual / dpose_i
        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian_pose_i;
        jacobian_pose_i.setZero();
        jacobian_pose_i.block<3, 3>(0, 0).setIdentity();
        jacobian_pose_i.block<3, 3>(0, 3) = -Qwi.matrix() * Sophus::SO3d::hat(t_bi_gnss);
        jacobians_[0] = jacobian_pose_i;
    }
}

} // namespace Vsolver
