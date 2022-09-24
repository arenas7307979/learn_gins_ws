#ifndef VSOLVER_GNSS_H
#define VSOLVER_GNSS_H

#include <memory>
#include <string>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "slam_vio/Vsolver/edge.h"
#include "slam_vio/Vsolver/eigen_types.h"
#include "slam_vio/integration_base.h"

namespace Vsolver {

class EdgeGNSS : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit EdgeGNSS(const Eigen::Vector3d &local_gnss_pwgps_info_, const Eigen::Vector3d &lever_arm, const Eigen::Vector3d &blh_variance) :
        Edge(3, 1, std::vector<std::string>{"VertexPose"}) {
        local_gnss_pwgps_info = local_gnss_pwgps_info_;
        t_bi_gnss = lever_arm;
        sqrt_information_ = Eigen::MatrixXd::Zero(3, 3);
        sqrt_information_(0,0) = 1.0 / (blh_variance[0]);
        sqrt_information_(1,1) = 1.0 / (blh_variance[1]);
        sqrt_information_(2,2) = 1.0 / (blh_variance[2]);
        information_ = sqrt_information_.transpose() * sqrt_information_;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeGNSS";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

private:
    Eigen::Vector3d local_gnss_pwgps_info; //translation info : Pw_gnss ~ Pwb(phone platform)
    Eigen::Vector3d t_bi_gnss;
};
} // namespace Vsolver
#endif
