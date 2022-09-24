#ifndef VSOLVER_EDGELAST_BIAS_H
#define VSOLVER_EDGELAST_BIAS_H

#include <memory>
#include <string>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "slam_vio/Vsolver/edge.h"
#include "slam_vio/Vsolver/eigen_types.h"
#include "slam_vio/integration_base.h"

namespace Vsolver
{
class EdgeLastBias : public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit EdgeLastBias() : Edge(6, 1, std::vector<std::string>{"VertexBias"})
    {
        SetInformation(Eigen::Matrix3d::Identity());
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeLastBias";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

protected:
    static constexpr double IMU_GRY_BIAS_STD = 7200 / 3600.0 * M_PI / 180.0; // 7200 deg / hr
    static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;               // 20000 mGal
    static constexpr double IMU_SCALE_STD = 5.0e3 * 1.0e-6;                  // 5000 PPM
    static constexpr double IMU_ACC_Z_SCALE = 100;
    static constexpr double ODO_SCALE_STD = 2.0e4 * 1.0e-6; // 0.02
};
} // namespace Vsolver
#endif
