#pragma once

#include <ba_solver/graph_optimizor/edge.hpp>


// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor {
template <typename Scalar>
class EdgeBiasLast final : public EdgeBase<Scalar>
{
public:
  /* 构造函数与析构函数 */
  EdgeBiasLast() : EdgeBase<Scalar>(6, 1, nullptr)
  {

  }
  ~EdgeBiasLast() {}

public:
  /* 计算残差 */
  virtual void ComputeResidual(void) override;
  /* 计算残差对于每一个节点参数的雅可比矩阵 */
  virtual void ComputeJacobians(void) override;

  //TODO:: add noise parameter
  static constexpr double IMU_GRY_BIAS_STD =7200 / 3600.0 * M_PI / 180.0;                            // 7200 deg / hr
  static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5; // 20000 mGal
  static constexpr double IMU_SCALE_STD = 5.0e3 * 1.0e-6;    // 5000 PPM
  static constexpr double IMU_ACC_Z_SCALE = 100;
  static constexpr double ODO_SCALE_STD = 2.0e4 * 1.0e-6; // 0.02
};

    /* 类成员方法定义如下 */
    /* 计算残差 */
    template<typename Scalar>
    void EdgeBiasLast<Scalar>::ComputeResidual(void) {
        VectorX<Scalar> param = this->GetVertex(0)->GetParameters();
        Eigen::Map<const Vector3<Scalar>> Ba(param.data()), Bg(param.data() + 3);
        Vector6<Scalar> r;
        // bg, ba
        r[0] = Ba.x() / this->IMU_ACC_BIAS_STD;
        r[1] = Ba.y() / this->IMU_ACC_BIAS_STD;
        r[2] = Ba.z() / this->IMU_ACC_BIAS_STD;
        r[3] = Bg.x() / this->IMU_GRY_BIAS_STD;
        r[4] = Bg.y() / this->IMU_GRY_BIAS_STD;
        r[5] = Bg.z() / this->IMU_GRY_BIAS_STD;
        this->SetResidual(r);
    }

    /* 计算残差对于每一个节点参数的雅可比矩阵 */
    template <typename Scalar>
    void EdgeBiasLast<Scalar>::ComputeJacobians(void)
    {

      Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian_last_bias = Eigen::Matrix<Scalar, 6, 6, Eigen::RowMajor>::Zero();
      jacobian_last_bias(0, 0) = 1.0 / this->IMU_ACC_BIAS_STD;
      jacobian_last_bias(1, 1) = 1.0 / this->IMU_ACC_BIAS_STD;
      jacobian_last_bias(2, 2) = 1.0 / this->IMU_ACC_BIAS_STD;
      jacobian_last_bias(3, 3) = 1.0 / this->IMU_GRY_BIAS_STD;
      jacobian_last_bias(4, 4) = 1.0 / this->IMU_GRY_BIAS_STD;
      jacobian_last_bias(5, 5) = 1.0 / this->IMU_GRY_BIAS_STD;
      this->SetJacobian(0, jacobian_last_bias);
    }
}