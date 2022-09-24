#pragma once

#include <ba_solver/graph_optimizor/edge.hpp>


// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor {  
    template<typename Scalar>
    class EdgeGNSS final : public EdgeBase<Scalar> {
    public:
      /* 构造函数与析构函数 */
      EdgeGNSS(const Vector3<Scalar>& gnss_obs,
               const Vector3<Scalar>& lever_arm,
               const Vector3<Scalar>& blh_variance)
        : EdgeBase<Scalar>(3, 1, nullptr),lever_arm_(lever_arm)
      {
        this->SetObservation(gnss_obs);

        Matrix3<Scalar> information;
        Matrix3<Scalar> sqrt_information_ = Matrix3<Scalar>::Zero(3, 3);
        sqrt_information_(0, 0) = 1.0 / blh_variance[0];
        sqrt_information_(1, 1) = 1.0 / blh_variance[1];
        sqrt_information_(2, 2) = 1.0 / blh_variance[2];
        information = sqrt_information_.transpose() * sqrt_information_;
        this->SetInformation(information);
      }
        ~EdgeGNSS() {}

    public:
        /* 计算残差 */
        virtual void ComputeResidual(void) override;
        /* 计算残差对于每一个节点参数的雅可比矩阵 */
        virtual void ComputeJacobians(void) override;
    
        // lever_arm_ = translation from body to gnss
        Vector3<Scalar> lever_arm_;
    };

    /* 类成员方法定义如下 */
    /* 计算残差 */
    template<typename Scalar>
    void EdgeGNSS<Scalar>::ComputeResidual(void) {
        VectorX<Scalar> param = this->GetVertex(0)->GetParameters();
        Eigen::Map<const Sophus::SO3<Scalar>> Qwi(param.data());
        Eigen::Map<const Vector3<Scalar>> Pwi(param.data() + 4);
        Vector3<Scalar> r;
        r = Pwi + (Qwi * this->lever_arm_) - this->GetObservation();
        this->SetResidual(r);
    }

    /* 计算残差对于每一个节点参数的雅可比矩阵 */
    template <typename Scalar>
    void EdgeGNSS<Scalar>::ComputeJacobians(void)
    {
      // 从节点中提取参数
      VectorX<Scalar> param = this->GetVertex(0)->GetParameters();
      Eigen::Map<const Sophus::SO3<Scalar>> Qwi(param.data());
      Eigen::Map<const Vector3<Scalar>> Pwi(param.data() + 4);

      Eigen::Matrix<Scalar, 3, 6, Eigen::RowMajor> jacobian_gnss = Eigen::Matrix<Scalar, 3, 6, Eigen::RowMajor>::Zero();

      jacobian_gnss.block(0, 0, 3, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity();
      jacobian_gnss.block(0, 3, 3, 3) = -Qwi.matrix() * Sophus::SO3<Scalar>::hat(this->lever_arm_);
      this->SetJacobian(0, jacobian_gnss);
    }
}