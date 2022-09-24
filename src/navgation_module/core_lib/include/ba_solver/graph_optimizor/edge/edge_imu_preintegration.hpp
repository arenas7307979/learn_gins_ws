#pragma once

#include <ba_solver/graph_optimizor/edge.hpp>
#include "slam_vio/bull_integration_base.h"
#include <sophus/se3.hpp>

// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor
{

template <typename Scalar>
class EdgeIMU_Preintegration final : public EdgeBase<Scalar>
{
public:
    /* 构造函数与析构函数 */
    EdgeIMU_Preintegration(const BullIntegrationBasePtr &pre_integration,
                           const Vector3<Scalar> &Gw_)
        : EdgeBase<Scalar>(15, 6, nullptr), Gw(Gw_), pre_integration_(pre_integration)
    {
      this->SetInformation(pre_integration->covariance.inverse());
    }
    ~EdgeIMU_Preintegration() {}

public:
    /* 计算残差 */
    virtual void ComputeResidual(void) override;
    /* 计算残差对于每一个节点参数的雅可比矩阵 */
    virtual void ComputeJacobians(void) override;

    //TODO:: only support double type.
    BullIntegrationBasePtr pre_integration_;
    Vector3<Scalar> Gw;
};

/* 类成员方法定义如下 */
/* 计算残差 */
template <typename Scalar>
void EdgeIMU_Preintegration<Scalar>::ComputeResidual(void)
{
    VectorX<Scalar> param_0 = this->GetVertex(0)->GetParameters(); // VertexPose_i
    VectorX<Scalar> param_1 = this->GetVertex(1)->GetParameters(); // VertexSpeed_i
    VectorX<Scalar> param_2 = this->GetVertex(2)->GetParameters(); // VertexBias_i
    VectorX<Scalar> param_3 = this->GetVertex(3)->GetParameters(); // VertexPose_j
    VectorX<Scalar> param_4 = this->GetVertex(4)->GetParameters(); // VertexSpeed_j
    VectorX<Scalar> param_5 = this->GetVertex(5)->GetParameters(); // VertexBias_j

    Eigen::Map<const Sophus::SO3<Scalar>> Qwi(param_0.data()), Qwj(param_3.data());
    Eigen::Map<const Vector3<Scalar>> Pwi(param_0.data() + 4), Pwj(param_3.data() + 4),
        Vi(param_1.data()), Bai(param_2.data()), Bgi(param_2.data() + 3), Vj(param_4.data()),
        Baj(param_5.data()), Bgj(param_5.data() + 3);

    //only support double type.
    Eigen::Matrix<Scalar, 15, 1> r = this->pre_integration_->evaluate(Pwi, Qwi, Vi, Bai, Bgi, Pwj, Qwj, Vj, Baj, Bgj, this->Gw);
    this->SetResidual(r);
}

/* 计算残差对于每一个节点参数的雅可比矩阵 */
template <typename Scalar>
void EdgeIMU_Preintegration<Scalar>::ComputeJacobians(void)
{
    // 从节点中提取参数

    // MatrixX<Scalar> jacobian0 = jacobian_gnss;
    // this->SetJacobian(0, jacobian0);

    //==========================
    VectorX<Scalar> param_0 = this->GetVertex(0)->GetParameters(); // VertexPose_i
    VectorX<Scalar> param_1 = this->GetVertex(1)->GetParameters(); // VertexSpeed_i
    VectorX<Scalar> param_2 = this->GetVertex(2)->GetParameters(); // VertexBias_i
    VectorX<Scalar> param_3 = this->GetVertex(3)->GetParameters(); // VertexPose_j
    VectorX<Scalar> param_4 = this->GetVertex(4)->GetParameters(); // VertexSpeed_j
    VectorX<Scalar> param_5 = this->GetVertex(5)->GetParameters(); // VertexBias_j
    Eigen::Map<const Sophus::SO3<Scalar>> Qwi(param_0.data()), Qwj(param_3.data());
    Eigen::Map<const Vector3<Scalar>> Pwi(param_0.data() + 4), Pwj(param_3.data() + 4),
        Vi(param_1.data()), Bai(param_2.data()), Bgi(param_2.data() + 3), Vj(param_4.data()),
        Baj(param_5.data()), Bgj(param_5.data() + 3);

    Scalar sum_dt = this->pre_integration_->sum_dt;
    Matrix3<Scalar> dp_dba = this->pre_integration_->jacobian.template block<3, 3>(B_O_P, B_O_BA);
    Matrix3<Scalar> dp_dbg = this->pre_integration_->jacobian.template block<3, 3>(B_O_P, B_O_BG);
    Matrix3<Scalar> dq_dbg = this->pre_integration_->jacobian.template block<3, 3>(B_O_R, B_O_BG);
    Matrix3<Scalar> dv_dba = this->pre_integration_->jacobian.template block<3, 3>(B_O_V, B_O_BA);
    Matrix3<Scalar> dv_dbg = this->pre_integration_->jacobian.template block<3, 3>(B_O_V, B_O_BG);
    Sophus::SO3<Scalar> Qiw = Qwi.inverse();
    Matrix3<Scalar> Riw = Qiw.unit_quaternion().toRotationMatrix();
    
    Matrix3<Scalar> I3x3 = Eigen::Matrix<Scalar, 3, 3>::Identity();
    Sophus::SO3<Scalar> Qij = Qiw * Qwj;
    Sophus::SO3<Scalar> Qji = Qwj.inverse() * Qwi;
    Vector3<Scalar> dba = Bai - this->pre_integration_->linearized_ba;
    Vector3<Scalar> dbg = Bgi - this->pre_integration_->linearized_bg;

    //corrected_q_
    Sophus::SO3<Scalar> corrected_delta_q = this->pre_integration_->delta_q * Sophus::SO3<Scalar>::exp(dq_dbg * dbg);

    if (this->pre_integration_->jacobian.maxCoeff() > 1e8 ||
        this->pre_integration_->jacobian.minCoeff() < -1e8)
    {
        std::cout << "imu numerical unstable in preintegration" << '\n';
        // std::cout << pre_integration->jacobian << '\n';
    }

    // Block of size (p,q), starting at (i,j)	
    // matrix.block(i,j,p,q);
    // matrix.block<p,q>(i,j);

    {
      // d_residual / dpose_i[twi, qwi]
      Eigen::Matrix<Scalar, 15, 6, Eigen::RowMajor> jacobian_pose_i;
      jacobian_pose_i.setZero();

      jacobian_pose_i.block(B_O_P, 0, 3, 3) = -Riw;
      jacobian_pose_i.block(B_O_P, 3, 3, 3) = Sophus::SO3d::hat(Riw * (0.5 * this->Gw * sum_dt * sum_dt + Pwj - Pwi - Vi * sum_dt));
      jacobian_pose_i.block(B_O_R, 3, 3, 3) =-(Sophus::Qleft(Qji) * Sophus::Qright(corrected_delta_q)).bottomRightCorner(3, 3);
      jacobian_pose_i.block(B_O_V, 3, 3, 3) = Sophus::SO3d::hat(Riw * (this->Gw * sum_dt + Vj - Vi));
      this->SetJacobian(0, jacobian_pose_i);
    }

    // d_residual / dv_i
    {
        Eigen::Matrix<Scalar, 15, 3, Eigen::RowMajor> jacobian_speed;
        jacobian_speed.setZero();
        jacobian_speed.block(B_O_P, 0, 3, 3) = -Riw * sum_dt; //drp/dvi
        jacobian_speed.block(B_O_V, 0, 3, 3) = -Riw;          //drv/dvi
        this->SetJacobian(1, jacobian_speed);
    }

    // d_residual / dba_bg
    //B_O_BA - B_O_V - 3 : 分母順序依照輸入vertex, 與ob_gins殘差順序無關
    {
        Eigen::Matrix<Scalar, 15, 6, Eigen::RowMajor> jacobian_bias_i;
        jacobian_bias_i.setZero();
        jacobian_bias_i.block(B_O_P, 0, 3, 3) = -dp_dba; //rp/bai
        jacobian_bias_i.block(B_O_P, 3, 3, 3) = -dp_dbg; //rp/bgi
        jacobian_bias_i.block(B_O_R, 3, 3, 3) =
            -Sophus::Qleft(Qji * corrected_delta_q).bottomRightCorner(3, 3) * dq_dbg; //rq/bgi
        jacobian_bias_i.block(B_O_V, 0, 3, 3) = -dv_dba;                                //rv/bai
        jacobian_bias_i.block(B_O_V, 3, 3, 3) = -dv_dbg;                                //rv/bgi
        jacobian_bias_i.block(B_O_BA, 0, 3, 3) = -I3x3;                                 //rba/bai
        jacobian_bias_i.block(B_O_BG, 3, 3, 3) = -I3x3;                                 //rbg/bgi
        this->SetJacobian(2, jacobian_bias_i);
    }

    // d_residual / dpose_j[twj, qwj]
    {
        Eigen::Matrix<Scalar, 15, 6, Eigen::RowMajor> jacobian_pose_j;
        jacobian_pose_j.setZero();
        jacobian_pose_j.block(B_O_P, 0, 3, 3) = Riw;                                                                        //rq/twj
        jacobian_pose_j.block(B_O_R, 3, 3, 3) = Sophus::Qleft(corrected_delta_q.inverse() * Qij).bottomRightCorner(3, 3); //rq/qwj
        this->SetJacobian(3, jacobian_pose_j);
    }

    // d_residual / dv_j
    {
        Eigen::Matrix<Scalar, 15, 3, Eigen::RowMajor> jacobian_speed_j;
        jacobian_speed_j.setZero();
        jacobian_speed_j.block(B_O_V, 0, 3, 3) = Riw; //rv/vj
        this->SetJacobian(4, jacobian_speed_j);
    }

    // d_residual / baj, bgj
    {
        Eigen::Matrix<Scalar, 15, 6, Eigen::RowMajor> jacobian_bias_j;
        jacobian_bias_j.setZero();
        jacobian_bias_j.block(B_O_BA, 0, 3, 3) = I3x3; //rba/baj
        jacobian_bias_j.block(B_O_BG, 3, 3, 3) = I3x3; //rbg/bgj
        this->SetJacobian(5, jacobian_bias_j);
    }
}

} // namespace GraphOptimizor