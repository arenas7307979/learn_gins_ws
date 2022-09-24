#include "slam_vio/ceres/ceres_bull_imu_factor.h"
#include "slam_vio/sophus_extra.h"

CERES_BULL_IMUFactor::CERES_BULL_IMUFactor(BullIntegrationBasePtr _pre_integration, const Eigen::Vector3d& Gw_) : pre_integration(_pre_integration), Gw(Gw_)
{
}

bool CERES_BULL_IMUFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Sophus::SO3d> Qwi(parameters[0]), Qwj(parameters[3]);
    Eigen::Map<const Eigen::Vector3d> Pwi(parameters[0] + 4), Pwj(parameters[3] + 4),
        Vi(parameters[1]), Bai(parameters[2]), Bgi(parameters[2] + 3),
        Vj(parameters[4]), Baj(parameters[5]), Bgj(parameters[5] + 3);

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration->evaluate(Pwi, Qwi, Vi, Bai, Bgi,
                                         Pwj, Qwj, Vj, Baj, Bgj, Gw);

    Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();

    //sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    if (jacobians)
    {
        double sum_dt = pre_integration->sum_dt;
        Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(B_O_P, B_O_BA);
        Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(B_O_P, B_O_BG);
        Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(B_O_R, B_O_BG);
        Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(B_O_V, B_O_BA);
        Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(B_O_V, B_O_BG);

        Sophus::SO3d Qiw = Qwi.inverse(), Qjw = Qwj.inverse();
        Eigen::Matrix3d Riw = Qiw.matrix(), I3x3 = Eigen::Matrix3d::Identity();
        Sophus::SO3d corrected_delta_q = pre_integration->delta_q * Sophus::SO3d::exp(dq_dbg * (Bgi - pre_integration->linearized_bg));

        // if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
        // {
        //      std::cout << "numerical unstable in preintegration" << '\n';
        //     //std::cout << pre_integration->jacobian << '\n';
        // }


        //dr_15dim / dr_pose_i(t,R) [order follow update_load]
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();

            jacobian_pose_i.block<3, 3>(B_O_P, 0) = -Riw;
            jacobian_pose_i.block<3, 3>(B_O_P, 3) = Sophus::SO3d::hat(Qiw * (0.5 * Gw * sum_dt * sum_dt + Pwj - Pwi - Vi * sum_dt));
            jacobian_pose_i.block<3, 3>(B_O_R, 3) = -(Sophus::Qleft(Qjw * Qwi) * Sophus::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_pose_i.block<3, 3>(B_O_V, 3) = Sophus::SO3d::hat(Qiw * (Gw * sum_dt + Vj - Vi));

            jacobian_pose_i = sqrt_info * jacobian_pose_i;
        }

        //dr_15dim/dvi
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speedi(jacobians[1]);
            jacobian_speedi.setZero();
            jacobian_speedi.block<3, 3>(B_O_P, 0) = -Riw * sum_dt;
            jacobian_speedi.block<3, 3>(B_O_V, 0) = -Riw;
            jacobian_speedi = sqrt_info * jacobian_speedi;
        }

       //dr_15dim/dba, bg_i
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_i(jacobians[2]);
            jacobian_bias_i.setZero();
            jacobian_bias_i.block<3, 3>(B_O_P, 0) = -dp_dba;
            jacobian_bias_i.block<3, 3>(B_O_P, 3) = -dp_dbg;
            jacobian_bias_i.block<3, 3>(B_O_R, 3) = -Sophus::Qleft(Qjw * Qwi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_bias_i.block<3, 3>(B_O_V, 0) = -dv_dba;
            jacobian_bias_i.block<3, 3>(B_O_V, 3) = -dv_dbg;
            jacobian_bias_i.block<3, 3>(B_O_BA, 0) = -I3x3;
            jacobian_bias_i.block<3, 3>(B_O_BG, 3) = -I3x3;
            jacobian_bias_i = sqrt_info * jacobian_bias_i;
        }

        //dr_15dim / dr_pose_j(t,R) [order follow update_load]
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[3]);
            jacobian_pose_j.setZero();

            jacobian_pose_j.block<3, 3>(B_O_P, 0) = Riw;

            jacobian_pose_j.block<3, 3>(B_O_R, 3) = Sophus::Qleft(corrected_delta_q.inverse() * Qiw * Qwj).bottomRightCorner<3, 3>();

            jacobian_pose_j = sqrt_info * jacobian_pose_j;
        }

        if (jacobians[4])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[4]);
            jacobian_speed_j.setZero();
            jacobian_speed_j.block<3, 3>(B_O_V, 0) = Riw;
            jacobian_speed_j = sqrt_info * jacobian_speed_j;
        }


        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_j(jacobians[5]);
            jacobian_bias_j.setZero();
            jacobian_bias_j.block<3, 3>(B_O_BA, 0) = I3x3;
            jacobian_bias_j.block<3, 3>(B_O_BG, 3) = I3x3;
            jacobian_bias_j = sqrt_info * jacobian_bias_j;
        }
    }

    return true;
}

// void CERES_BULL_IMUFactor::check(double **parameters)
// {
//     double *res = new double[15];
//     double **jaco = new double *[4];
//     jaco[0] = new double[15 * 7];
//     jaco[1] = new double[15 * 9];
//     jaco[2] = new double[15 * 7];
//     jaco[3] = new double[15 * 9];
//     Evaluate(parameters, res, jaco);
//     puts("check CERES_BULL_IMUFactor begins");

//     puts("my CERES_BULL_IMUFactor");

//     std::cout << Eigen::Map<Eigen::Matrix<double, 15, 1>>(res).transpose() << std::endl
//               << std::endl;
//     std::cout << "=====" << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
//               << std::endl;
//     std::cout << "=====" << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[1]) << std::endl
//               << std::endl;
//     std::cout << "=====" << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
//               << std::endl;
//     std::cout << "=====" << std::endl;
//     std::cout << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[3]) << std::endl
//               << std::endl;
//     std::cout << "=====" << std::endl;
//     // std::cout << num_jacobian << std::endl;
// }


 bool CERES_LAST_IMU_BIAS_ERROR::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const{
     Eigen::Map<const Eigen::Vector3d> baj(parameters[0]), bgj(parameters[0] + 3);

     Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
     // ba, bg
     residual[0] = baj.x() / IMU_ACC_BIAS_STD;
     residual[1] = baj.y() / IMU_ACC_BIAS_STD;
     residual[2] = baj.z() / IMU_ACC_BIAS_STD;
     residual[3] = bgj.x() / IMU_GRY_BIAS_STD;
     residual[4] = bgj.y() / IMU_GRY_BIAS_STD;
     residual[5] = bgj.z() / IMU_GRY_BIAS_STD;

     if (jacobians)
     {

         //dr_15dim / dr_pose_i(t,R) [order follow update_load]
         if (jacobians[0])
         {
             //J[0] = d[residual_gnss] / d[bai, bgi]
             {
                 //d_residual / dpose_i
                 Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                 jacobian_pose_i.setZero();
                 jacobian_pose_i(0, 0) = 1.0 / IMU_ACC_BIAS_STD;
                 jacobian_pose_i(1, 1) = 1.0 / IMU_ACC_BIAS_STD;
                 jacobian_pose_i(2, 2) = 1.0 / IMU_ACC_BIAS_STD;
                 jacobian_pose_i(3, 3) = 1.0 / IMU_GRY_BIAS_STD;
                 jacobian_pose_i(4, 4) = 1.0 / IMU_GRY_BIAS_STD;
                 jacobian_pose_i(5, 5) = 1.0 / IMU_GRY_BIAS_STD;
                 std::cout << "residual=" << residual << std::endl;
                 std::cout << "jacobian_pose_i=" << jacobian_pose_i <<std::endl;
                //  exit(-1);
             }
         }
         
     }
     return true;
 }