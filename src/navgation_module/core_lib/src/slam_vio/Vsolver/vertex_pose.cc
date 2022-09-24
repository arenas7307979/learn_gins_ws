#include "slam_vio/Vsolver/vertex_pose.h"
#include <sophus/se3.hpp>
//#include <iostream>

namespace Vsolver
{

    //parameters[q, p]
    //delta [p, q]

    void VertexPose::Plus(const VecX &delta)
    {
        VecX &parameters = Parameters();

        double sum_value_t = delta.head<3>().sum();
        double sum_value_r = delta.tail<3>().sum();
        if (!std::isnan(sum_value_t))
        {
            parameters.tail<3>() += delta.head<3>();
        }
        else
        {
            std::cout << "\033[1;31 VertexPose::Plus delta[0][1][2] nan \033[0m\n"
                      << '\n';
        }

        Eigen::Map<Sophus::SO3d> q_so3(parameters.data());
        //POITODO:: find nan bug
        if (!std::isnan(sum_value_r))
        {
            q_so3 = q_so3 * Sophus::SO3d::exp(Vec3(delta[3], delta[4], delta[5])); // right multiplication with so3'
            // q_so3.normalize();
        }
        else
        {
            std::cout << "\033[1;31mbold VertexPose::Plus delta[3][4][5] nan \033[0m\n"
                      << '\n';
        }
  
        parameters[0] = q_so3.data()[0];
        parameters[1] = q_so3.data()[1];
        parameters[2] = q_so3.data()[2];
        parameters[3] = q_so3.data()[3];
    }
} // namespace Vsolver
