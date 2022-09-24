#include "slam_vio/Vsolver/vertex_so3.h"
#include <sophus/se3.hpp>
//#include <iostream>

namespace Vsolver {

//parameters[q]
//delta [q]

void VertexSO3::Plus(const VecX &delta) {
    VecX &parameters = Parameters();
    // Qd q(parameters[3], parameters[0], parameters[1], parameters[2]);
    Eigen::Map<Sophus::SO3d> q_so3(parameters.data());

    //POITODO:: find nan bug
    if ( !std::isnan(delta[0]) && !std::isnan(delta[1]) && !std::isnan(delta[2]) )
    {
        q_so3 = q_so3 * Sophus::SO3d::exp(Vec3(delta[0], delta[1], delta[2])); // right multiplication with so3'
        q_so3.normalize();
    }
    else
    {
            std::cout << "\033[1;31mbold VertexSO3::Plus delta[0][1][2] nan \033[0m\n" << '\n';
    }
    parameters[0] = q_so3.data()[0];
    parameters[1] = q_so3.data()[1];
    parameters[2] = q_so3.data()[2];
    parameters[3] = q_so3.data()[3];
}

} // namespace Vsolver
