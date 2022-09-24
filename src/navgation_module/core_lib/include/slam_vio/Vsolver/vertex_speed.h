#ifndef VSOLVER_SPEEDVERTEX_H
#define VSOLVER_SPEEDVERTEX_H

#include <memory>
#include "slam_vio/Vsolver/vertex.h"

namespace Vsolver {
/**
 * SpeedBias vertex
 * parameters: v, 3 DoF
 * 
 */
class VertexSpeed : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexSpeed() : Vertex(3) {}

    std::string TypeInfo() const {
        return "VertexSpeed";
    }

};

}
#endif
