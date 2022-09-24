#ifndef VSOLVER_TRANSLATION_H
#define VSOLVER_TRANSLATION_H

#include <memory>
#include "slam_vio/Vsolver/vertex.h"

namespace Vsolver
{
/**
 * VSOLVER_TRANSLATION
 * parameters: tx ty tz 3 DoF
 * 
 */
class VertexTranslation : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexTranslation() : Vertex(3) {}

    std::string TypeInfo() const
    {
        return "VertexTranslation";
    }
};

} // namespace Vsolver
#endif
