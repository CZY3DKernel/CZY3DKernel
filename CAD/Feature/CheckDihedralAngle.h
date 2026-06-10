#pragma once

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include "DihedralAngleType.h"

class CheckDihedralAngle
{
   public:
       /*
       * @brief calculate angle type between face F and face G along commonEdge, commonEdge from face F
       * @param 
       */
    DihedralAngleType AngleBetweenFaces(const TopoDS_Face& F, const TopoDS_Edge& commonEdge,
                                        const TopoDS_Face& G, const bool allowSmooth,
        const double smoothAngularTol) const;
   
    protected:
   
};
