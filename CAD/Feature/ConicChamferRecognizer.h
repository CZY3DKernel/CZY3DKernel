#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <Precision.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <vector>
#include <map>
#include <set>

#include "./ChamferInfo.h"

class AttributeAdjacentGraph;

class ConicChamferRecognizer
{
   public:
    // 构造函数
    ConicChamferRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag,
                            const TopoDS_Face& face);

    /*
     * @brief recognize if face is chamfer;
     * @param[out] feature
     * @return true if face is chamfer and set feature; false otherwise
     */
    bool IsChamfer(ChamferFeature& feature);

   private:

       bool HasSmoothEdges(const TopoDS_Face& face) const;

    bool CheckBeltLengthWidthRatio(const TopoDS_Face& face, const double beltLength,
                                      const double beltWidth) const;
       bool CheckAngleConditions(const TopoDS_Face& face,
                                 const std::vector<TopoDS_Face>& adjacentFaces) const;

    double CalculateEdgeLength(const TopoDS_Edge& edge) const;
       double CalculateAngleBetweenFaces(const TopoDS_Face& face1, const TopoDS_Face& face2,
                                         gp_Dir& normal1, gp_Dir& normal2) const;

    std::shared_ptr<AttributeAdjacentGraph> m_aag;

    const TopoDS_Face& m_face;
};
