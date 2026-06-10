#include "./ChamferRecognizerUtils.h"

#include <assert.h>
#include <vector>

#include <Geom_Curve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Standard_Type.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS_Edge.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Tool.hxx>
#include <Math_Matrix.hxx>
#include <Math_Vector.hxx>
#include <Math_Jacobi.hxx>
#include <Geom2d_Curve.hxx>
#include <Math_SVD.hxx>
#include <gp_Pln.hxx>
#include <Math_Gauss.hxx>
#include <GeomLProp_CLProps.hxx>
#include <Math_GaussLeastSquare.hxx>
#include <GC_MakeCircle.hxx>
#include <TopExp_Explorer.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <TopoDS.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <ShapeFix_Edge.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_OffsetSurface.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <ShapeAnalysis_Edge.hxx>

#include "./SmoothEdgesFinder.h"
#include "./AttributeAdjacentGraph.h"

void ChamferRecognizerUtils::FilterNonSmoothCompEdges(const TopoDS_Face& face, std::shared_ptr<AttributeAdjacentGraph> aag, std::vector<CompEdge>& compEdges)
{
    SmoothEdgesFinder smoothEdgeFinder(aag);

    TColStd_PackedMapOfInteger edgeIndices = smoothEdgeFinder.PerformForFace(face);

    int last = 0;
    for (int i = 0; i < compEdges.size(); ++i)
    {
        bool isSmooth = false;

        for (const TopoDS_Edge& edge : compEdges[i].GetEdges())
        {
            const int edgeId = aag->GetEdgeId(edge);
            if (edgeIndices.Contains(edgeId))
            {
                isSmooth = true;
                break;
            }
        }

        if (!isSmooth)
        {
            compEdges[last] = compEdges[i];
            ++last;
        }
    }

    compEdges.resize(last);
}


// 条件4：检查带长带宽比是否大于5
bool ChamferRecognizerUtils::CheckBeltLengthWidthRatio(const double beltLength,
                                                       const double beltWidth)
{
    const double ratio = 1.5;
    // 步骤3：检查比例
    return beltLength > 1.5 * beltWidth;
}
