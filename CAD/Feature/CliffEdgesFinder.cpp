
#include "./CliffEdgesFinder.h"


// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>

#include "./AttributeAdjacentGraph.h"
#include "./FeatureCommon.h"

//-----------------------------------------------------------------------------

CliffEdgesFinder::CliffEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag)
    : m_aag(aag)
{
}

//-----------------------------------------------------------------------------

TColStd_PackedMapOfInteger CliffEdgesFinder::PerformForFace(
    const TopoDS_Face& A, const TColStd_PackedMapOfInteger& smoothEdgeIndices)
{
    TColStd_PackedMapOfInteger result;

    // Get face of interest and its host surface.
    BRepAdaptor_Surface AS(A, false);
    
    // Get all smoothly connected neighbor faces.
    for (TopExp_Explorer exp(A, TopAbs_EDGE);exp.More(); exp.Next() )
    {
        const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        const int edgeId = m_aag->GetEdgeId(edge);
        if (smoothEdgeIndices.Contains(edgeId))
        {
            continue;
        }

        // Get a host curve of the common edge and pick up a midpoint (probe point)
        // to analyze the differential properties of the neighbor faces. We also
        // need a local tangent direction on the edge.
        double f, l;
        Handle(Geom_Curve) C = BRep_Tool::Curve(edge, f, l);
        //
        if (C.IsNull())
        {
            continue;
        }
        //
        const double midParam = (f + l) * 0.5;
        /*const double paramStep = (l - f) * 0.1;
        const double A_param = midParam - paramStep;
        const double B_param = midParam + paramStep;*/
        
        gp_Pnt P;

        //T can be calculate with C->D1
        gp_Vec T;
        C->D1(midParam, P, T);
        
        if (T.Magnitude() < RealEpsilon())
        {
            continue;
        }

        // Solve point inversion problem for P on A.
        gp_Pnt2d UV;
        FeatureCommon::GetMidUVByCurve2d(A, edge, UV);
        
        // Calculate differential properties
        BRepLProp_SLProps AProps(AS, UV.X(), UV.Y(), 2, 1e-5);
        //
        if (!AProps.IsCurvatureDefined())
        {
            continue;
        }

        gp_Dir A_minD, A_maxD;
        //
        AProps.CurvatureDirections(A_maxD, A_minD);
        
        const double A_minK = AProps.MinCurvature();
        const double A_maxK = AProps.MaxCurvature();

        // If A is a blend, then a2 is a blend curvature, and a1 is other
        // a1 correspond with the direction parallel with the tangent
        // curvature.
        double a1, a2;
        gp_Dir a1_dir, a2_dir;
        //
        if (Abs(A_minD.Dot(T)) > Abs(A_maxD.Dot(T)))
        {
            a1_dir = A_minD;
            a1 = A_minK;
            a2_dir = A_maxD;
            a2 = A_maxK;
        }
        else
        {
            a1_dir = A_maxD;
            a1 = A_maxK;
            a2_dir = A_minD;
            a2 = A_minK;
        }

        // tangent on edge must be collinear with the max or min curvature direction
        if (!T.IsParallel(a1_dir, FeatureCommon::Angular()))
        {
continue;
            if (!FeatureCommon::IsPlane(A))
            {
                gp_Pnt2d uvFeiwu;

                // 1->along, 2-> perpendicular
                double aAlong = 0, aPerpendicular = 0;
                FeatureCommon::EvaluateCurvature(A, edge, midParam, uvFeiwu, aAlong,
                                                 aPerpendicular);
                a1 = aAlong;
                a2 = aPerpendicular;
            }
        }

        // Self-curvature test: a2 is greater than a1 in magnitude (i.e., blend
        // curvature dominates).
        if ((Abs(a2) > Abs(a1)) && (Abs(a2 - a1) > Precision::Confusion()))
        {
            result.Add(edgeId);
        }
    }

    return result;
}
