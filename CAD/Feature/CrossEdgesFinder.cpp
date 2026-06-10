#include "./CrossEdgesFinder.h"

// OCCT includes
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <BRepLProp_SLProps.hxx>

#include "./AttributeAdjacentGraph.h"
#include "./FeatureCommon.h"

CrossEdgesFinder::CrossEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag) : m_aag(aag)
{
}

TColStd_PackedMapOfInteger CrossEdgesFinder::PerformForFace(
    const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdges,
    const TColStd_PackedMapOfInteger& springEdges)
{
    TColStd_PackedMapOfInteger result;

    // Get face of interest and its host surface.
    BRepAdaptor_Surface AS(face, false);
    // Get all smooth non-spring edges
    TColStd_PackedMapOfInteger remainingEdges;
    remainingEdges.Subtraction(smoothEdges, springEdges);

    // Test remaining edges on hitting the cross-edge rule
    for (TColStd_PackedMapOfInteger::Iterator it(remainingEdges); it.More(); it.Next())
    {
        const int edgeId = it.Key();
        const TopoDS_Edge& edge = m_aag->GetEdge(edgeId);

        double f = 0, l = 1;
        Handle(Geom_Curve) C = BRep_Tool::Curve(edge, f, l);
        //
        const double midParam = (f + l) * 0.5;

        gp_Pnt P;
        gp_Vec T;
        C->D1(midParam, P, T);

        // Solve point inversion problem for P on A.
        gp_Pnt2d UV;
        {
            ShapeAnalysis_Surface SAS(AS.Surface().Surface());
            UV = SAS.ValueOfUV(P, 1.0e-2);
        }

        // Calculate differential properties
        BRepLProp_SLProps AProps(AS, UV.X(), UV.Y(), 2, 1e-5);
        //
        if (!AProps.IsCurvatureDefined())
        {
            return false;
        }

        gp_Dir A_minD, A_maxD;
        //
        AProps.CurvatureDirections(A_maxD, A_minD);
        //
        const double A_minK = AProps.MinCurvature();
        const double A_maxK = AProps.MaxCurvature();
        ;

        // If A is a blend, then a2 is a blend curvature, and a1 is other
        // a1 correspond with the direction parallel with the tangent
        // curvature.
        double a1, a2, b2;
        gp_Dir a1_dir, a2_dir, b2_dir;
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
        if (T.IsParallel(a1_dir, FeatureCommon::Angular()))
        {
            result.Add(edgeId);
        }
    }

    return result;
}
