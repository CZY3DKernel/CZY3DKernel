
#include "./SpringEdgesFinder.h"

#include <assert.h>

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>

#include "./AttributeAdjacentGraph.h"
#include "./FeatureCommon.h"

//-----------------------------------------------------------------------------

SpringEdgesFinder::SpringEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag)
    : m_aag(aag)
{
}

//-----------------------------------------------------------------------------

TColStd_PackedMapOfInteger SpringEdgesFinder::PerformForFace(
    const TopoDS_Face& A, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    bool& isCandidateBlend, double& candidateRadius)
{
    TColStd_PackedMapOfInteger result;
    // Initial guess is that this face looks like a blend.
    isCandidateBlend = false;

    // Get face of interest and its host surface.
    BRepAdaptor_Surface AS(A, false);

    // Get all smoothly connected neighbor faces.
    for (TColStd_PackedMapOfInteger::Iterator it(smoothEdgeIndices); it.More(); it.Next())
    {
        const int edgeId = it.Key();
        // Pick up just current common edge between the two faces.
        // This edge is a smooth edge.
        const TopoDS_Edge smoothEdge = m_aag->GetEdge(edgeId);
        // Get neighbor face and its host surface.
        const TopoDS_Face& B = m_aag->GetNeibourFace(A, smoothEdge);

        const int neibourId = m_aag->GetFaceId(B);

        if (B.IsNull())
        {
            continue;
        }

        BRepAdaptor_Surface BS(B, false);

        // Get a host curve of the common edge and pick up a midpoint (probe point)
        // to analyze the differential properties of the neighbor faces. We also
        // need a local tangent direction on the edge.
        double f, l;
        Handle(Geom_Curve) C = BRep_Tool::Curve(smoothEdge, f, l);
        //
        if (C.IsNull())
        {
            continue;
        }
        //
        const double midParam = (f + l) * 0.5;
        /* const double paramStep = (l - f) * 0.1;
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
        gp_Pnt2d goodUV;
        FeatureCommon::GetMidUVByCurve2d(A, smoothEdge, goodUV);

        #ifdef _DEBUG
        Handle(Geom_Surface) surface = BRep_Tool::Surface(A);
        gp_Pnt p1 = surface->Value(goodUV.X(), goodUV.Y());

        const double p1ToP = p1.Distance(P);

        if (p1ToP > 1e-2)
        {
            printf("disError:%f\n", p1ToP);
        }
        
        #endif

        /* gp_Pnt2d UV;
        {
            ShapeAnalysis_Surface SAS(AS.Surface().Surface());
            UV = SAS.ValueOfUV(P, 1.0e-6);

            gp_Pnt CP= AS.Surface().Surface()->Value(UV.X(), UV.Y());

            gp_Pnt goodCP = AS.Surface().Surface()->Value(goodUV.X(), goodUV.Y());

            gp_Pnt2d UVProject;
            if (FeatureCommon::ProjectPointToSurface(P, A, UVProject))
            {
                gp_Pnt project = AS.Surface().Surface()->Value(UVProject.X(), UVProject.Y());
                
                int stop = 0;
            }

            int stop = 0;
        }*/

        // Solve point inversion problem for P on B.
        gp_Pnt2d goodST;
        FeatureCommon::GetMidUVByCurve2d(B, smoothEdge, goodST);
#ifdef _DEBUG
        Handle(Geom_Surface) surfaceB = BRep_Tool::Surface(B);
        gp_Pnt pb = surfaceB->Value(goodST.X(), goodST.Y());

        const double pbToP = pb.Distance(P);
        if (pbToP > 1e-2)
        {
            printf("disError: %f\n", pbToP);
        }

#endif
        /*
        gp_Pnt2d ST;
        {
            BS 里面的surface可能被重新参数化
            ShapeAnalysis_Surface SAS(BS.Surface().Surface());
            ST = SAS.ValueOfUV(P, 1.0e-6);

            gp_Pnt CP = BS.Surface().Surface()->Value(ST.X(), ST.Y());
            gp_Pnt goodCP = BS.Surface().Surface()->Value(goodST.X(), goodST.Y());

            int stop = 0;
        }*/

        // Calculate differential properties
        BRepLProp_SLProps AProps(AS, goodUV.X(), goodUV.Y(), 2, 1e-5);
        BRepLProp_SLProps BProps(BS, goodST.X(), goodST.Y(), 2, 1e-5);
        //
        if (!AProps.IsCurvatureDefined())
        {
            continue;
        }
        if (!BProps.IsCurvatureDefined())
        {
            continue;
        }

        gp_Dir A_minD, A_maxD, B_minD, B_maxD;
        //
        AProps.CurvatureDirections(A_maxD, A_minD);
        BProps.CurvatureDirections(B_maxD, B_minD);
        //
        const double A_minK = AProps.MinCurvature();
        const double A_maxK = AProps.MaxCurvature();
        const double B_minK = BProps.MinCurvature();
        const double B_maxK = BProps.MaxCurvature();

        // If A is a blend, then a2 is a blend curvature, and a1 is other
        // a1 correspond with the direction parallel with the tangent
        // curvature.
        double a1 = 0, a2 = 0, b2 = 0, b1 = 0;
        gp_Dir a1_dir, a2_dir, b2_dir, b1_dir;
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

        #ifdef  _DEBUG
        double angle1 = T.Angle(a1_dir);
        double angle2 = T.Angle(a2_dir);
        #endif  //  _DEBUG

        if (!T.IsParallel(a1_dir, FeatureCommon::Angular()))
        {
            if (!FeatureCommon::IsPlane(A))
            {
                gp_Pnt2d uvFeiwu;

                // 1->along, 2-> perpendicular
                double aAlong = 0, aPerpendicular = 0;
                FeatureCommon::EvaluateCurvature(A, smoothEdge, midParam, uvFeiwu, aAlong,
                                                         aPerpendicular);
                a1 = aAlong;
                a2 = aPerpendicular;
            }
        }

        //
        if (Abs(B_minD.Dot(T)) > Abs(B_maxD.Dot(T)))
        {
            b2_dir = B_maxD;
            b2 = B_maxK;

            b1_dir = B_minD;
            b1 = B_minK;
        }
        else
        {
            b2_dir = B_minD;
            b2 = B_minK;

            b1_dir = B_maxD;
            b1 = B_maxK;
        }

        #ifdef _DEBUG
        double angle1B = T.Angle(b1_dir);
        double angle2B = T.Angle(b2_dir);
#endif  //  _DEBUG

        if (!T.IsParallel(b1_dir, FeatureCommon::Angular()))
        {
            if (!FeatureCommon::IsPlane(B))
            {
                gp_Pnt2d uvFeiwu;
                double bAlong = 0, bPerpendicular = 0;

                FeatureCommon::EvaluateCurvature(B, smoothEdge, midParam, uvFeiwu, bAlong,
                                                 bPerpendicular);
                b1 = bAlong;
                b2 = bPerpendicular;
            }
        }

#if defined COUT_DEBUG
        std::cout << "\t[Face #" << face_idx << "] a1 = " << a1 << std::endl;
        std::cout << "\t[Face #" << face_idx << "] a2 = " << a2 << std::endl;
        std::cout << "\t[Face #" << neighbor_idx << "] b2 = " << b2 << std::endl;
        std::cout << "---" << std::endl;
#endif

        // Self-curvature test: a2 is greater than a1 in magnitude (i.e., blend
        // curvature dominates).
        if ((Abs(a2) > Abs(a1)) && (Abs(a2 - a1) > Precision::Confusion()))
        {
            // Neighbor-curvature test. The coefficient here is used to give a
            // magnitude how much more curved the blend face is supposed to be.
            if ((Abs(a2) > 1.5 * Abs(b2)))
            {
                // A is more likely to be a blend since blends usually have higher
                // curvature than the support faces.
                if (!isCandidateBlend)
                {
                    isCandidateBlend = true;
                    candidateRadius = 1.0 / Abs(a2);
                }
                //
                result.Add(edgeId);
            }
        }
    }

    return result;
}
