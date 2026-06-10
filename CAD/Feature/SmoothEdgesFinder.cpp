#include "./SmoothEdgesFinder.h"

#include <assert.h>

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>

#include "./AttributeAdjacentGraph.h"
#include "./CheckDihedralAngle.h"

/*
* @brief detect if commonEdge is smooth edge bwtween F and G
* @param face F
* @param commonEdge
* @param face G
* @return true if commonEdge is smooth; false otherwise
*/
bool DetectSmoothEdge(const TopoDS_Face& F, const TopoDS_Edge& commonEdge,
    const TopoDS_Face& G)
{
    const DihedralAngleType angleType =
        CheckDihedralAngle().AngleBetweenFaces(F, commonEdge, G, true, 0);

    switch (angleType)
    {
        case DihedralAngleType::Smooth:
        case DihedralAngleType::SmoothConvex:
        case DihedralAngleType::SmoothConcave:
        {
            return true;
        }

        case DihedralAngleType::Undefined:
        case DihedralAngleType::Concave:
        case DihedralAngleType::Convex:
        case DihedralAngleType::NonManifold:
        {
            return false;
        }

        default:
        {
            // invalid flow
            assert(false);
            break;
        }
    }
    return false;
}

TColStd_PackedMapOfInteger SmoothEdgesFinder::PerformForFace(const TopoDS_Shape& face) {
    
    TColStd_PackedMapOfInteger result;

    const TopTools_IndexedDataMapOfShapeListOfShape& edgesToFaces = m_aag->GetEdgesToFaces();

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(face, TopAbs_EDGE, edges);

    //iterate all edges from face
    for (TopTools_IndexedMapOfShape::Iterator it(edges); it.More(); it.Next())
    {
        const TopTools_ListOfShape& faces = edgesToFaces.FindFromKey(it.Value());
        const TopoDS_Edge& edge = TopoDS::Edge(it.Value());
        if (faces.Extent() == 1)
        {
            const TopoDS_Face& face = TopoDS::Face(faces.First());
            if (DetectSmoothEdge(face, edge, face))
            {
                const int edgeId = m_aag->GetEdgeId(edge);
                result.Add(edgeId);
            }
        }
        //smooth edge must between two faces
        else if (faces.Extent() == 2)
        {
            if (faces.First().IsNotEqual(faces.Last()))
            {
                if (DetectSmoothEdge(TopoDS::Face(faces.First()), edge, TopoDS::Face(faces.Last())))
                {
                    const int edgeId = m_aag->GetEdgeId(edge);
                    result.Add(edgeId);
                }
            }
            else//equal
            {
                const TopoDS_Face& face = TopoDS::Face(faces.First());
                if (DetectSmoothEdge(face, edge, face))
                {
                    const int edgeId = m_aag->GetEdgeId(edge);
                    result.Add(edgeId);
                }
            }
        }
    }

    return result;
}
