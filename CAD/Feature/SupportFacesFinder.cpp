#include "./SupportFacesFinder.h"

#include <assert.h>

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include "./AttributeAdjacentGraph.h"
#include "./CheckDihedralAngle.h"

TColStd_PackedMapOfInteger SupportFacesFinder::PerformForFace(const TopoDS_Face& face, const TColStd_PackedMapOfInteger& springEdgeIndices)
{
    
    TColStd_PackedMapOfInteger result;

    const TopTools_IndexedDataMapOfShapeListOfShape& edgesToFaces = m_aag->GetEdgesToFaces();

    //iterate all edges from face
    for (TColStd_PackedMapOfInteger::Iterator it(springEdgeIndices);it.More(); it.Next())
    {
        const TopoDS_Edge edge = m_aag->GetEdge(it.Key());
        const TopTools_ListOfShape& faces = edgesToFaces.FindFromKey(edge);

        //smooth edge must between two faces
        if (faces.Extent() != 2)
        {
            continue;
        }
        
        if (faces.First().IsEqual(face))
        {
            result.Add(m_aag->GetFaceId(TopoDS::Face(faces.Last())));
        }
        else if (faces.Last().IsEqual(face))
        {
            result.Add(m_aag->GetFaceId(TopoDS::Face(faces.First())));
        }
        else
        {
            printf("current face = %d\n", m_aag->GetFaceId(face));
            printf("edgeId = %d\n", it.Key());

            printf("face set = {");
            for (TopTools_ListOfShape::Iterator fit(faces); fit.More(); fit.Next())
            {
                printf("%d, ", m_aag->GetFaceId(TopoDS::Face(fit.Value())));
            }
            puts("}");

            assert(false);
        }
    }

    return result;
}
