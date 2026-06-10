#include "./AttributeAdjacentGraph.h"

#include <assert.h>

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

AttributeAdjacentGraph::AttributeAdjacentGraph(const TopoDS_Shape& shape) 
:m_shape(shape)
{
    TopExp::MapShapes(m_shape, TopAbs_FACE, m_faces);

    TopExp::MapShapes(m_shape, TopAbs_EDGE, m_edges);

    TopExp::MapShapes(m_shape, TopAbs_VERTEX, m_vertices);
    
    TopExp::MapShapesAndAncestors(m_shape, TopAbs_VERTEX, TopAbs_EDGE, m_verticesToEdges);

    TopExp::MapShapesAndAncestors(m_shape, TopAbs_VERTEX, TopAbs_FACE, m_verticesToFaces);

    TopExp::MapShapesAndAncestors(m_shape, TopAbs_EDGE, TopAbs_FACE, m_edgesToFaces);

}

const TopTools_ListOfShape& AttributeAdjacentGraph::GetEdgesByVertex(const TopoDS_Vertex& v) const
{
    if (!m_verticesToEdges.Contains(v))
    {
        static TopTools_ListOfShape s_empty;
        return s_empty;
    }

    return m_verticesToEdges.FindFromKey(v);
}

const TopTools_ListOfShape& AttributeAdjacentGraph::GetFacesByEdge(const TopoDS_Edge& e) const
{
    if (!m_edgesToFaces.Contains(e))
    {
        static TopTools_ListOfShape s_empty;
        return s_empty;
    }

    return m_edgesToFaces.FindFromKey(e);
}

const TopTools_IndexedMapOfShape& AttributeAdjacentGraph::GetEdges() const
{
    return m_edges;
}

const TopTools_IndexedDataMapOfShapeListOfShape& AttributeAdjacentGraph::GetEdgesToFaces() const
{
    return m_edgesToFaces;
}

int AttributeAdjacentGraph::GetEdgeId(const TopoDS_Edge& edge) const
{
    return m_edges.FindIndex(edge);
}

const TopTools_IndexedMapOfShape& AttributeAdjacentGraph::GetFaces() const
{
    return m_faces;
}

const TopoDS_Edge AttributeAdjacentGraph::GetEdge(const int edgeId) const
{
    return TopoDS::Edge(m_edges.FindKey(edgeId));
}

const TopoDS_Face AttributeAdjacentGraph::GetFace(const int faceId) const
{
    if (faceId < 1 || faceId > m_faces.Extent())
    {
        assert(false);
        return TopoDS_Face();
    }

    return TopoDS::Face(m_faces.FindKey(faceId));
}

int AttributeAdjacentGraph::GetFaceId(const TopoDS_Face& face) const
{
    return m_faces.FindIndex(face);
}

/*
 * @brief get edge ids on face
 * @param face
 * @return edge ids
 */
TColStd_PackedMapOfInteger AttributeAdjacentGraph::GetEdgeIds(const TopoDS_Face& face) const
{
    TColStd_PackedMapOfInteger result;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        result.Add(GetEdgeId(edge));
    }

    return result;
}


TopoDS_Face AttributeAdjacentGraph::GetNeibourFace(const TopoDS_Face& face,
                                                    const TopoDS_Edge& edge) const
{
    const TopTools_ListOfShape& faces = m_edgesToFaces.FindFromKey(edge);

    // smooth edge must between two faces
    if (faces.Extent() != 2)
    {
        return {};
    }

    if (faces.First().IsEqual(face))
    {
        return TopoDS::Face(faces.Last());
    }
    else if (faces.Last().IsEqual(face))
    {
        return TopoDS::Face(faces.First());
    }
    else
    {
        assert(false);
    }

    return {};
}

void AttributeAdjacentGraph::AddBlendInfo(const TopoDS_Face& face,
                                        const std::shared_ptr<BlendProperty>& blendInfo)
{
    m_blendInfos.Add(face, blendInfo);
}

void AttributeAdjacentGraph::AddChamferFeature(const TopoDS_Face& face,
                                        const std::shared_ptr<ChamferFeature>& feature)
{
    m_chamferFeatures.Add(face, feature);
}


std::shared_ptr<BlendProperty> AttributeAdjacentGraph::GetBlendProperty(const TopoDS_Face& face) const
{
    if (m_blendInfos.Contains(face))
    {
        return m_blendInfos.FindFromKey(face);
    }

    return nullptr;
}

std::shared_ptr<ChamferFeature> AttributeAdjacentGraph::GetChamferFeature(const TopoDS_Face& face) const
{
    if (m_chamferFeatures.Contains(face))
    {
        return m_chamferFeatures.FindFromKey(face);
    }

    return nullptr;
}

bool AttributeAdjacentGraph::HasBlendProperty(const TopoDS_Face& face) const
{
    if (face.IsNull())
    {
        return false;
    }
    return m_blendInfos.Contains(face);
}

bool AttributeAdjacentGraph::HasChamferFeature(const TopoDS_Face& face) const
{
    if (face.IsNull())
    {
        return false;
    }

    return m_chamferFeatures.Contains(face);
}

const CskIndexedDataMapOfShapeBlendProperty& AttributeAdjacentGraph::GetBlendProperties() const
{
    return m_blendInfos;
}

/*
 * @brief get neibour faces of current face
 * @param face, current face
 * @param return neibour faces
 */
TopTools_IndexedMapOfShape AttributeAdjacentGraph::GetNeibourFaces(
    const TopoDS_Face& face) const
{
    TopTools_IndexedMapOfShape neighbours;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
    {
        neighbours.Add(GetNeibourFace(face, TopoDS::Edge(exp.Current())));
    }

    return neighbours;
}
