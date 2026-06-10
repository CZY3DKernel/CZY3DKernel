#pragma once

// STL includes
#include <vector>

// OCCT includes
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TColStd_PackedMapOfInteger.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include "./BlendProperty.h"
#include "./ChamferInfo.h"

EXTENSION_EXPORT class AttributeAdjacentGraph
{
public:
    AttributeAdjacentGraph(const TopoDS_Shape& shape);

    int GetEdgeId(const TopoDS_Edge& edge) const;

    /*
    * @brief get edge ids on face
    * @param face
    * @return edge ids
    */
    TColStd_PackedMapOfInteger GetEdgeIds(const TopoDS_Face& face) const;
    int GetFaceId(const TopoDS_Face& face) const;

    const TopTools_IndexedMapOfShape& GetEdges() const;
    const TopTools_IndexedMapOfShape& GetFaces() const;
    const TopTools_IndexedDataMapOfShapeListOfShape& GetEdgesToFaces() const;
    const TopoDS_Edge GetEdge(const int edgeId) const;
    const TopoDS_Face GetFace(const int faceId) const;

    const TopTools_ListOfShape& GetEdgesByVertex(const TopoDS_Vertex& v) const;

    const TopTools_ListOfShape& GetFacesByEdge(const TopoDS_Edge& e) const;

    /*
    * @brief get neibour face of current face through connected edge
    * @param face, current face
    * @param edge, connected edge
    * @param return neibour face
    */
    TopoDS_Face GetNeibourFace(const TopoDS_Face& face, const TopoDS_Edge& edge) const;

    std::shared_ptr<BlendProperty> GetBlendProperty(const TopoDS_Face& face) const;

    std::shared_ptr<ChamferFeature> GetChamferFeature(const TopoDS_Face& face) const;

    void AddBlendInfo(const TopoDS_Face& face, const std::shared_ptr<BlendProperty>& blendInfo);

    void AddChamferFeature(const TopoDS_Face& face,
                           const std::shared_ptr<ChamferFeature>& feature);

    bool HasChamferFeature(const TopoDS_Face& face) const;

    /*
     * @brief get neibour faces of current face
     * @param face, current face
     * @param return neibour faces
     */
    TopTools_IndexedMapOfShape GetNeibourFaces(const TopoDS_Face& face) const;

    /*
     * @brief check some face has blend info
     * @param face
     * @param return true if face has blend info; false otherwise
     */
    bool HasBlendProperty(const TopoDS_Face& face) const;

    const CskIndexedDataMapOfShapeBlendProperty& GetBlendProperties() const;

protected:

  //! Master CAD model.
  TopoDS_Shape m_shape;

  //! All faces of the master model.
  TopTools_IndexedMapOfShape m_faces;
  TopTools_IndexedMapOfShape m_vertices;
  TopTools_IndexedMapOfShape m_edges;

  //! Map of vertices versus edges.
  TopTools_IndexedDataMapOfShapeListOfShape m_verticesToEdges;
  
  //! Map of vertices versus faces.
  TopTools_IndexedDataMapOfShapeListOfShape m_verticesToFaces;

  //! Map of edges versus faces.
  TopTools_IndexedDataMapOfShapeListOfShape m_edgesToFaces;

  CskIndexedDataMapOfShapeBlendProperty m_blendInfos;

  CskIndexedDataMapOfShapeChamferFeature m_chamferFeatures;
};
