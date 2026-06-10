
#ifndef FR_AAG_h
#define FR_AAG_h

// FR includes
#include "./FR_AdjacencyMx.h"
#include "./FR_CheckDihedralAngle.h"
#include "./FR_FeatureAttr.h"
#include "./FR_FeatureFaces.h"
#include "./FR_Collections.h"
//#include "./FR_Utils.h"

// STL includes
#include <vector>

// OCCT includes
#include <NCollection_IncAllocator.hxx>
#include <NCollection_DataMap.hxx>
#include <Standard_GUID.hxx>
#include <NCollection_Map.hxx>
#include <NCollection_Vector.hxx>
//#include <CskStandard_OStream.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

#include "../COMMON/Extension_Export.hxx"

class FR_AAGRandomIterator;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! \brief Attributed Adjacency Graph for faces of a CAD model.
//!
//! Attributed Adjacency Graph (AAG) is a complementary structure for
//! B-Rep allowing for extension of geometry with semantics (features).
//! Some details on the structure can be found in the following paper:
//!
//! [Joshi S., Chang T.C. Graph-based heuristics for recognition of machined
//! features from a 3D solid model // Comput. Des. 1988. Vol. 20, N 2. P. 58-66.]
//!
//! AAG is a structure to store semantics of a CAD model just like TopoDS_Shape
//! is a structure to store its geometric form. Using AAG gives the following
//! advantages:
//!
//! - AAG gives a formal graph view on the CAD model. This enables using graph
//!   algorithms such as finding connectivity components, search, etc.
//!
//! - AAG caches serial indices of sub-shapes. Therefore, it is not necessary
//!   to use expensive TopExp utilities for a CAD model being unchanged as
//!   an algorithm runs.
//!
//! - AAG extends B-Rep with attributes to represent semantics of the model.
//!   Therefore, this structure is a storage for features recognized from
//!   geometry.
//!
//! - AAG is a convenience structure for traversing adjacent faces. The adjacency
//!   relation is explicit in AAG by definition.
//!
//! AAG is a main working structure for feature recognition. Any feature
//! recognition algorithm accepts or constructs AAG for operation.
class FR_AAG : public Standard_Transient
{
public:

  //! Enumerates options for caching maps of shapes.
  enum CachedMap
  {
    CachedMap_SubShapes     = 0x0001,
    CachedMap_Faces         = 0x0002,
    CachedMap_Edges         = 0x0004,
    CachedMap_Vertices      = 0x0008,
    CachedMap_EdgesFaces    = 0x0010,
    CachedMap_VerticesEdges = 0x0020,
    //
    CachedMap_Minimal = CachedMap_Faces,
    CachedMap_All = CachedMap_Faces      |
                    CachedMap_Edges      |
                    CachedMap_Vertices   |
                    CachedMap_SubShapes  |
                    CachedMap_EdgesFaces |
                    CachedMap_VerticesEdges
  };

  //---------------------------------------------------------------------------

  //! Format version for the serialized AAG.
  enum BinFormat
  {
    BinFormat_V1 = 0x001
  };

  //---------------------------------------------------------------------------

  //! Type definition for map of attributes.
  typedef NCollection_DataMap<Standard_GUID,
                              Handle(FR_FeatureAttr), Standard_GUID> t_attrMap;

  //---------------------------------------------------------------------------

  //! Arc between two nodes of AAG. The arc is the explicit representation
  //! for adjacency relation.
  struct t_arc
  {
    t_topoId F1; //!< First face.
    t_topoId F2; //!< Second face.

    //! ctor default.
    t_arc() : F1(0), F2(0) {}

    //! ctor with parameters.
    t_arc(const t_topoId _F1, const t_topoId _F2) : F1(_F1), F2(_F2) {}

    //! \return hash code for the arc.
    static int HashCode(const t_arc& arc, const int upper)
    {
      int key = arc.F1 + arc.F2;
      key += (key << 10);
      key ^= (key >> 6);
      key += (key << 3);
      key ^= (key >> 11);
      return (key & 0x7fffffff) % upper;
    }

    size_t operator()(const t_arc& arc) const
    {
        int key = arc.F1 + arc.F2;
        key += (key << 10);
        key ^= (key >> 6);
        key += (key << 3);
        key ^= (key >> 11);
        key = (key & 0x7fffffff);
        // 哈希计算逻辑
        //return std::hash<int>()(key);
        return key;
    }

    //! \return true if two links are equal.
    static int IsEqual(const t_arc& arc1, const t_arc& arc2)
    {
      return ( (arc1.F1 == arc2.F1) && (arc1.F2 == arc2.F2) ) ||
             ( (arc1.F2 == arc2.F1) && (arc1.F1 == arc2.F2) );
    }

     bool operator()(const t_arc& arc1, const t_arc& arc2) const noexcept
    {
         return ((arc1.F1 == arc2.F1) && (arc1.F2 == arc2.F2)) ||
                ((arc1.F2 == arc2.F1) && (arc1.F1 == arc2.F2));
    }
  };

  //---------------------------------------------------------------------------

  //! Collection of attributes.
  class t_attr_set
  {
  public:

    //! Convenience iterator for the set of attributes associated with
    //! node or arc in AAG.
    class Iterator
    {
    public:

      //! Ctor accepting the set of attributes to iterate.
      //! \param[in] attributes set of attributes to iterate.
      Iterator(const t_attr_set& attributes) : m_attrs(attributes)
      {
        m_it.Initialize( m_attrs.GetMap() );
      }

      //! \return true if there is something more to iterate starting from
      //!         current position, false -- otherwise.
      bool More() const
      {
        return m_it.More();
      }

      //! Moves iterator to the next position.
      void Next()
      {
        m_it.Next();
      }

      //! \return GUID of the currently iterated attribute.
      const Standard_GUID& GetGUID() const
      {
        return m_it.Key();
      }

      //! \return currently iterated attribute.
      const Handle(FR_FeatureAttr)& GetAttr() const
      {
        return m_it.Value();
      }

      //! \return non-const reference to the currently iterated attribute.
      Handle(FR_FeatureAttr)& ChangeAttr()
      {
        return m_it.ChangeValue();
      }

    protected:

      //! Attributes to iterate over.
      const t_attr_set& m_attrs;

      //! Internal iterator.
      t_attrMap::Iterator m_it;

    private:

      // To avoid C4512 "assignment operator could not be generated".
      Iterator& operator=(const Iterator&) { return *this; }
    };

  public:

    t_attr_set() {} //!< Default ctor.

    //! Constructor accepting a single attribute to populate the internal set.
    //! \param[in] single_attr single attribute to populate the set with.
    t_attr_set(const Handle(FR_FeatureAttr)& single_attr)
    {
      this->Add(single_attr);
    }

  public:

    //! Finds attribute by its global type ID.
    //! \param[in] id attribute's global ID.
    //! \return attribute instance.
    const Handle(FR_FeatureAttr)& operator()(const Standard_GUID& id) const
    {
      return m_set(id);
    }

    //! Adds the given attribute to the set.
    //! \param[in] attr attribute to add.
    void Add(const Handle(FR_FeatureAttr)& attr)
    {
      m_set.Bind(attr->GetGUID(), attr);
    }

    //! Checks whether the set of attributes contains an attribute of the
    //! given type.
    //! \param[in] id attribute's global ID.
    //! \return true in case of success, false -- otherwise.
    bool Contains(const Standard_GUID& id) const
    {
      return m_set.IsBound(id);
    }

    //! Returns pointer to Item by Key.
    //! Returns NULL is Key was not bound.
    //! \param[in] id attribute's global ID.
    //! \return pointer to attribute instance.
    const Handle(FR_FeatureAttr)* Seek(const Standard_GUID& id) const
    {
      return m_set.Seek(id);
    }

    //! \return internal collection.
    const t_attrMap& GetMap() const
    {
      return m_set;
    }

    //! \return internal collection.
    t_attrMap& ChangeMap()
    {
      return m_set;
    }

  private:

    //! Internal set storing attributes in association with their global IDs.
    t_attrMap m_set;

  };

  //---------------------------------------------------------------------------

  //! This attribute set collection is specialized to ensure the angle attribute
  //! is separated from the hash map. Because the angle attribute is the most
  //! "popular" type of attribute for graph edges, we do not want to search
  //! for its GUID in any sort of a hash-table, as this search tends to be slow
  //! in heavy calculations (hasher for GUIDs is not super fast).
  class t_arc_attr_set : public t_attr_set
  {
  public:

    t_arc_attr_set() {} //!< Default ctor.

    //! Constructor accepting a single attribute to populate the internal set or
    //! initialize a separate member field in the case when the passed attribute
    //! corresponds to the dihedral angle.
    //! \param[in] A the attribute to populate the set with.
    
      t_arc_attr_set(const Handle(FR_FeatureAttr) & A);

  public:

    Handle(FR_FeatureAttr) AngleAttr;
  };


  //---------------------------------------------------------------------------

  //! Arc attributes.
  typedef NCollection_DataMap<t_arc, t_arc_attr_set, t_arc> t_arc_attributes;

  //! Node attributes.
  typedef NCollection_DataMap<t_topoId, t_attr_set> t_node_attributes;

  //---------------------------------------------------------------------------

public:

  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_AAG, Standard_Transient)

public:

  /** @name Construction and destruction
   *  Functions to build and destroy AAG.
   */
  //@{

  //! Initializes AAG from the given master model and selected faces.
  //! \param[in] masterCAD        master model (full CAD).
  //! \param[in] selectedFaces    selected faces.
  //! \param[in] allowSmooth      indicates whether to allow "smooth" value for
  //!                             arc attribute. This value means that the
  //!                             joint between faces is at least G1, so in
  //!                             order to calculate dihedral angle, the neighborhood
  //!                             of transition has to be analyzed. The latter
  //!                             analysis introduces additional cost, so you
  //!                             may disable it if you are Ok to attribute G0
  //!                             joints only.
  //! \param[in] smoothAngularTol angular tolerance used for recognition
  //!                             of smooth dihedral angles. A smooth angle
  //!                             may appear to be imperfect by construction,
  //!                             but still smooth by the design intent. With
  //!                             this parameter you're able to control it.
  //! \param[in] cachedMaps       flag indicating which maps of sub-shapes to
  //!                             cache. Since building topo maps is costly,
  //!                             it is generally a good idea to reuse them
  //!                             as much as possible. Using this flag you
  //!                             can control which maps will be built.
  EXTENSION_EXPORT
    FR_AAG(const TopoDS_Shape&               masterCAD,
                const TopTools_IndexedMapOfShape& selectedFaces,
                const bool                        allowSmooth      = false,
                const double                      smoothAngularTol = 1.e-4,
                const int                         cachedMaps       = CachedMap_Minimal);

  //! Constructor accepting master CAD only.
  //! \param[in] masterCAD        master CAD.
  //! \param[in] allowSmooth      indicates whether "smooth" attribution for arcs
  //!                             is allowed (true) or not (false).
  //! \param[in] smoothAngularTol angular tolerance used for recognition
  //!                             of smooth dihedral angles. A smooth angle
  //!                             may appear to be imperfect by construction,
  //!                             but still smooth by the design intent. With
  //!                             this parameter you're able to control it.
  //! \param[in] cachedMaps       flag indicating which maps of sub-shapes to
  //!                             cache. Since building topo maps is costly,
  //!                             it is generally a good idea to reuse them
  //!                             as much as possible. Using this flag you
  //!                             can control which maps will be built.
  EXTENSION_EXPORT
    FR_AAG(const TopoDS_Shape& masterCAD,
                const bool          allowSmooth      = false,
                const double        smoothAngularTol = 1.e-4,
                const int           cachedMaps       = CachedMap_Minimal);

  //! Destructor.
  EXTENSION_EXPORT
    ~FR_AAG();

  //@}

public:

  /** @name Serialization
   *  Methods to serialize and deserialize AAG.
   */
  //@{

  //! Serializes the given AAG data structure to a binary file with the name
  //! passed as an argument.
  //! \param[in] aag       the AAG to serialize.
  //! \param[in] pFilename the filename of the target binary file to serialize to.
  //! \param[in] progress  the progress notifier.
  //! \return true in case of successful serialization, otherwise -- false.
  EXTENSION_EXPORT static bool
    Serialize(const Handle(FR_AAG)& aag,
              const char*                pFilename);

  //! Deserializes AAG data structure from a binary file with the name passed as
  //! an argument.
  //! \param[in]  pFilename the filename of the target binary file to deserialize from.
  //! \param[out] aag       the deserialized AAG.
  //! \param[in]  progress  the progress notifier.
  //! \return true in case of successful deserialization, otherwise -- false.
  EXTENSION_EXPORT static bool
    Deserialize(const char*          pFilename,
                Handle(FR_AAG)& aag);

  //@}

public:

  /** @name Derived graphs
   *  Methods to construct derived graphs and sub-graphs from AAG.
   */
  //@{

  //! \brief Constructs deep copy of AAG.
  //! \return copy of this AAG.
  EXTENSION_EXPORT Handle(FR_AAG)
    Copy() const;

  //! \brief Captures sub-graph being equal the original graph.
  //! \sa PopSubgraph() method to pop the created sub-graph from the stack.
  EXTENSION_EXPORT void
    PushSubgraph();

  //! \brief Captures sub-graph.
  //!
  //! Prepares a sub-graph containing the passed faces only. This sub-graph
  //! is pushed to the internal stack of sub-graphs eliminating all neighborhood
  //! relations which are out of interest in the current recognition setting.
  //!
  //! \param[in] faces2Keep indices of faces to keep in the model.
  //!
  //! \sa PopSubgraph() method to pop the created sub-graph from the stack.
  EXTENSION_EXPORT void
    PushSubgraph(const FR_Feature& faces2Keep);

  //! \brief Captures sub-graph.
  //!
  //! Prepares a sub-graph by removing the passed faces from the lower graph
  //! in the stack. This sub-graph is then pushed to the internal stack of
  //! sub-graphs eliminating all neighborhood relations which are out of
  //! interest in the current recognition setting.
  //!
  //! \param[in] faces2Exclude indices of faces to exclude from the model.
  //!
  //! \sa PopSubgraph() method to pop the created sub-graph from the stack.
  EXTENSION_EXPORT void
    PushSubgraphX(const FR_Feature& faces2Exclude);

  //! \brief Captures sub-graph.
  //!
  //! Prepares a sub-graph by removing the passed face from the lower graph
  //! in the stack. This sub-graph is then pushed to the internal stack of
  //! sub-graphs eliminating all neighborhood relations which are out of
  //! interest in the current recognition setting.
  //!
  //! \param[in] face2Exclude index of face to exclude from the model.
  //!
  //! \sa PopSubgraph() method to pop the created sub-graph from the stack.
  EXTENSION_EXPORT void
    PushSubgraphX(const t_topoId face2Exclude);

  //! \brief Pops the top sub-graph from the internal stack.
  //!
  //! Goes back to the parent graph from sub-graph. Use this method to
  //! recover the previous state of AAG.
  EXTENSION_EXPORT void
    PopSubgraph();

  //! \brief Pops all subgraphs except for the top one.
  //!
  //! Use this method to get rid of any subgraphs that might persist
  //! in the stack.
  EXTENSION_EXPORT void
    PopSubgraphs();

  //@}

public:

  //! By default, AAG stores only the adjacency relationships realized over
  //! the edges of a model. In some situations, it might be necessary to
  //! consider vertex-adjacent relationships as well (when two faces share
  //! a vertex while having no common edge). Call this method to add the
  //! corresponding graph links.
  //! \param[in] domain the faces whose vertices to add. If empty, all vertices
  //!                   of all faces are used.
  EXTENSION_EXPORT void
    AddVertexAdjacencyArcs(const FR_Feature& domain = FR_Feature());

  //! \return master CAD shape.
  EXTENSION_EXPORT const TopoDS_Shape&
    GetMasterShape() const;

  //! \deprecated Use GetMasterShape() instead.
  //! \return master CAD shape.
  //! \sa GetMasterShape()
  [[deprecated("Use the GetMasterShape() method instead.")]]
  EXTENSION_EXPORT const TopoDS_Shape&
    GetMasterCAD() const;

  //! \return number of graph nodes.
  EXTENSION_EXPORT int
    GetNumberOfNodes() const;

  //! Sets the collection of "selected" faces, i.e., the faces which are
  //! of particular interest to the client code. There is no logic behind
  //! this technique of marking some faces as selected.
  //! \param[in] selectedFaces faces to set as "selected".
  EXTENSION_EXPORT void
    SetSelectedFaces(const TopTools_IndexedMapOfShape& selectedFaces);

  //! Returns all selected faces.
  //! \return collection of 1-based indices of the selected faces.
  EXTENSION_EXPORT const FR_Feature&
    GetSelectedFaces() const;

  //! Returns true if the index is in range.
  //! \param[in] face_idx face index.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasFace(const t_topoId face_idx) const;

  //! Returns true if the passed face is in graph.
  //! \param[in] face face to check.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasFace(const TopoDS_Shape& face) const;

  //! Returns topological face by its internal index (e.g. coming from iterator).
  //! \param[in] face_idx face index.
  //! \return topological face.
  EXTENSION_EXPORT const TopoDS_Face&
    GetFace(const t_topoId face_idx) const;

  //! Returns face ID.
  //! \param[in] face face of interest.
  //! \return face ID.
  EXTENSION_EXPORT t_topoId
    GetFaceId(const TopoDS_Shape& face) const;

  //! Checks whether the given face has any neighbors recorded in the AAG.
  //! Normally it has, but in some abnormal situations no neighbors could
  //! be there.
  //! \param[in] face_idx face index.
  //! \return true in case if at least one neighbor presents, false -- otherwise.
  EXTENSION_EXPORT bool
    HasNeighbors(const t_topoId face_idx) const;

  //! Returns neighbors for the face having the given internal index.
  //! \param[in] face_idx  face index.
  //! \param[in] stackTail indicates whether to take neighbors from the
  //!                      tail of the adjacency matrix stack (the default
  //!                      is to use the stack head).
  //! \return indices of the neighbor faces.
  EXTENSION_EXPORT const FR_Feature&
    GetNeighbors(const t_topoId face_idx,
                 const bool     stackTail = false) const;

  //! Returns neighbors for all the faces from the passed collection.
  //! \param[in] fids      face indices to return neighbors for.
  //! \param[in] stackTail indicates whether to take neighbors from the
  //!                      tail of the adjacency matrix stack (the default
  //!                      is to use the stack head).
  //! \return indices of the neighbor faces.
  EXTENSION_EXPORT FR_Feature
    GetNeighbors(const FR_Feature& fids,
                 const bool             stackTail = false) const;

  //! Returns only those neighbor faces which share the given edge with the
  //! passed face of interest.
  //! \param[in] face_idx ID of the face of interest.
  //! \param[in] edge     common edge.
  //! \return indices of the neighbor faces sharing the given edge.
  EXTENSION_EXPORT FR_Feature
    GetNeighborsThru(const t_topoId face_idx, const TopoDS_Edge& edge);

  //! Returns neighbor faces for the given face of interest with additional
  //! filter on edges realizing the neighborhood.
  //! \param[in] face_idx index of the face of interest.
  //! \param[in] edge_ids indices of edges of interest.
  //! \return indices of the neighbor faces.
  EXTENSION_EXPORT FR_Feature
    GetNeighborsThru(const t_topoId         face_idx,
                     const FR_Feature& edge_ids);

  //! Returns only those neighbor faces which do not share the given edges with
  //! the passed face of interest.
  //! \param[in] face_idx ID of the face of interest.
  //! \param[in] xEdges   edge where neighborhood is restricted.
  //! \return indices of the neighbor faces not sharing the given edges.
  EXTENSION_EXPORT FR_Feature
    GetNeighborsThruX(const t_topoId         face_idx,
                      const FR_Feature& xEdges);

  //! Returns the vertex-adjacent neighbors for the face having the given
  //! internal index.
  //! \param[in] face_idx face index.
  //! \return indices of the neighbor faces.
  EXTENSION_EXPORT FR_Feature
    GetNeighborsThruVerts(const t_topoId face_idx);

  //! Returns full collection of neighbor faces.
  //! \return neighborhood data.
  EXTENSION_EXPORT const FR_AdjacencyMx&
    GetNeighborhood() const;

  //! Returns all faces of the master model.
  //! \return all faces.
  EXTENSION_EXPORT const TopTools_IndexedMapOfShape&
    GetMapOfFaces() const;

  //! Returns all faces of the master model having unique TShape pointers.
  //! If the map is empty, it is constructed.
  //! \return all faces.
  EXTENSION_EXPORT const FR_IndexedMapOfTShape&
    RequestTMapOfFaces();

  //! Returns all edges of the master model.
  //! If the map is empty, it is constructed.
  //! \return all edges.
  EXTENSION_EXPORT const TopTools_IndexedMapOfShape&
    RequestMapOfEdges();

  //! Returns all edges of the master model having unique TShape pointers.
  //! If the map is empty, it is constructed.
  //! \return all edges.
  EXTENSION_EXPORT const FR_IndexedMapOfTShape&
    RequestTMapOfEdges();

  //! Returns all vertices of the master model.
  //! If the map is empty, it is constructed.
  //! \return map of all vertices.
  EXTENSION_EXPORT const TopTools_IndexedMapOfShape&
    RequestMapOfVertices();

  //! Returns all vertices of the master model having unique TShape pointers.
  //! If the map is empty, it is constructed.
  //! \return all vertices.
  EXTENSION_EXPORT const FR_IndexedMapOfTShape&
    RequestTMapOfVertices();

  //! Returns all subshapes of the master model.
  //! If the map is empty, it is constructed.
  //! \return map of all sub-shapes.
  EXTENSION_EXPORT const TopTools_IndexedMapOfShape&
    RequestMapOfSubShapes();

  //! Returns all sub-shapes of the master model having unique TShape pointers.
  //! If the map is empty, it is constructed.
  //! \return all sub-shapes.
  EXTENSION_EXPORT const FR_IndexedMapOfTShape&
    RequestTMapOfSubShapes();

  //! \brief Returns map of indexed sub-shapes of the given type.
  //! If the map is empty, it is constructed.
  //!
  //! \param[in]  ssType sub-shape type (TopAbs_VERTEX, TopAbs_EDGE or TopAbs_FACE).
  //! \param[out] map    requested map of sub-shapes.
  EXTENSION_EXPORT void
    RequestMapOf(const TopAbs_ShapeEnum      ssType,
                 TopTools_IndexedMapOfShape& map);

  //! \brief Returns map of indexed sub-shapes with unique TShape pointers
  //!        of the given type.
  //!
  //! If the map is empty, it is constructed.
  //!
  //! \param[in]  ssType sub-shape type (TopAbs_VERTEX, TopAbs_EDGE or TopAbs_FACE).
  //! \param[out] map    requested map of sub-shapes.
  EXTENSION_EXPORT void
    RequestTMapOf(const TopAbs_ShapeEnum      ssType,
                  FR_IndexedMapOfTShape& map);

  //! Returns vertices and their owner edges.
  //! If the map is empty, it is constructed.
  //! \return map of vertices and their owner edges.
  EXTENSION_EXPORT const TopTools_IndexedDataMapOfShapeListOfShape&
    RequestMapOfVerticesEdges();

  //! Returns vertices with unique TShape pointers and their owner edges.
  //! If the map is empty, it is constructed.
  //! \return map of vertices and their owner edges.
  EXTENSION_EXPORT const FR_IndexedDataMapOfTShapeListOfShape&
    RequestTMapOfVerticesEdges();

  //! Returns vertices and their owner faces.
  //! If the map is empty, it is constructed.
  //! \return map of vertices and their owner faces.
  EXTENSION_EXPORT const TopTools_IndexedDataMapOfShapeListOfShape&
    RequestMapOfVerticesFaces();

  //! Returns vertices with unique TShape pointers and their owner faces.
  //! If the map is empty, it is constructed.
  //! \return map of vertices and their owner faces.
  EXTENSION_EXPORT const FR_IndexedDataMapOfTShapeListOfShape&
    RequestTMapOfVerticesFaces();

  //! Returns edges and their owner faces.
  //! If the map is empty, it is constructed.
  //! \return map of edges and their owner faces.
  EXTENSION_EXPORT const TopTools_IndexedDataMapOfShapeListOfShape&
    RequestMapOfEdgesFaces();

  //! Returns edges with unique TShape pointers and their owner faces.
  //! If the map is empty, it is constructed.
  //! \return map of edges and their owner faces.
  EXTENSION_EXPORT const FR_IndexedDataMapOfTShapeListOfShape&
    RequestTMapOfEdgesFaces();

  //! Finds a (sub-)shape among those cached in the AAG by the string
  //! representation of its address. Use this method to access the transient
  //! pointers to the shapes. These transient pointers can be then used
  //! to access the persistent pointers (i.e., serial indices or topological
  //! names).
  //!
  //! This method is non-const as it may build the internal map of shapes
  //! in case this map is not yet available.
  //!
  //! \param[in] addr string representation of the address to seek.
  //! \return shape (transient pointer).
  EXTENSION_EXPORT TopoDS_Shape
    FindSubShapeByAddr(const std::string& addr);

  //! Checks if the graph contains the passed arc (i.e., the referenced
  //! faces are adjacent).
  //! \param[in] arc graph arc to check.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasArc(const t_arc& arc) const;

  //! Checks whether the given arc has any attributes or not.
  //! \param[in] arc the arc to check.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasArcAttributes(const t_arc& arc) const;

  //! Checks whether the given arc has any attributes or not. If yes, an
  //! arc attribute is returned.
  //! \param[in]  arc     the arc to check.
  //! \param[out] attrAdj the returned adjacency attribute.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasArcAttribute(const t_arc&                 arc,
                    Handle(FR_FeatureAttr)& attrAdj) const;

  //! \return attributes associated with graph arcs.
  EXTENSION_EXPORT const t_arc_attributes&
    GetArcAttributes() const;

  //! Accessor for the default arc attribute, which holds adjacency
  //! properties between two faces, such as the dihedral angle,
  //! type (convex or concave), and edges that define face adjacency
  //! at the B-rep topological structure level.
  //!
  //! \param[in] arc the graph arc in question.
  //! \return attribute associated with the given arc.
  EXTENSION_EXPORT Handle(FR_FeatureAttr)
    GetArcAttribute(const t_arc& arc) const;

  //! Accessor for the arc attribute having the specified GUID.
  //! \param[in] arc     the graph arc in question.
  //! \param[in] attr_id the GUID of the attribute to access.
  //! \return attribute associated with the given arc.
  EXTENSION_EXPORT Handle(FR_FeatureAttr)
    GetArcAttribute(const t_arc&         arc,
                    const Standard_GUID& attr_id) const;

  //! Sets the given attribute for the passed arc in the AAG. If an attribute
  //! of this type already exists, this method does nothing and returns false.
  //! \param[in] arc  the graph arc of interest.
  //! \param[in] attr the attribute to set.
  //! \return true if the attribute has been set, false -- otherwise.
  EXTENSION_EXPORT bool
    SetArcAttribute(const t_arc&                       arc,
                    const Handle(FR_FeatureAttr)& attr);

  //! Checks whether the given node has any attributes or not.
  //! \param[in] node ID of the graph node to check.
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasNodeAttributes(const t_topoId node) const;

  //! Accessor for the entire collection of nodal attributes.
  //! \return attributes associated with all graph node.
  EXTENSION_EXPORT const t_node_attributes&
    GetNodeAttributes() const;

  //! Iterate over the nodal attributes for the passed face attempting
  //! to find an attribute of a certain type.
  //! \param[in] node the 1-based ID of the node (a face).
  //! \return the found attribute's handle or null if not found.
  template <typename TAttrType>
  Handle(TAttrType) FindNodeAttribute(const t_topoId node) const
  {
    Handle(TAttrType)              found;
    const FR_AAG::t_attr_set& attrSet = this->GetNodeAttributes(node);
    //
    for ( FR_AAG::t_attr_set::Iterator nit(attrSet); nit.More(); nit.Next() )
    {
      const Handle(FR_FeatureAttr)& attr = nit.GetAttr();
      //
      if ( attr->IsKind( STANDARD_TYPE(TAttrType) ) )
      {
        found = Handle(TAttrType)::DownCast(attr);
        break;
      }
    }
    return found;
  }

  //! Accessor for the collection of nodal attributes.
  //! \param[in] node ID of the graph node of interest.
  //! \return attributes associated with the given graph node.
  EXTENSION_EXPORT const t_attr_set&
    GetNodeAttributes(const t_topoId node) const;

  //! Returns attribute associated with the given graph node.
  //! \param[in] node    ID of the graph node of interest.
  //! \param[in] attr_id ID of the attribute to access.
  //! \return attribute associated with the given node.
  EXTENSION_EXPORT Handle(FR_FeatureAttr)
    GetNodeAttribute(const t_topoId       node,
                     const Standard_GUID& attr_id) const;

  //! Removes attribute with the passed GUID from the given graph node.
  //! \param[in] node    ID of the graph node of interest.
  //! \param[in] attr_id ID of the attribute to remove.
  //! \return true if the attribute was removed, false -- otherwise (e.g., if
  //!         such attribute does not exist).
  EXTENSION_EXPORT bool
    RemoveNodeAttribute(const t_topoId       node,
                        const Standard_GUID& attr_id);

  //! Removes all attributes assigned to nodes.
  EXTENSION_EXPORT void
    RemoveNodeAttributes();

  //! Cleans up all the attributes assigned with the AAG nodes except for
  //! the ones having the GUIDs from the passed `keep` collection.
  //! \param[in] keep the attribute types to keep alive on removal.
  EXTENSION_EXPORT void
    RemoveNodeAttributes(const NCollection_Map<Standard_GUID, Standard_GUID>& keep);

  //! Sets the entire collection of nodal attributes.
  //! \param[in] attrs attributes to set.
  EXTENSION_EXPORT void
    SetNodeAttributes(const t_node_attributes& attrs);

  //! Sets the given attribute for a node in AAG. If an attribute of this type
  //! is already there, this method does nothing and returns false.
  //! \param[in] node ID of the graph node of interest.
  //! \param[in] attr attribute to set.
  //! \return true if the attribute has been set, false -- otherwise.
  EXTENSION_EXPORT bool
    SetNodeAttribute(const t_topoId                     node,
                     const Handle(FR_FeatureAttr)& attr);

  //! Finds base faces, i.e., the faces having inner loops.
  //! \param[out] resultFaceIds IDs of the found faces (if any).
  //! \return true if anything has been found, false -- otherwise.
  EXTENSION_EXPORT bool
    FindBaseOnly(FR_Feature& resultFaceIds) const;

  //! Searches for the faces having ALL neighbors attributed with convex links.
  //! \param[out] resultFaceIds IDs of the found faces (if any).
  //! \return true if anything has been found, false -- otherwise.
  EXTENSION_EXPORT bool
    FindConvexOnly(FR_Feature& resultFaceIds) const;

  //! Searches for the faces having ALL neighbors attributed with convex links.
  //! \param[out] resultFaces found faces (if any).
  //! \return true if anything has been found, false -- otherwise.
  EXTENSION_EXPORT bool
    FindConvexOnly(TopTools_IndexedMapOfShape& resultFaces) const;

  //! Searches for the faces having ALL neighbors attributed with concave links.
  //! \param[out] resultFaceIds IDs of the found faces (if any).
  //! \return true if anything has been found, false -- otherwise.
  EXTENSION_EXPORT bool
    FindConcaveOnly(FR_Feature& resultFaceIds) const;

  //! Searches for the faces having ALL neighbors attributed with concave links.
  //! \param[out] resultFaces found faces (if any).
  //! \return true if anything has been found, false -- otherwise.
  EXTENSION_EXPORT bool
    FindConcaveOnly(TopTools_IndexedMapOfShape& resultFaces) const;

  //! Removes the passed faces with all corresponding arcs from AAG.
  //! \param[in] faces faces to remove.
  EXTENSION_EXPORT void
    Remove(const TopTools_IndexedMapOfShape& faces);

  //! Removes the passed faces with all corresponding arcs from AAG.
  //! \param[in] faceIndices indices of faces to remove.
  EXTENSION_EXPORT void
    Remove(const FR_Feature& faceIndices);

  //! Collapses the graph by eliminating the passed face ID while keeping
  //! the incident arcs in the adjacency matrix. The dihedral angle types
  //! will be propagated for the newly inserted transition links in the
  //! cases when dihedral angles are equal.
  //!
  //! CAUTION: the newly inserted dihedral angle attributes are not
  //!          automatically cleaned up from the graph in the
  //!          calls like `PopSubgraph()`, so make sure to ignore them.
  //! \param[in] fid the face index to collapse.
  EXTENSION_EXPORT void
    Collapse(const int fid);

  //! Collapses the graph by eliminating the passed faces while keeping
  //! the incident arcs in the adjacency matrix. The dihedral angle types
  //! will be propagated for the newly inserted transition links in the
  //! cases when dihedral angles are equal.
  //!
  //! CAUTION: the newly inserted dihedral angle attributes are not
  //!          automatically cleaned up from the graph in the
  //!          calls like `PopSubgraph()`, so make sure to ignore them.
  //! \param[in] faceIndices the indices of faces to collapse.
  EXTENSION_EXPORT void
    Collapse(const FR_Feature& faceIndices);

  //! Returns all face IDs to the passed outcome collection.
  //! \param[out] allFaces the output to populate.
  EXTENSION_EXPORT void
    GetAllFaces(FR_Feature& allFaces) const;

  //! Calculates number of the connected components.
  //! \return number of connected components in a graph.
  EXTENSION_EXPORT int
    GetConnectedComponentsNb();

  //! Calculates the number of the connected components in AAG without the
  //! given faces represented by their indexes.
  //! \param[in] excludedFaceIndices face indices to exclude from consideration.
  //! \return number of connected components in a graph.
  EXTENSION_EXPORT int
    GetConnectedComponentsNb(const FR_Feature& excludedFaceIndices);

  //! Calculates connected components for the given set of indexes.
  //! \param[in]  seeds seed nodes for the detection.
  //! \param[out] res   found connected components.
  EXTENSION_EXPORT void
    GetConnectedComponents(const FR_Feature&        seeds,
                           std::vector<FR_Feature>& res);

  //! Calculates connected components for the full graph.
  //! \param[out] res found connected components.
  EXTENSION_EXPORT void
    GetConnectedComponents(std::vector<FR_Feature>& res);

  //! \deprecated Kept for compatibility only (use std::vector version instead).
  //!
  //! Calculates connected components for the full graph.
  //! \param[out] res found connected components.
  [[deprecated("Use the overloaded version with std::vector instead.")]]
  EXTENSION_EXPORT void
    GetConnectedComponents(NCollection_Vector<FR_Feature>& res);
  
  //! Checks if the given vertex has any incident edges of the passed vexity.
  //! \param[in] V             the vertex to test.
  //! \param[in] type          the vexity to test.
  //! \param[in] edges2Exclude the edges to exclude from consideration (optional).
  //! \return true/false.
  EXTENSION_EXPORT bool
    HasIncidentEdgeOfVexity(const TopoDS_Vertex&              V,
                            const FR_FeatureAngleType    type,
                            const TopTools_IndexedMapOfShape& edges2Exclude = TopTools_IndexedMapOfShape());

  //! Clears cached maps.
  EXTENSION_EXPORT void
    ClearCache();

  /* Convenience methods */

  //! Returns the casted handle to the node attribute of the template type
  //! for the given face ID.
  //! \param[in] fid face ID in question.
  //! \return attribute.
  template<typename t_attr_type>
  Handle(t_attr_type) ATTR_NODE(const t_topoId fid) const
  {
    return Handle(t_attr_type)::DownCast( this->GetNodeAttribute( fid, t_attr_type::GUID() ) );
  }

  //! Returns the casted handle to the arc attribute of the template type
  //! for the given pair of face IDs.
  //! \param[in] arc face IDs defining the arc in question.
  //! \return attribute.
  template<typename t_attr_type>
  Handle(t_attr_type) ATTR_ARC(const t_arc& arc) const
  {
    return Handle(t_attr_type)::DownCast( this->GetArcAttribute(arc) );
  }

public:

  //! Dumps AAG structure to the passed output stream.
  //! \param[in, out] out target stream.
  EXTENSION_EXPORT void
    Dump(std::ostream& out) const;

  //! Dumps AAG structure as JSON.
  //! \param[in,out] out        target stream.
  //! \param[in]     whitespace num of spaces to prefix each row.
  EXTENSION_EXPORT void
    DumpJSON(std::ostream& out,
             const int         whitespace = 0) const;

protected:

  //! Initializes graph tool with master CAD and selected faces.
  //! \param[in] masterCAD        master model (full CAD).
  //! \param[in] selectedFaces    selected faces.
  //! \param[in] allowSmooth      indicates whether "smooth" attribution for
  //!                             arcs is allowed (true) or not (false).
  //! \param[in] smoothAngularTol angular tolerance used for recognition
  //!                             of smooth dihedral angles. A smooth angle
  //!                             may appear to be imperfect by construction,
  //!                             but still smooth by the design intent. With
  //!                             this parameter you're able to control it.
  //! \param[in] cachedMaps       flag indicating which maps of sub-shapes to
  //!                             cache. Since building topo maps is costly,
  //!                             it is generally a good idea to reuse them
  //!                             as much as possible. Using this flag you
  //!                             can control which maps will be built.
  EXTENSION_EXPORT void
    init(const TopoDS_Shape&               masterCAD,
         const TopTools_IndexedMapOfShape& selectedFaces,
         const bool                        allowSmooth,
         const double                      smoothAngularTol,
         const int                         cachedMaps);

  //! Fills graph with nodes for mate faces.
  //! \param[in] mateFaces faces to add (if not yet added).
  EXTENSION_EXPORT void
    addMates(const TopTools_ListOfShape& mateFaces);

  //! Dumps all graph nodes with their attributes to JSON.
  //! \param[in,out] out        target output stream.
  //! \param[in]     whitespace num of spaces to prefix each row.
  EXTENSION_EXPORT void
    dumpNodesJSON(std::ostream& out,
                  const int         whitespace = 0) const;

  //! Dumps single graph node with its attributes to JSON.
  //! \param[in]     node       ID of the graph node to dump.
  //! \param[in]     isFirst    indicates whether the currently dumped node is
  //!                           the first one to correctly put commas.
  //! \param[in,out] out        target output stream.
  //! \param[in]     whitespace num of spaces to prefix each row.
  EXTENSION_EXPORT void
    dumpNodeJSON(const t_topoId    node,
                 const bool        isFirst,
                 std::ostream& out,
                 const int         whitespace = 0) const;

  //! Dumps all graph arcs with their attributes to JSON.
  //! \param[in,out] out        target output stream.
  //! \param[in]     whitespace num of spaces to prefix each row.
  EXTENSION_EXPORT void
    dumpArcsJSON(std::ostream& out,
                 const int         whitespace = 0) const;

  //! Dumps the given graph arc with its attributes to JSON.
  //! \param[in]     arc        graph arc in question.
  //! \param[in]     isFirst    indicates whether the currently dumped arc is
  //!                           the first one to correctly put commas.
  //! \param[in,out] out        target output stream.
  //! \param[in]     whitespace num of spaces to prefix each row.
  EXTENSION_EXPORT void
    dumpArcJSON(const t_arc&      arc,
                const bool        isFirst,
                std::ostream& out,
                const int         whitespace = 0) const;

protected:

  FR_AAG() : m_bAllowSmooth(false), m_fSmoothAngularTol(0.0) {} //!< Default ctor.

protected:

  //! Master CAD model.
  TopoDS_Shape m_master;

  //! Selected faces. Selection is performed externally using any criterion
  //! which we do not care about here. One typical scenario is to select
  //! those faces corresponding to some feature in the model.
  FR_Feature m_selected;

  //! All sub-shapes.
  TopTools_IndexedMapOfShape m_subShapes;

  //! All sub-shapes with distinct TShape pointers.
  FR_IndexedMapOfTShape m_tSubShapes;

  //! All faces of the master model.
  TopTools_IndexedMapOfShape m_faces;

  //! All faces of the master model with distinct TShape pointers.
  FR_IndexedMapOfTShape m_tFaces;

  //! All edges of the master model.
  TopTools_IndexedMapOfShape m_edges;

  //! All edges of the master model with distinct TShape pointers.
  FR_IndexedMapOfTShape m_tEdges;

  //! All vertices of the master model.
  TopTools_IndexedMapOfShape m_vertices;

  //! All vertices of the master model with distinct TShape pointers.
  FR_IndexedMapOfTShape m_tVertices;

  //! Map of vertices versus edges.
  TopTools_IndexedDataMapOfShapeListOfShape m_verticesEdges;

  //! Map of vertices with distinct TShape pointers versus edges.
  FR_IndexedDataMapOfTShapeListOfShape m_tVerticesEdges;

  //! Map of vertices versus faces.
  TopTools_IndexedDataMapOfShapeListOfShape m_verticesFaces;

  //! Map of vertices with distinct TShape pointers versus faces.
  FR_IndexedDataMapOfTShapeListOfShape m_tVerticesFaces;

  //! Map of edges versus faces.
  TopTools_IndexedDataMapOfShapeListOfShape m_edgesFaces;

  //! Map of edges with distinct TShape pointers versus faces.
  FR_IndexedDataMapOfTShapeListOfShape m_tEdgesFaces;

  //! The data maps stored in this stack represent adjacency matrices. The
  //! stack is used to keep sub-graphs.
  std::vector<FR_AdjacencyMx> m_neighborsStack;

  //! Stores attributes associated with each arc.
  t_arc_attributes m_arcAttributes;

  //! Stores attributes associated with nodes.
  t_node_attributes m_nodeAttributes;

  //! Indicates whether to allow smooth transitions or not.
  bool m_bAllowSmooth;

  //! Angular tolerance to use for attribution of "smooth" dihedral edges.
  double m_fSmoothAngularTol;

  //! Experimental allocator (does it make any sense?).
  Handle(NCollection_IncAllocator) m_alloc;

  //! Dihedral angle checker that is declared as a member field in order
  //! to cache faces and their edges.
  Handle(FR_CheckDihedralAngle) m_checkDihAngle;

};

#endif
