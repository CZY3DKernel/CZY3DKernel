
#ifndef FR_FeatureAttrAdjacency_h
#define FR_FeatureAttrAdjacency_h

// FR includes
#include "./FR_FeatureAttr.h"

// OCCT includes
#include <TColStd_PackedMapOfInteger.hxx>
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Attribute to store actual realization of adjacency between faces.
class FR_FeatureAttrAdjacency : public FR_FeatureAttr
{
  DEFINE_STANDARD_RTTI_INLINE(FR_FeatureAttrAdjacency, FR_FeatureAttr)

public:

  //! Creates adjacency attribute with empty collection of edge indices.
  //! \sa SetEdgeIndices() method to populate the attribute.
  FR_FeatureAttrAdjacency() : FR_FeatureAttr() {}

  //! Creates adjacency attribute and initializes it with the indices of
  //! edges connecting the adjacent faces.
  //! \param[in] edgeIndices unordered collection of edge indices to set.
  FR_FeatureAttrAdjacency(const TColStd_PackedMapOfInteger& edgeIndices)
  : FR_FeatureAttr (),
    m_edgeIndices       (edgeIndices)
  {}

public:

  //! \return static GUID associated with this type of attribute.
  static const Standard_GUID& GUID()
  {
    static Standard_GUID guid("2C9F5B79-4D0A-4824-8878-A1E8CF404E31");
    return guid;
  }

  //! \return GUID associated with this type of attribute.
  virtual const Standard_GUID& GetGUID() const
  {
    return GUID();
  }

  //! \return human-readable name of the attribute.
  virtual const char* GetName() const override
  {
    return "Adjacency";
  }

public:

  //! \return unordered collection of edge indices.
  const TColStd_PackedMapOfInteger& GetEdgeIndices() const
  {
    return m_edgeIndices;
  }

  //! Sets collection of edge indices.
  //! \param[in] edgeIndices collection of indices of common edges.
  void SetEdgeIndices(const TColStd_PackedMapOfInteger& edgeIndices)
  {
    m_edgeIndices = edgeIndices;
  }

  //! Gathers edges by the stored indices.
  //! \param[out] edges output collection of transient edges as queried
  //!                   from AAG.
  void GetEdges(TopTools_IndexedMapOfShape& edges)
  {
    const TopTools_IndexedMapOfShape& edgeMap = this->getAAG()->RequestMapOfEdges();

    for ( TColStd_MapIteratorOfPackedMapOfInteger eit(m_edgeIndices); eit.More(); eit.Next() )
    {
      const int eid = eit.Key();
      edges.Add( edgeMap(eid) );
    }
  }

protected:

  TColStd_PackedMapOfInteger m_edgeIndices; //!< Indices of common edges.

};

#endif
