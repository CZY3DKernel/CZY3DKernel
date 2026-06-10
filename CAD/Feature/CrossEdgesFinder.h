#pragma once

#include <memory>

#include <TopoDS_Face.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

class AttributeAdjacentGraph;

class CrossEdgesFinder
{
public:

    CrossEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag);

public:

  //! Performs detection of cross edges for the given face.
  //! \param[in] face        the face of interest.
  //! \param[in] smoothEdges     smooth edges indices.
  //! \param[in] springEdges     spring edges indices.
  //! \return true in case of success, false -- otherwise.
 TColStd_PackedMapOfInteger PerformForFace(
     const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgesIndices,
     const TColStd_PackedMapOfInteger& springEdgesIndices);

protected:
  std::shared_ptr<AttributeAdjacentGraph> m_aag;    //!< AAG.
};
