#pragma once

#include <TColStd_PackedMapOfInteger.hxx>
#include <TopoDS_Face.hxx>

class AttributeAdjacentGraph;

class CliffEdgesFinder
{
public:
    CliffEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag);

public:

  //! Performs detection of cliff edges for the given face.
  //! \param[in]  face         the face of interest.
  //! \param[in]  smoothEdgeIndices  indices of the smooth edges.
  //! \return cliff edge indices
 TColStd_PackedMapOfInteger PerformForFace(const TopoDS_Face& face,
                                              const TColStd_PackedMapOfInteger& smoothEdgeIndices);

protected:
  std::shared_ptr<AttributeAdjacentGraph> m_aag;    //!< AAG.
};
