#pragma once

#include <TColStd_PackedMapOfInteger.hxx>
#include <TopoDS_Face.hxx>

class AttributeAdjacentGraph;

class SpringEdgesFinder
{
public:
    SpringEdgesFinder(const std::shared_ptr<AttributeAdjacentGraph>& aag);

public:

  //! Performs detection of spring edges for the given face.
  //! \param[in]  face         the face of interest.
  //! \param[in]  smoothEdgeIndices  indices of the smooth edges.
  //! \param[out] isCandidateBlend true if the given face looks like a blend,
  //!                              false -- otherwise.
  //! \param[out] candidateRadius  if the face of interest was recognized as a
  //!                              blend candidate, then this output parameter
  //!                              will be used to give back the candidate
  //!                              radius.
  //! \return true in case of success, false -- otherwise.
 TColStd_PackedMapOfInteger PerformForFace(const TopoDS_Face& face,
                                              const TColStd_PackedMapOfInteger& smoothEdgeIndices,
                                              bool& isCandidateBlend, double& candidateRadius);

protected:
  std::shared_ptr<AttributeAdjacentGraph> m_aag;    //!< AAG.
};
