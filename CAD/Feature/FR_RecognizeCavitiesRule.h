#ifndef FR_RecognizeCavitiesRule_h
#define FR_RecognizeCavitiesRule_h

// FR includes
#include "./FR_RecognitionRule.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Recognition rule for cavities.
class FR_RecognizeCavitiesRule : public FR_RecognitionRule
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeCavitiesRule, FR_RecognitionRule)

public:

  //! Ctor accepting the AAG iterator.
  //! \param[in] it       the AAG iterator.
  //! \param[in] maxSize  the max allowed feature size.
  //! \param[in] progress the progress entry.
  //! \param[in] plotter  the imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeCavitiesRule(const Handle(FR_AAGIterator)& it,
                                  const double                       maxSize);

protected:

  //! Recognizes feature starting from the current position of AAG iterator.
  //! \param[out] featureFaces   the outcome feature faces.
  //! \param[out] featureIndices the indices of the outcome feature faces.
  //! \return true/false.
  EXTENSION_EXPORT virtual bool
    recognize(TopTools_IndexedMapOfShape& featureFaces,
              FR_Feature&            featureIndices) override;

protected:

  //! Initializes the internal data structures.
  EXTENSION_EXPORT void
    init();

  //! Completes feature growing from the given base face and starting from the
  //! given start face.
  //! \param[in]  startId        index of the face to start off.
  //! \param[in]  start_face_idx indexes of the starting faces.
  //! \param[out] featureFaces   feature faces.
  //! \param[out] featureIndices indices of the feature faces.
  //! \param[out] isOk           indicates whether the feature is Ok to propagate on.
  EXTENSION_EXPORT void
    propagate(const int                   startId,
              const FR_Feature&      start_face_idx,
              TopTools_IndexedMapOfShape& featureFaces,
              FR_Feature&            featureIndices,
              bool&                       isOk);

  //! Checks that given wire has convex connection with adjacent faces.
  //! \return true if adjacency is convex.
  EXTENSION_EXPORT bool
    isConvex(const int          seed_face_id,
             const TopoDS_Wire& wire);

  //! Checks that the feature candidate faces do not span the entire shape.
  //! \return true/false.
  EXTENSION_EXPORT bool
    isNotEntireShape(const TopTools_IndexedMapOfShape& newFeatureFaces,
                     const FR_Feature&            newFeatureIndices);

  //! Checks whether the passed feature candidate fits into the size criterion.
  //! \return true/false.
  EXTENSION_EXPORT bool
    isSizeOk(const TopTools_IndexedMapOfShape& newFeatureFaces);

protected:

  //! Max allowed feature size.
  double m_fMaxSize = 0;

  //! Data map to improve access to the outer wire.
  NCollection_DataMap<TopoDS_Face, TopoDS_Wire> m_mapFaceOuterWire;

};

#endif
