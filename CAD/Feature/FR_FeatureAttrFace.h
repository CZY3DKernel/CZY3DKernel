
#ifndef FR_FeatureAttrFace_h
#define FR_FeatureAttrFace_h

// FR includes
#include "./FR_FeatureAttr.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Attribute for feature faces. This attribute brings feature ID which
//! allows marking faces as belonging to the same group.
class FR_FeatureAttrFace : public FR_FeatureAttr
{
  friend class FR_AAG;

  DEFINE_STANDARD_RTTI_INLINE(FR_FeatureAttrFace, FR_FeatureAttr)

public:

  //! Creates attribute.
  //! \param[in] featureId feature ID.
  FR_FeatureAttrFace(const int featureId)
  : FR_FeatureAttr (),
    m_iFaceId           (0),
    m_iFeatureId        (featureId)
  {}

public:

  //! \return feature index.
  int GetFeatureId() const { return m_iFeatureId; }

  //! Sets face ID.
  //! \param faceId [in] 1-based face ID.
  void SetFaceId(const int faceId)
  {
    m_iFaceId = faceId;
  }

  //! \return face ID.
  int GetFaceId() const
  {
    return m_iFaceId;
  }

protected:

  int m_iFaceId;    //!< 1-based face ID.
  int m_iFeatureId; //!< 1-based feature ID.

};

#endif
