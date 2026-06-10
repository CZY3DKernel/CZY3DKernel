#ifndef FR_RecognizeCavities_h
#define FR_RecognizeCavities_h

// FR includes
#include "./FR_Recognizer.h"

// Standard includes
#include <unordered_map>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Recognizes all negative features in a CAD part. Their shape
//! does not matter. A feature to recognize is expected to terminate
//! at inner contours of its base faces.
class FR_RecognizeCavities : public FR_Recognizer
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeCavities, FR_Recognizer)

public:

  //! The recognized cavities with their base (capping) faces.
  typedef std::vector< std::pair<FR_Feature, FR_Feature> > t_cavities;

public:

  //! Ctor with a shape.
  //! \param[in] shape    the shape to recognize.
  //! \param[in] progress the progress notifier.
  //! \param[in] plotter  the imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeCavities(const TopoDS_Shape&  shape);

  //! Ctor with AAG.
  //! \param[in] aag      the AAG instance for the shape to recognize.
  //! \param[in] progress the progress notifier.
  //! \param[in] plotter  the imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeCavities(const Handle(FR_AAG)& aag);

public:

  //! Sets the max allowed feature size.
  //! \param[in] maxSize the feature size to set.
  EXTENSION_EXPORT void
    SetMaxSize(const double maxSize);

  //! \return the max allowed feature size.
  EXTENSION_EXPORT double
    GetMaxSize() const;

  //! \return cavity features distributed by their base faces.
  EXTENSION_EXPORT const t_cavities&
    GetCavities() const;

public:

  //! Performs recognition.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    Perform();

protected:

  //! Returns all seed faces for the recognizer. A seed face should have
  //! only convex dihedral angles at its inner contours.
  //! \param[out] seeds the found seed faces.
  EXTENSION_EXPORT void
    findSeeds(FR_Feature& seeds);

  //! Collects cavity features and their base faces.
  EXTENSION_EXPORT void
    collectCavities();

protected:

  //! Max allowed feature size.
  double m_fMaxSize = 0;

  //! Recognized cavities and their base faces.
  t_cavities m_cavities;

};

#endif
