#ifndef FR_RecognizeDrillHoles_h
#define FR_RecognizeDrillHoles_h

// FR includes
#include "./FR_FeatureFaces.h"
#include "./FR_Recognizer.h"

// OCCT includes
#include <Precision.hxx>
#include <Standard_Type.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Utility to recognize drilled holes.
class FR_RecognizeDrillHoles : public FR_Recognizer
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeDrillHoles, FR_Recognizer)

public:

  //! Ctor.
  //! \param[in] shape    full CAD model.
  //! \param[in] doHard   indicates whether to indicate "hard" holes.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeDrillHoles(const TopoDS_Shape&  shape,
                                const bool           doHard   = false);

  //! Ctor.
  //! \param[in] aag      attributed adjacency graph.
  //! \param[in] doHard   indicates whether to indicate "hard" holes.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeDrillHoles(const Handle(FR_AAG)& aag,
                                const bool                 doHard   = false);

public:

  //! Sets the linear tolerance to use. If not set, the tolerance will be
  //! taken from the model.
  //! \param[in] tol the tolerance to set.
  EXTENSION_EXPORT void
    SetLinearTolerance(const double tol);

  //! Sets the precision of canonical recognition to use.
  //! \param[in] prec the precision to set.
  EXTENSION_EXPORT void
    SetCanRecPrecision(const double prec);

  //! Sets hard feature recognition mode.
  EXTENSION_EXPORT void
    SetHardFeatureModeOn();

  //! Disables hard feature recognition mode.
  EXTENSION_EXPORT void
    SetHardFeatureModeOff();

  //! Sets hard features recognition mode state by the flag.
  //! \param[in] isOn the Boolean value to set.
  EXTENSION_EXPORT void
    SetHardFeatureMode(const bool isOn);

  //! Turns on/off pure conical holes detection mode.
  //! \param[in] isOn value to set.
  EXTENSION_EXPORT void
    SetPureConicalAllowed(const bool isOn);

  //! Sets a set of seed faces to start the recognition from. If seed faces are
  //! not specified, the algorithm will traverse the entire AAG trying each
  //! face as a seed one.
  //! \param[in] fids the face IDs to use as seeds.
  EXTENSION_EXPORT void
    SetSeedFaceIds(const TColStd_PackedMapOfInteger& fids);

  //! Sets the collection of face IDs to exclude from the consideration.
  //! Use this method to narrow down the candidate list of the seed faces.
  //! \param[in] fids the face IDs to exclude from the collection of seeds.
  EXTENSION_EXPORT void
    SetFaceIdsToExclude(const TColStd_PackedMapOfInteger& fids);

public:

  //! Performs recognition.
  //! \param[in] radius max radius (infinite by default).
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    Perform(const double radius = Precision::Infinite());

protected:

  //! Iterates AAG and recognizes features.
  //! \param[in] radius max radius.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    performInternal(const double radius);

  //! Extension point for derived classes to provide their feature
  //! matching logic.
  //! \param[in]  cc      the connected component representing the reduced
  //!                     problem graph to apply matching on.
  //! \param[out] feature the found features.
  EXTENSION_EXPORT virtual void
    matchConnectedComponent(const Handle(FR_AAG)& cc,
                            FR_Feature&           feature) override;

protected:

  double                     m_fLinToler;      //!< Linear tolerance to use.
  double                     m_fCanRecPrec;    //!< Precision of canonical recognition.
  bool                       m_bHardMode;      //!< Hard feature mode (on/off).
  bool                       m_bPureConicalOn; //!< Pure conical holes will be detected in this mode.
  TColStd_PackedMapOfInteger m_seeds;          //!< IDs of the user-defined seed faces.
  TColStd_PackedMapOfInteger m_xSeeds;         //!< IDs of the excluded faces.

};

#endif
