#ifndef FR_RecognizeShafts_h
#define FR_RecognizeShafts_h

// FR includes
#include "./FR_Recognizer.h"
#include "./FR_Shaft.h"

#include <vector>

// OpenCascade includes
#include <Precision.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Utility to recognize shaft features ("holes inverted").
EXTENSION_EXPORT class FR_RecognizeShafts : public FR_Recognizer
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeShafts, FR_Recognizer)

public:

  //! Ctor.
  //! \param[in] aag      the attributed adjacency graph.
  //! \param[in] progress the progress notifier.
  //! \param[in] plotter  the imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeShafts(const Handle(FR_AAG)& aag);

public:

  //! Sets the linear tolerance to use. If not set, the tolerance will be
  //! taken from the model.
  //! \param[in] tol the tolerance to set.
  EXTENSION_EXPORT void
    SetLinearTolerance(const double tol);

  //! Sets the universum of faces to recognize.
  //! \param[in] domain the faces to recognize.
  EXTENSION_EXPORT void
    SetDomain(const FR_Feature& domain);

  //! Sets the collection of face IDs to exclude from the consideration.
  //! Use this method to narrow down the candidate list of the seed faces.
  //! \param[in] fids the face IDs to exclude from the collection of seeds.
  EXTENSION_EXPORT void
    SetExcludedFaces(const FR_Feature& fids);

public:

  //! Performs recognition.
  //! \param[in] radius the max radius (infinite by default).
  //! \return true in the case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    Perform(const double radius = Precision::Infinite());

public:

  //! \return the stored AAG.
  const Handle(FR_AAG)& GetAAG() const
  {
    return m_aag;
  }

  //! \return the recognized shaft features.
  const std::vector<Handle(FR_Shaft)>& GetShafts() const
  {
    return m_shafts;
  }

protected:

  //! Performs recognition.
  //! \param[in] radius the max radius.
  //! \return true in the case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    performInternal(const double radius);

  //! Computes shaft properties for the given collection of indices.
  //! \param[in]  fids  the face IDs of an isolated shaft feature.
  //! \param[out] shaft the populated shaft feature.
  EXTENSION_EXPORT void
    populateShaft(const FR_Feature& fids,
                  Handle(FR_Shaft)& shaft) const;

protected:

  double                             m_fLinToler; //!< Linear tolerance to use.
  FR_Feature                    m_domain;    //!< Universum.
  FR_Feature                    m_xSeeds;    //!< IDs of the excluded faces.
  std::vector<Handle(FR_Shaft)> m_shafts;    //!< Extracted shafts with props.

};

#endif
