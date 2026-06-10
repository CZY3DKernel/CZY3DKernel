#ifndef FR_RecognizeShafts_h
#define FR_RecognizeShafts_h

// FR includes
#include "./FR_Recognizer.h"

// OpenCascade includes
#include <Precision.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Utility to recognize shaft features ("holes inverted").
class FR_RecognizeBaseFaces : public FR_Recognizer
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeBaseFaces, FR_Recognizer)

public:

  //! Ctor.
  //! \param[in] aag      the attributed adjacency graph.
  //! \param[in] progress the progress notifier.
  //! \param[in] plotter  the imperative plotter.
  EXTENSION_EXPORT
    FR_RecognizeBaseFaces(const Handle(FR_AAG)& aag);

public:

  //! Performs recognition.
  //! \param[in] radius the max radius (infinite by default).
  //! \return true in the case of success, false -- otherwise.
 EXTENSION_EXPORT bool Perform();

public:

  //! \return the stored AAG.
  const Handle(FR_AAG)& GetAAG() const
  {
    return m_aag;
  }

protected:

};

#endif
