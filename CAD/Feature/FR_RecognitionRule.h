#ifndef FR_RecognitionRule_h
#define FR_RecognitionRule_h

// FR includes
#include "./FR_AAGIterator.h"

// OCCT includes
#include <TopTools_IndexedMapOfShape.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Abstract class for feature recognition rules used in recognizers. The idea
//! behind any "rule" is to provide the recognition logic at certain AAG iterator's
//! position. I.e., whenever we are at the rule's `recognize()` method, we start from
//! an externally defined position of a "cursor" pointing to the candidate seed face.
//! Therefore, a rule itself is not bothered with selection of seed faces (the latter
//! is done by the caller code).
class FR_RecognitionRule : public Standard_Transient
{
public:

  DEFINE_STANDARD_RTTI_INLINE(FR_RecognitionRule, Standard_Transient)

public:

  //! Constructs the base rule initializing it with the given AAG iterator.
  //! \param[in] it       AAG iterator.
  //! \param[in] progress progress entry.
  //! \param[in] plotter  plotter entry.
  FR_RecognitionRule(const Handle(FR_AAGIterator)& it)
  {
    m_it = it;
  }

public:

  //! \return faces traversed during recognition.
  const FR_Feature& JustTraversed() const
  {
    return m_traversed;
  }

  //! Sets faces as traversed.
  //! \param[in] fid index of the just traversed face.
  void SetTraversed(const int fid)
  {
    m_traversed.Add(fid);
  }

  //! Adds the passed face IDs to the traversed set.
  //! \param[in] fids the IDs to add.
  void AddTraversed(const FR_Feature& fids)
  {
    m_traversed.Unite(fids);
  }

  //! Checks whether the passed face has been traversed.
  //! \param[in] fid the index to check.
  //! \return true/false.
  bool IsTraversed(const int fid) const
  {
    return m_traversed.Contains(fid);
  }

  //! Performs recognition.
  //! \param[out] featureFaces   detected faces.
  //! \param[out] featureIndices indices of the detected faces.
  //! \return true in case of success, false -- otherwise.
  bool Recognize(TopTools_IndexedMapOfShape& featureFaces,
                 FR_Feature&            featureIndices)
  {
    m_traversed.Clear();
    //
    return this->recognize(featureFaces, featureIndices);
  }

private:

  //! Extension point for the derived classes to do real job.
  virtual bool
    recognize(TopTools_IndexedMapOfShape& featureFaces,
              FR_Feature&            featureIndices) = 0;

protected:

  Handle(FR_AAGIterator) m_it;        //!< AAG iterator.
  FR_Feature             m_traversed; //!< Faces traversed during recognition.

};

#endif
