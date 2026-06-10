#ifndef FR_Recognizer_h
#define FR_Recognizer_h

// FR includes
#include "./FR_FeatureFaces.h"

// OpenCascade includes
#include <TopTools_IndexedMapOfShape.hxx>

#include "../Common/Extension_Export.hxx"

class FR_AAG;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Abstract base class for feature recognizers. It provides a unified
//! usage pattern, data structures and a fixed call procedure for all AAG-based
//! feature identification algorithms.
EXTENSION_EXPORT class FR_Recognizer : public Standard_Transient
{
  // OCCT RTTI
    DEFINE_STANDARD_RTTI_INLINE(FR_Recognizer, Standard_Transient)

public:

  //! Ctor.
  //! \param[in] masterCAD full CAD model.
  //! \param[in] aag       AAG (will be created from CAD if nullptr is passed).
  //! \param[in] progress  progress notifier.
  //! \param[in] plotter   imperative plotter.
    FR_Recognizer(const TopoDS_Shape&        masterCAD,
                       const Handle(FR_AAG)& aag);

  //! Ctor.
  //! \param[in] aag      AAG (should not be nullptr here).
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
    FR_Recognizer(const Handle(FR_AAG)& aag);

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
    FR_Recognizer();

  //! Destructor.
    ~FR_Recognizer();

public:

  //! Initializes the recognition tool with the part shape in question.
  //! \param[in] masterCAD master CAD model.
 void Init(const TopoDS_Shape& masterCAD);

  //! Classifies the detected features using graph isomorphism matching.
  //! \param[out] feature          found features.
  //! \param[in]  indicateProgress whether to run progress indication.
  //! \return true in case of success, false -- otherwise.
 virtual bool Classify(FR_Feature& feature, const bool indicateProgress = true);

public:

  //! \return non-const reference to the detected faces.
 TopTools_IndexedMapOfShape& ChangeResultFaces();

  //! \return result collection of detected faces as transient shapes.
 const TopTools_IndexedMapOfShape& GetResultFaces() const;

  //! \return non-const reference to the detected faces IDs.
 FR_Feature& ChangeResultIndices();

  //! \return result collection of detected faces as integer ids.
 const FR_Feature& GetResultIndices() const;

  //! \return working AAG instance.
 const Handle(FR_AAG) & GetAAG() const;

  //! Sets AAG to operate on.
  //! \param[in] aag AAG instance to set.
 void SetAAG(const Handle(FR_AAG) & aag);

protected:

  //! Extension point for derived classes to provide their feature
  //! matching logic.
  //! \param[in]  cc      the connected component representing the reduced
  //!                     problem graph to apply matching on.
  //! \param[out] feature the found features.
 virtual void matchConnectedComponent(const Handle(FR_AAG) & cc, FR_Feature& feature);

protected:

  //-------------------------------------------------------------------------//
  // IN
  //-------------------------------------------------------------------------//

  TopoDS_Shape        m_master; //!< Master CAD.
  Handle(FR_AAG) m_aag;    //!< Master AAG.

  //-------------------------------------------------------------------------//
  // OUT
  //-------------------------------------------------------------------------//

  struct
  {
    TopTools_IndexedMapOfShape faces; //!< Detected feature faces.
    FR_Feature            ids;   //!< Indices of the detected feature faces.

    void Clear()
    {
      faces.Clear();
      ids.Clear();
    }
  } m_result;

};

#endif
