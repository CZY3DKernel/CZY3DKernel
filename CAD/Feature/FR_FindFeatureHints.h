
#ifndef FR_FindFeatureHints_h
#define FR_FindFeatureHints_h

// FR includes
#include "./FR.h"

// OCCT includes
#include <Geom2d_Line.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Utility to detect feature hints. A feature hint is an imprint of one
//! feature on another. This utility can be helpful in finding feature
//! interactions which is known to be a major cornerstone in feature
//! identification field.
class FR_FindFeatureHints : public Standard_Transient
{
public:

  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_FindFeatureHints, Standard_Transient)

public:

  //! Constructor.
  //! \param[in] face     working face.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  EXTENSION_EXPORT FR_FindFeatureHints(const TopoDS_Face& face);

public:

  //! Checks whether the working face is "puzzled" in its domain.
  //! \return true/false.
  EXTENSION_EXPORT bool
    IsPuzzled();

protected:

  //! Attempts to extract line from the given edge.
  //! \param[in] edge target edge.
  //! \return geometric line or null if the edge is not of linear type.
 EXTENSION_EXPORT Handle(Geom2d_Line) edgeAsLine(const TopoDS_Edge& edge) const;

protected:

  TopoDS_Face m_face;  //!< Working face.

};

#endif
