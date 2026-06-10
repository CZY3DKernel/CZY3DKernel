
#ifndef FR_CheckValidity_h
#define FR_CheckValidity_h

// FR includes
#include "./FR.h"

// OCCT includes
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_MODELING
//!
//! Utility to check validity of shapes.
class FR_CheckValidity : public Standard_Transient
{
public:

  //! Returns maximum tolerance value bound to the passed shape.
  //! \param[in] shape shape to check.
  //! \return maximum tolerance value.
  EXTENSION_EXPORT static double
    MaxTolerance(const TopoDS_Shape& shape);


};

#endif
