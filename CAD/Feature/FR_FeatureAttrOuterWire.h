
#ifndef FR_FeatureAttrOuterWire_h
#define FR_FeatureAttrOuterWire_h

// FR includes
#include "./FR.h"

// FR includes
#include "./FR_FeatureAttrFace.h"

// OpenCascade includes
#include <TopoDS_Wire.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! AAG attribute to store outer wire of a face.
class FR_FeatureAttrOuterWire : public FR_FeatureAttr
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_FeatureAttrOuterWire, FR_FeatureAttr)

public:

  //! Ctor.
  FR_FeatureAttrOuterWire()
  //
  : FR_FeatureAttr()
  {}

  //! Complete ctor.
  FR_FeatureAttrOuterWire(const TopoDS_Wire& _wire)
  : FR_FeatureAttr (),
    wire                (_wire)
  {}

  //! \return static GUID associated with this type of attribute.
  static const Standard_GUID& GUID()
  {
      static Standard_GUID guid("8DD0E0C0-481E-4370-A319-6A75D87FBF2E");
    return guid;
  }

  //! \return GUID associated with this type of attribute.
  virtual const Standard_GUID& GetGUID() const override
  {
    return GUID();
  }

  //! \return human-friendly name of the attribute.
  virtual const char* GetName() const override
  {
    return "Outer wire";
  }

public:

  TopoDS_Wire wire;

};

#endif
