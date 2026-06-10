#ifndef FR_ShapePartnerHasher_h
#define FR_ShapePartnerHasher_h

// FR includes
#include "./FR.h"

#include <assert.h>

// OCCT includes
#include <TopoDS_Shape.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_CORE
//!
//! Hasher which does not take into account neither locations nor
//! orientations of shapes.
class FR_ShapePartnerHasher
{
public:

  static int HashCode(const TopoDS_Shape& S, const int Upper)
  {
      assert(false);
    const int I  = (int) ptrdiff_t( S.TShape().operator->() );
    //const int HS = ::HashCode(I, Upper);
    const int HS = 0;
    //
    return HS;
  }

  static bool IsEqual(const TopoDS_Shape& S1, const TopoDS_Shape& S2)
  {
    return S1.IsPartner(S2);
  }

  size_t operator()(const TopoDS_Shape& theShape) const noexcept
  {
      const int I = (int) ptrdiff_t(theShape.TShape().operator->());
      
      return I;
  }

  bool operator()(const TopoDS_Shape& S1, const TopoDS_Shape& S2) const noexcept
  {
      return S1.IsPartner(S2);
  }
};

#endif
