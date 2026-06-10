
#ifndef FR_FeatureFaces_h
#define FR_FeatureFaces_h

// FR includes
#include "./FR_FeatureType.h"

// OCCT includes
#include <NCollection_DataMap.hxx>
#include <Standard_GUID.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Feature ID.
typedef int FR_FeatureId;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Feature as a set of indices of faces.
typedef TColStd_PackedMapOfInteger FR_Feature;

//-----------------------------------------------------------------------------

//! \ingroup ASI_CORE
//!
//! Technical namespace for common functions.
namespace FR
{
  //! Dumps the passed feature face IDs to the standard output and
  //! debugging streams. This function is supposed to be used as
  //! "watch" for features. To use in Visual Studio, run in Command
  //! Window:
  //!
  //! `? ({,,FR.dll}FR::Dump)(feature)`
  //!
  //! Here `feature` is of type `TColStd_PackedMapOfInteger`.
  //!
  //! \param[in] feature the feature to dump.
  void
    Dump(const FR_Feature& feature);
};

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Features by indices.
typedef NCollection_DataMap<FR_FeatureId, FR_Feature> FR_Features;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Handy typedef for indices of feature faces organized by feature types.
typedef NCollection_DataMap<int, FR_Features> FR_FeaturesByType;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Undefined GUID.
typedef Standard_GUID FR_BadGuid;

#endif
