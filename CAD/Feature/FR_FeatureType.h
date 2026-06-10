
#ifndef FR_FeatureType_h
#define FR_FeatureType_h

// FR includes
#include "./FR.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Enumeration for basic feature types.
enum FR_FeatureType
{
  FeatureType_UNDEFINED = 0,   //!< Undefined feature type.

  //-------------------------------------------------------------------------//
  // Volumetric features
  FeatureType_HoleCylindrical, //!< Cylindrical hole.
  FeatureType_Isolated,        //!< Isolated feature.

  //-------------------------------------------------------------------------//
  // Surface features
  FeatureType_BlendOrdinary,   //!< Ordinary blend.

  //-------------------------------------------------------------------------//
  FeatureType_LAST             //!< Last item is reserved for extensions.
};

#endif
