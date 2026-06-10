
#ifndef FR_FeatureAngleType_h
#define FR_FeatureAngleType_h

// FR includes
#include "./FR.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Type of angle. Numerical values associated with the items are taken
//! from fundamental paper of Joshi "Graph-based heuristics for recognition
//! of machined features from a 3D solid model", 1988.
enum FR_FeatureAngleType
{
  FeatureAngleType_Undefined     = -1,
  FeatureAngleType_Concave       =  0,
  FeatureAngleType_Convex        =  1,
  FeatureAngleType_Smooth        =  2,
  FeatureAngleType_SmoothConcave =  3,
  FeatureAngleType_SmoothConvex  =  4,
  FeatureAngleType_NonManifold   =  5,
  //
  FeatureAngleType_LAST
};

//! \ingroup ASI_AFR
//!
//! Auxiliary functions for dealing with feature angles.
namespace FR_FeatureAngle
{
  //! Returns true if the passed angle type is definite, i.e., it is
  //! not undefined and manifold.
  //! \param[in] type type to check.
  //! \return true/false.
  inline bool IsDefinite(const FR_FeatureAngleType type)
  {
    if ( type == FeatureAngleType_Undefined ||
         type == FeatureAngleType_NonManifold )
      return false;

    return true;
  }

  //! Returns true if the passed angle type is a smooth angle.
  //! \param[in] type type to check.
  //! \return true/false.
  inline bool IsSmooth(const FR_FeatureAngleType type)
  {
    if ( type == FeatureAngleType_Smooth ||
         type == FeatureAngleType_SmoothConcave ||
         type == FeatureAngleType_SmoothConvex )
      return true;

    return false;
  }

  //! Returns true if the passed angle type is a convex angle.
  //! \param[in] type type to check.
  //! \return true/false.
  inline bool IsConvex(const FR_FeatureAngleType type)
  {
    if ( type == FeatureAngleType_Convex ||
         type == FeatureAngleType_SmoothConvex )
      return true;

    return false;
  }

  //! Returns true if the passed angle type is a concave angle.
  //! \param[in] type type to check.
  //! \return true/false.
  inline bool IsConcave(const FR_FeatureAngleType type)
  {
    if ( type == FeatureAngleType_Concave ||
         type == FeatureAngleType_SmoothConcave )
      return true;

    return false;
  }

  //! Converts the passed feature angle type to string.
  //! \param[in] angle the angle type to convert to string.
  //! \return string representation of the passed angle type.
  inline const char*
    ToString(const FR_FeatureAngleType angle)
  {
    switch ( angle )
    {
      case FeatureAngleType_Concave:
        return "concave";
      case FeatureAngleType_Convex:
        return "convex";
      case FeatureAngleType_Smooth:
        return "smooth";
      case FeatureAngleType_SmoothConcave:
        return "smooth concave";
      case FeatureAngleType_SmoothConvex:
        return "smooth convex";
      case FeatureAngleType_Undefined:
      default:
        break;
    }

    return "undefined";
  }
};

#endif
