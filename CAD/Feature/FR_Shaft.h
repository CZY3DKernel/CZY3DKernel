#ifndef FR_Shaft_h
#define FR_Shaft_h

// FR includes
#include "./FR_FeatureFaces.h"

// OpenCascade includes
#include <gp_Ax1.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Descriptor for a shaft feature.
struct FR_Shaft : public Standard_Transient
{
  DEFINE_STANDARD_RTTI_INLINE(FR_Shaft, Standard_Transient)

  FR_Feature fids;     //!< Face IDs of a shaft.
  double          diameter; //!< Diameter.
  double          length;   //!< Length.
  gp_Ax1 axis;           //!< Shaft axis.

  //! Default ctor.
  EXTENSION_EXPORT
    FR_Shaft();

  //! Checks if the passed structure equals this one.
  //! \param[in] other     the shaft structure to compare this one with.
  //! \param[in] linToler  the linear tolerance.
  //! \param[in] angTolDeg the angular tolerance in degrees.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT bool
    IsEqual(const Handle(FR_Shaft)& other,
            const double                 linToler,
            const double                 angTolDeg) const;

  //! Constructs shaft data structure from a JSON object.
  //! \param[in]  pJsonGenericObj the JSON object to construct the shaft structure from.
  //! \param[out] shaft           the outcome shaft.
  EXTENSION_EXPORT static void
    FromJSON(void*                  pJsonGenericObj,
             Handle(FR_Shaft)& shaft);

  //! Serializes the passed shaft structure to JSON (the passed `out` stream).
  //! \param[in]     shaft  the shaft structure to serialize.
  //! \param[in]     indent the pretty indentation shift.
  //! \param[in,out] out    the output JSON string stream.
  EXTENSION_EXPORT static void
    ToJSON(const Handle(FR_Shaft)& shaft,
           const int                    indent,
           std::ostream&                out);
};

#endif // FR_Shaft_h
