#include "./FR_Shaft.h"

// FR includes
//#include <FR_JsonDict.h>
#include "./FR_Utils.h"

#if defined USE_RAPIDJSON

// Rapidjson includes
#include <rapidjson/document.h>

typedef rapidjson::Document::Array     t_jsonArray;
typedef rapidjson::Document::ValueType t_jsonValue;

#endif

//-----------------------------------------------------------------------------

FR_Shaft::FR_Shaft()
//
: Standard_Transient (),
  diameter           (0.),
  length             (0.)
{}

//-----------------------------------------------------------------------------

bool FR_Shaft::IsEqual(const Handle(FR_Shaft)& other,
                            const double                 linToler,
                            const double                 angTolDeg) const
{
  if ( !this->fids.IsEqual(other->fids) )
    return false;

  if ( Abs(this->diameter - other->diameter) > linToler )
    return false;

  if ( Abs(this->length - other->length) > linToler )
    return false;

  if ( !FR_Utils::Verify::AreEqualAxes(this->axis, other->axis, angTolDeg) )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

void FR_Shaft::FromJSON(void*                  pJsonGenericObj,
                             Handle(FR_Shaft)& shaft)
{
#if defined USE_RAPIDJSON
  t_jsonValue*
    pJsonObj = reinterpret_cast<t_jsonValue*>(pJsonGenericObj);

  // Iterate members of the shaft object.
  for ( t_jsonValue::MemberIterator mit = pJsonObj->MemberBegin();
        mit != pJsonObj->MemberEnd(); mit++ )
  {
    std::string prop( mit->name.GetString() );

    // Face IDs.
    if ( prop == asiPropName_FaceIds )
    {
      if ( shaft.IsNull() )
        shaft = new FR_Shaft;

      t_jsonArray arr = mit->value.GetArray();
      FR_Utils::Json::ReadFeature(&arr, shaft->fids);
    }

    // Diameter.
    else if ( prop == asiPropName_Diameter )
    {
      if ( shaft.IsNull() )
        shaft = new FR_Shaft;

      shaft->diameter = mit->value.GetDouble();
    }

    // Length.
    else if ( prop == asiPropName_Length )
    {
      if ( shaft.IsNull() )
        shaft = new FR_Shaft;

      shaft->length = mit->value.GetDouble();
    }

    // Axis.
    else if ( prop == asiPropName_Axis )
    {
      if ( !mit->value.IsNull() )
      {
        if ( shaft.IsNull() )
          shaft = new FR_Shaft;

        t_jsonArray arr = mit->value.GetArray();

        gp_XYZ coords;
        FR_Utils::Json::ReadCoords(&arr, coords);
        //
        if ( coords.Modulus() > RealEpsilon() )
          shaft->axis = gp_Ax1(gp::Origin(), coords);
      }
    }
  }
#endif
}

//-----------------------------------------------------------------------------

void FR_Shaft::ToJSON(const Handle(FR_Shaft)& shaft,
                           const int                    indent,
                           std::ostream&                out)
{
  //std::string ws(indent, ' ');
  //std::string nl = "\n" + ws;

  ///* Dump props */

  //out <<        nl << "\"" << asiPropName_FaceIds  << "\": " << FR_Utils::Json::FromFeature(shaft->fids);
  //out << "," << nl << "\"" << asiPropName_Diameter << "\": " << shaft->diameter;
  //out << "," << nl << "\"" << asiPropName_Length   << "\": " << shaft->length;
  //out << "," << nl << "\"" << asiPropName_Axis     << "\": " << FR_Utils::Json::FromDirAsTuple( shaft->axis.Direction() );
}
