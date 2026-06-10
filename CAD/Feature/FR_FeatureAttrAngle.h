
#ifndef FR_FeatureAttrAngle_h
#define FR_FeatureAttrAngle_h

// FR includes
#include "./FR_FeatureAttrAdjacency.h"
#include "./FR_FeatureAngleType.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Attribute storing information about feature angle between faces.
class FR_FeatureAttrAngle : public FR_FeatureAttrAdjacency
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_FeatureAttrAngle, FR_FeatureAttrAdjacency)

public:

  //! Creates feature angle attribute.
  FR_FeatureAttrAngle()
  //
  : FR_FeatureAttrAdjacency (),
    m_angleType                  (FeatureAngleType_Undefined),
    m_fAngleRad                  (0.)
  {}

  //! Creates feature angle attribute.
  //! \param[in] type angle type.
  //! \param[in] ang  angle in radians.
  FR_FeatureAttrAngle(const FR_FeatureAngleType type,
                           const double                   ang)
  //
  : FR_FeatureAttrAdjacency (),
    m_angleType                  (type),
    m_fAngleRad                  (ang)
  {}

  //! Creates feature angle attribute with common edges.
  //! \param[in] type              angle type.
  //! \param[in] ang               angle in radians.
  //! \param[in] commonEdgeIndices indices of common edges.
  FR_FeatureAttrAngle(const FR_FeatureAngleType    type,
                           const double                      ang,
                           const TColStd_PackedMapOfInteger& commonEdgeIndices)
  //
  : FR_FeatureAttrAdjacency (commonEdgeIndices),
    m_angleType                  (type),
    m_fAngleRad                  (ang)
  {}

public:

  //! \return static GUID associated with this type of attribute.
  static const Standard_GUID& GUID()
  {
    static Standard_GUID guid("ADCC7A78-E062-4CE2-B394-7F25DDFD555E");
    return guid;
  }

  //! \return GUID associated with this type of attribute.
  virtual const Standard_GUID& GetGUID() const
  {
    return GUID();
  }

  //! \return human-readable name of the attribute.
  virtual const char* GetName() const override
  {
    return "Dihedral angle";
  }

protected:

    
  //! Dumps extra props to JSON.
 virtual void dumpJSON(std::ostream& out,
                        const int         indent) const
  {
    /*std::string ws(indent, ' ');
    std::string nl = "\n" + ws;

    out << "," << nl << FR_Utils::Str::Quoted(asiPropName_AngleType) << ": "
                     << FR_Utils::Str::Quoted( FR_FeatureAngle::ToString(m_angleType) );

    out << "," << nl << FR_Utils::Str::Quoted(asiPropName_Value) << ": "
                     << m_fAngleRad;*/
  }

public:

  //! \return type of the angle between faces.
  FR_FeatureAngleType GetAngleType() const { return m_angleType; }

  //! Sets the type of an angle between faces.
  //! \param[in] type the angle type.
  void SetAngleType(const FR_FeatureAngleType type) { m_angleType = type; }

  //! \return angle value in radians.
  double GetAngleRad() const { return m_fAngleRad; }

  //! Sets the angle value in radians.
  void SetAngleRad(const double ang) { m_fAngleRad = ang; }

protected:

  FR_FeatureAngleType m_angleType; //!< Angle type.
  double                   m_fAngleRad; //!< Angle in radians.

};

#endif
