#ifndef FR_RecognizeDrillHolesRule_h
#define FR_RecognizeDrillHolesRule_h

// FR includes
#include "./FR_RecognitionRule.h"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Feature rule for drilled holes.
class FR_RecognizeDrillHolesRule : public FR_RecognitionRule
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_RecognizeDrillHolesRule, FR_RecognitionRule)

public:

  //! Constructs the rule initializing it with the given AAG iterator.
  //! \param[in] aag_it   AAG iterator.
  //! \param[in] target_R radius of interest (upper barrier).
  //! \param[in] progress progress entry.
  //! \param[in] plotter  plotter entry.
  FR_RecognizeDrillHolesRule(const Handle(FR_AAGIterator)& aag_it,
                                  const double                       target_R)
  //
  : FR_RecognitionRule (aag_it),
    m_fRadius               (0.0),
    m_fTargetRadius         (target_R),
    m_bHardMode             (false),
    m_bPureConicalOn        (true),
    m_fLinToler             (1.e-6),
    m_fAngToler             (1.0/180.0*M_PI), // 1 degree is a default precision for coaxiality check.
    m_fCanRecPrec           (1.e-3)
  {}

private:

  //! Recognizes feature starting from the current position of AAG iterator.
  //! \param[out] featureFaces   the detected feature faces.
  //! \param[out] featureIndices the indices of the detected feature faces.
  //! \return true/false.
  virtual bool
    recognize(TopTools_IndexedMapOfShape& featureFaces,
              TColStd_PackedMapOfInteger& featureIndices) override;

public:

  //! \return radius of the detected hole.
  double GetRadius() const { return m_fRadius; }

  //! \return target radius.
  double GetTargetRadius() const { return m_fTargetRadius; }

  //! Turns on/off hard feature detection mode.
  //! \param[in] isOn value to set.
  void SetHardFeatureMode(const bool isOn) { m_bHardMode = isOn; }

  //! Enables hard feature mode.
  void SetHardFeatureModeOn() { this->SetHardFeatureMode(true); }

  //! Disables hard feature mode.
  void SetHardFeatureModeOff() { this->SetHardFeatureMode(false); }

  //! Turns on/off pure conical holes detection mode.
  //! \param[in] isOn value to set.
  void SetPureConicalAllowed(const bool isOn) { m_bPureConicalOn = isOn; }

  //! Sets linear tolerance to use.
  //! \param[in] tol the tolerance to use.
  void SetLinearTolerance(const double tol) { m_fLinToler = tol; }

  //! Sets the precision of canonical recognition to use.
  //! \param[in] prec the precision to set.
  void SetCanRecPrecision(const double prec) { m_fCanRecPrec = prec; }

protected:

  //! Checks whether the passed face is a cylinder. Also performs
  //! canonical recognition. Does not extract any props.
  //! \param[in] fid ID of the face to check.
  //! \return true/false.
  bool isCylindrical(const int fid) const;

  //! Checks whether the passed face is a cylinder. Also performs
  //! canonical recognition.
  //! \param[in]  fid          face to check.
  //! \param[in]  checkNoHints indicates whether to check for feature hints
  //!                          to avoid having them.
  //! \param[in]  checkBore    checks if this face is a bore, i.e., a small
  //!                          shift along its normal decreases the radius.
  //! \param[out] radius       radius of the host cylinder.
  //! \param[out] angle_min    min angle of the cylindrical surface.
  //! \param[out] angle_max    max angle of the cylindrical surface.
  //! \param[out] ax           axis of the cylinder.
  //! \return true/false.
  bool isCylindrical(const int  fid,
                     const bool checkNoHints,
                     const bool checkBore,
                     double&    radius,
                     double&    angle_min,
                     double&    angle_max,
                     gp_Ax1&    ax) const;

  //! Checks whether the passed face is a cone.
  //! \param[in]  fid          face to check.
  //! \param[in]  checkNoHints indicates whether to check for feature hints
  //!                          to avoid having them.
  //! \param[out] radius       minor radius of the host cone.
  //! \param[out] angle_min    min angle of the surface.
  //! \param[out] angle_max    max angle of the surface.
  //! \param[out] ax           axis of the cone.
  //! \return true/false.
  bool isConical(const int  fid,
                 const bool checkNoHints,
                 double&    radius,
                 double&    angle_min,
                 double&    angle_max,
                 gp_Ax1&    ax) const;

  //! Recursive function to iterate cylindrical faces that are
  //! neighbors to the given seed.
  //! \param[in]     sid       the 1-based ID of the starting face.
  //! \param[in]     fid       the 1-based ID of the seed face.
  //! \param[in]     refRadius the reference radius.
  //! \param[in]     refAxis   the reference axis.
  //! \param[in,out] sumAng    the collected total angle.
  //! \param[out]    collected the collected cylindrical faces.
  void visitNeighborCylinders(const int        sid,
                              const int        fid,
                              const double     refRadius,
                              const gp_Ax1&    refAxis,
                              double&          sumAng,
                              FR_Feature& collected);

  //! Recursively visits neighbor coaxial toroidal faces.
  void visitNeighborToruses(const int        sid,
                            const int        fid,
                            const double     refRadius,
                            const gp_Ax1&    refAxis,
                            FR_Feature& collected);

  //! Checks if the passed turned face is internal or external, depending
  //! on its norm field orientation.
  //! \param[in] fid      the 1-based face index to check.
  //! \param[in] diameter the presumably known diameter.
  //! \param[in] ax       the presumably known axis.
  //! \return true for internal, false for external.
  bool
    isInternal(const int     fid,
               const double  diameter,
               const gp_Ax1& ax) const;

  //! Checks if the passed turned face is internal or external, depending
  //! on its norm field orientation.
  //! \param[in] fid      the 1-based face index to check.
  //! \param[in] diameter the presumably known diameter.
  //! \param[in] u        the U coordinate of the probe point on the face of interest.
  //! \param[in] v        the V coordinate of the probe point on the face of interest.
  //! \param[in] ax       the presumably known axis.
  //! \return true for internal, false for external.
  bool
    isInternal(const int     fid,
               const double  diameter,
               const double  u,
               const double  v,
               const gp_Ax1& ax) const;

protected:

  double m_fRadius;        //!< Radius of the detected hole.
  double m_fTargetRadius;  //!< Target radius.
  bool   m_bHardMode;      //!< Hard feature detection mode.
  bool   m_bPureConicalOn; //!< Pure conical holes will be detected in this mode.
  double m_fLinToler;      //!< Linear tolerance.
  double m_fAngToler;      //!< Angular tolerance.
  double m_fCanRecPrec;    //!< Precision of canonical recognition.

};

#endif
