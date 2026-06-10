#ifndef FR_RecognizeCanonical_h
#define FR_RecognizeCanonical_h

// FR includes
#include "./FR_AAG.h"

// OpenCascade includes
#include <ElCLib.hxx>
#include <gce_MakeCirc.hxx>
#include <gce_MakeCirc2d.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pnt.hxx>
#include <ShapeAnalysis.hxx>
#include <TColgp_Array1OfPnt2d.hxx>

//-----------------------------------------------------------------------------

//! \ingroup ASI_MODELING
//!
//! Canonical recognition tool.
class FR_RecognizeCanonical : public Standard_Transient
{
public:

  //! Checks if the passed surface is planar. The passed tolerance is used
  //! for recognizing as planar those surfaces that are not defined as
  //! analytic planes.
  //!
  //! \param[in]  surface  the surface in question.
  //! \param[in]  toler    the tolerance to use.
  //! \param[out] pln      the extracted plane.
  //! \param[in]  progress the progress notifier.
  //! \param[in]  plotter  the imperative plotter.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT static bool
    CheckIsPlanar(const Handle(Geom_Surface)& surface,
                  const double                toler,
                  gp_Pln&                     pln);

  //! Checks if the passed surface can be represented with an analytical
  //! cylinder.
  //!
  //! \param[in]  surface     the surface in question.
  //! \param[in]  uMinSurf    the min U parameter.
  //! \param[in]  uMaxSurf    the max U parameter.
  //! \param[in]  vMinSurf    the min V parameter.
  //! \param[in]  vMaxSurf    the max V parameter.
  //! \param[in]  toler       the tolerance to use.
  //! \param[in]  checkRanges the Boolean flag indicating whether to extract
  //!                         the parametric ranges of a cylindrical surface.
  //!                         If not enabled, this function will only check
  //!                         that a cylindrical primitive fits the initial
  //!                         surface without any trimming.
  //! \param[out] cyl         the extracted cylinder.
  //! \param[out] uMinCyl     the U min parameter of the cylinder's domain.
  //! \param[out] uMaxCyl     the U max parameter of the cylinder's domain.
  //! \param[out] vMinCyl     the V min parameter of the cylinder's domain.
  //! \param[out] vMaxCyl     the V max parameter of the cylinder's domain.
  //! \param[in]  progress    the progress notifier.
  //! \param[in]  plotter     the imperative plotter.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT static bool
    CheckIsCylindrical(const Handle(Geom_Surface)& surface,
                       const double                uMinSurf,
                       const double                uMaxSurf,
                       const double                vMinSurf,
                       const double                vMaxSurf,
                       const double                toler,
                       const bool                  checkRanges,
                       gp_Cylinder&                cyl,
                       double&                     uMinCyl,
                       double&                     uMaxCyl,
                       double&                     vMinCyl,
                       double&                     vMaxCyl);

  //! Checks if the passed surface can be represented with an analytical
  //! cylinder. This function only checks the shape, it does not extract
  //! parametric ranges.
  //!
  //! \param[in]  surface  the spline surface in question.
  //! \param[in]  toler    the tolerance to use.
  //! \param[out] cyl      the extracted cylinder.
  //! \param[in]  progress the progress notifier.
  //! \param[in]  plotter  the imperative plotter.
  //! \return true in case of success, false -- otherwise.
  EXTENSION_EXPORT static bool
    CheckIsCylindrical(const Handle(Geom_BSplineSurface)& surface,
                       const double                       toler,
                       gp_Cylinder&                       cyl);

  //! Attempts to create a toroidal surface out of the passed `surf` surface.
  EXTENSION_EXPORT static bool
    CheckIsTorusSphere(const Handle(Geom_Surface)& surf,
                       const Handle(Geom_Circle)&  circle,
                       const Handle(Geom_Circle)&  otherCircle,
                       const double                param1,
                       const double                param2,
                       const double                param1ToCrv,
                       const double                param2ToCrv,
                       const double                toler,
                       const double                isTryUMajor,
                       Handle(Geom_Surface)&       resSurf);

  //! Attempts to recognize a surface of linear extrusion from a freeform surface.
  EXTENSION_EXPORT static bool
    CheckIsLinearExtrusion(const Handle(Geom_Surface)& surf,
                           const double                tol,
                           Handle(Geom_Line)&          straightIso,
                           Handle(Geom_Curve)&         profileIso);

  //! Attempts to recognize a surface of linear extrusion from a freeform surface.
  EXTENSION_EXPORT static bool
    CheckIsLinearExtrusion(const Handle(Geom_Surface)& surf,
                           const double                tol);

  //! Checks type of the passed surface.
  //!
  //! \param[in]  surface  the surface to check.
  //! \param[in]  uMinSurf the min U parameter.
  //! \param[in]  uMaxSurf the max U parameter.
  //! \param[in]  vMinSurf the min V parameter.
  //! \param[in]  vMaxSurf the max V parameter.
  //! \param[in]  toler    the tolerance to use.
  //! \param[out] uMinRec  the U min parameter of the recognized surface.
  //! \param[out] uMaxRec  the U max parameter of the recognized surface.
  //! \param[out] vMinRec  the V min parameter of the recognized surface.
  //! \param[out] vMaxRec  the V max parameter of the recognized surface.
  //! \param[in]  progress the progress notifier.
  //! \param[in]  plotter  the imperative plotter.
  //! \return the recognized type of the surface.
  EXTENSION_EXPORT static Handle(Standard_Type)
    CheckType(const Handle(Geom_Surface)& surface,
              const double                uMinSurf,
              const double                uMaxSurf,
              const double                vMinSurf,
              const double                vMaxSurf,
              const double                toler,
              double&                     uMinRec,
              double&                     uMaxRec,
              double&                     vMinRec,
              double&                     vMaxRec);

  //! Checks type of the passed face.
  //!
  //! \param[in] face     the face to check.
  //! \param[in] toler    the tolerance to use.
  //! \param[in] progress the progress notifier.
  //! \param[in] plotter  the imperative plotter.
  //! \return the recognized type of the surface.
  EXTENSION_EXPORT static Handle(Standard_Type)
    CheckType(const TopoDS_Face&   face,
              const double         toler);

  //! Checks if the passed array of two-dimensional points
  //! can be approximated with a straight line segment
  //! within the prescribed tolerance.
  //! \param[in]  pts   the points to check.
  //! \param[in]  toler the fitting tolerance to use.
  //! \param[out] dev   the achieved deviation.
  //! \param[out] lin   the detected line.
  //! \return true/false.
  EXTENSION_EXPORT static bool
    IsLinear(const TColgp_Array1OfPnt2d& pts,
             const double                toler,
             double&                     dev,
             gp_Lin2d&                   lin);

  //! Checks if the passed array of three-dimensional points
  //! can be approximated with a straight line segment
  //! within the prescribed tolerance.
  //! \param[in]  pts   the points to check.
  //! \param[in]  toler the fitting tolerance to use.
  //! \param[out] dev   the achieved deviation.
  //! \return true/false.
  EXTENSION_EXPORT static bool
    IsLinear(const TColgp_Array1OfPnt& pts,
             const double              toler,
             double&                   dev);

  //! Checks if the passed array of three-dimensional points
  //! can be approximated with a straight line segment
  //! within the prescribed tolerance.
  //! \param[in]  pts   the points to check.
  //! \param[in]  toler the fitting tolerance to use.
  //! \param[out] dev   the achieved deviation.
  //! \param[out] lin   the detected line.
  //! \return true/false.
  EXTENSION_EXPORT static bool
    IsLinear(const TColgp_Array1OfPnt& pts,
             const double              toler,
             double&                   dev,
             gp_Lin&                   lin);

  //! Attempts to approximate the passed curve with a circle.
  //! \param[in]  curve the input curve.
  //! \param[in]  tol   the fitting tolerance.
  //! \param[in]  c1    the first parameter on the input curve.
  //! \param[in]  c2    the second parameter on the input curve.
  //! \param[out] cf    the first parameter on the output curve.
  //! \param[out] cl    the last parameter on the output curve.
  //! \param[out] dev   the reached deviation.
  //! \return the output circle.
  EXTENSION_EXPORT static Handle(Geom_Curve)
    FitCircle(const Handle(Geom_Curve)& curve,
              const double              tol,
              const double              c1,
              const double              c2,
              double&                   cf,
              double&                   cl,
              double&                   dev);

  //! Attempts to approximate the passed curve with a circle.
  //! \param[in]  curve the input curve.
  //! \param[in]  tol   the fitting tolerance.
  //! \param[in]  c1    the first parameter on the input curve.
  //! \param[in]  c2    the second parameter on the input curve.
  //! \param[out] cf    the first parameter on the output curve.
  //! \param[out] cl    the last parameter on the output curve.
  //! \param[out] dev   the reached deviation.
  //! \return the output circle.
  EXTENSION_EXPORT static Handle(Geom2d_Curve)
    FitCircle(const Handle(Geom2d_Curve)& curve,
              const double                tol,
              const double                c1,
              const double                c2,
              double&                     cf,
              double&                     cl,
              double&                     dev);

protected:

  //! Constructs a circle primitive passing through the given
  //! triple of points.
  //! \param[in]  P0  the first point.
  //! \param[in]  P1  the second point.
  //! \param[in]  P2  the third point.
  //! \param[out] crc the constructed circle.
  //! \return true if a circle was constructed, false -- otherwise.
  template <typename TPnt,
            typename TVec,
            typename TCircPrim,
            typename TCircBuilder>
  static bool
    getCircle(const TPnt& P0,
              const TPnt& P1,
              const TPnt& P2,
              TCircPrim&  crc)
  {
    const double eps = 1.e-09;
    const double d0  = P0.Distance(P1);
    const double d1  = P0.Distance(P2);
    //
    if ( d0 < eps || d1 < eps )
      return false;

    // Control if the points are not aligned.
    double ang = 0.;
    //
    TVec p0p1(P0, P1);
    TVec p0p2(P0, P2);
    ang = p0p1.CrossSquareMagnitude(p0p2);
    //
    if ( ang < d0*d1*eps )
      return false;

    // Make circle.
    TCircBuilder mkc(P0, P1, P2);
    //
    if ( !mkc.IsDone() )
      return false;

    crc = mkc.Value();
    return true;
  }

  //! Attempts to approximate the passed curve with a circle.
  //! \param[in]  curve the input curve.
  //! \param[in]  tol   the fitting tolerance.
  //! \param[in]  c1    the first parameter on the input curve.
  //! \param[in]  c2    the second parameter on the input curve.
  //! \param[out] cf    the first parameter on the output curve.
  //! \param[out] cl    the last parameter on the output curve.
  //! \param[out] dev   the reached deviation.
  //! \return the output circle.
  template<typename TCurve,
           typename TCircle,
           typename TCirclePrim,
           typename TCircleBuilder,
           typename TPnt,
           typename TVec>
  static Handle(TCurve)
    fitCircle(const Handle(TCurve)& curve,
              const double          tol,
              const double          c1,
              const double          c2,
              double&               cf,
              double&               cl,
              double&               dev)
  {
    /*
     The idea of this function is to build a circle by three points,
     and then verify that the rest of the curve is in a good agreement
     with it.
    */

    if ( curve->IsKind( STANDARD_TYPE(TCircle) ) )
    {
      cf = c1;
      cl = c2;
      return curve;
    }

    Handle(TCircle) circ;

    // Get three test points on a circle.
    TPnt P0, P1, P2;
    double ca = (c1 + c1 + c2) / 3;
    double cb = (c1 + c2 + c2) / 3;
    //
    P0 = curve->Value(c1);
    P1 = curve->Value(ca);
    P2 = curve->Value(cb);

    // Fit circle by three test points.
    TCirclePrim circPrim;
    if ( !getCircle<TPnt, TVec, TCirclePrim, TCircleBuilder>(P0, P1, P2, circPrim) )
      return circ;

    cf = 0;

    // Now check if the constructed curve fits the remaining points
    // on the curve. For that, we take some extra 20 points.
    double du = (c2 - c1) / 20;
    double maxDist = 0.;
    //
    for ( int i = 0; i <= 20; ++i )
    {
      double u    = c1 + du*i;
      TPnt   PP   = curve->Value(u);
      double dist = circPrim.Distance(PP);
      //
      if ( dist > tol )
        return circ; // Not done.

      if ( dist > maxDist )
        maxDist = dist;
    }
    dev = maxDist; // Max distance from the curve points to the circle.

    /* Define parameters. */

    const double PI2 = 2.*M_PI;

    cf  = ElCLib::Parameter             ( circPrim, curve->Value(c1) );
    cf += ShapeAnalysis::AdjustToPeriod ( cf, 0., PI2 );

    // The first parameter should be close to zero.
    if ( Abs(cf) < Precision::PConfusion() || Abs(PI2 - cf) < Precision::PConfusion() )
      cf = 0.;

    double cm = ElCLib::Parameter             ( circPrim, curve->Value( (c1 + c2)/2.) );
    cm       += ShapeAnalysis::AdjustToPeriod ( cm, cf, cf + PI2 );
    cl        = ElCLib::Parameter             ( circPrim, curve->Value(c2) );
    cl       += ShapeAnalysis::AdjustToPeriod ( cl, cm, cm + PI2 );

    circ = new TCircle(circPrim);
    return circ;
  }

};

#endif
