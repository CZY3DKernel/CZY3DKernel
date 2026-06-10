#include "./FR_RecognizeCanonical.h"

#include <optional>

// FR includes
#include "./FR_ConvertCanonicalCurve.h"
#include "./FR_Utils.h"

// OpenCascade includes
#include <BRepTools.hxx>
#include <ElSLib.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_BezierCurve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomLib.hxx>
#include <GeomLProp_CLProps.hxx>
#include <GeomLProp_SLProps.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>

#undef DRAW_DEBUG
#if defined DRAW_DEBUG
  #pragma message("===== warning: DRAW_DEBUG is enabled")
#endif

//------------------------------------------------------------------------------

#define INF_LIMIT     100
#define MAGIC         21.
#define MAGIC_RATIO   15.
#define ELEVATION_TOL 0.666

//------------------------------------------------------------------------------

// Auxiliary functions.
namespace
{
  bool CheckSurfacePlanar(const TColgp_Array1OfPnt&   poles,
                          const double                toler,
                          const Handle(Geom_Surface)& surface,
                          gp_Pln&                     plane)
  {
    bool   IsPlan = false;
    bool   Essai  = true;
    double gx, gy, gz;
    int    nbPoles = poles.Length();
    gp_Pnt Bary;
    gp_Dir DX, DY;

    if ( nbPoles > 10 )
    {
      // Fast check.
      TColgp_Array1OfPnt Aux(1, 5);
      Aux(1) = poles(1);
      Aux(2) = poles(nbPoles/3);
      Aux(3) = poles(nbPoles/2);
      Aux(4) = poles(nbPoles/2 + nbPoles/3);
      Aux(5) = poles(nbPoles);

      GeomLib::Inertia(Aux, Bary, DX, DY, gx, gy, gz);
      Essai = (gz < toler);
    }

    if ( Essai )
    {
      // Complete check.
      GeomLib::Inertia(poles, Bary, DX, DY, gx, gy, gz);

      if ( (gz < toler) && (gy > toler) )
      {
        gp_Pnt P;
        gp_Vec DU, DV;
        double umin, umax, vmin, vmax;
        surface->Bounds(umin, umax, vmin, vmax);
        surface->D1( (umin+umax)/2, (vmin+vmax)/2, P, DU, DV );

        // We take DX as close as possible to DU.
        gp_Dir du(DU);
        double Angle1 = du.Angle(DX);
        double Angle2 = du.Angle(DY);
        if ( Angle1 > M_PI/2 ) Angle1 = M_PI - Angle1;
        if ( Angle2 > M_PI/2 ) Angle2 = M_PI - Angle2;
        if ( Angle2 < Angle1 )
        {
          du = DY; DY = DX; DX = du;
        }
        if ( DX.Angle(DU) > M_PI/2 ) DX.Reverse();
        if ( DY.Angle(DV) > M_PI/2 ) DY.Reverse();

        gp_Ax3 axe(Bary, DX^DY, DX);
        plane.SetPosition(axe);
        plane.SetLocation(Bary);
        IsPlan = true;
      }
    }

    return IsPlan;
  }

  bool CheckPointsOnPlane(const TColgp_Array1OfPnt& P,
                          const gp_Pln&             Plan,
                          const double              Tol)
  {
    int  ii;
    bool B = true;

    for ( ii = 1; ii <= P.Length() && B; ++ii )
      B = ( Plan.Distance(P(ii) ) < Tol);

    return B;
  }

  bool CheckCurveOnPlane(const Handle(Geom_Curve)& C,
                         const gp_Pln&             Plan,
                         const double              Tol)
  {
    bool B = true;
    int ii, Nb;
    GeomAdaptor_Curve AC(C);
    GeomAbs_CurveType Type = AC.GetType();
    Handle(TColgp_HArray1OfPnt) TabP;

    switch ( Type )
    {
      case GeomAbs_Line:
      {
        Nb = 2;
        break;
      }
      case GeomAbs_Circle:
      {
        Nb = 3;
        break;
      }
      case GeomAbs_Ellipse:
      case GeomAbs_Hyperbola:
      case GeomAbs_Parabola:
      {
        Nb = 5;
        break;
      }
      case GeomAbs_BezierCurve:
      {
        Nb = AC.NbPoles();
        Handle(Geom_BezierCurve) BZ = AC.Bezier();
        TabP = new TColgp_HArray1OfPnt( 1, AC.NbPoles() );
        //
        for ( ii = 1; ii <= Nb; ++ii )
          TabP->SetValue( ii, BZ->Pole(ii) );

        break;
      }
      case GeomAbs_BSplineCurve:
      {
        Nb = AC.NbPoles();
        Handle(Geom_BSplineCurve) BZ = AC.BSpline();
        TabP = new TColgp_HArray1OfPnt( 1, AC.NbPoles() );
        //
        for ( ii = 1; ii <= Nb; ++ii )
          TabP->SetValue( ii, BZ->Pole(ii) );

        break;
      }
      default:
      {
          Nb = 8 + 3 * AC.NbIntervals(GeomAbs_CN);
      }
    }

    // General case.
    if ( TabP.IsNull() )
    {
      double u, du, f, l, d;
      f = AC.FirstParameter();
      l = AC.LastParameter();
      du = (l - f) / (Nb - 1);

      // Sample curve.
      for ( ii = 1; ii <= Nb && B; ++ii )
      {
        u = (ii - 1)*du + f;
        d = Plan.Distance( C->Value(u) );
        B = (d < Tol);
      }
    }
    else
    {
      B = ::CheckPointsOnPlane(TabP->Array1(), Plan, Tol);
    }

    return B;
  }

  bool EvaluateCurvature(const Handle(Geom_Surface)& surf,
                         const double                angleRad,
                         const gp_Pnt2d&             UV,
                         double&                     k)
  {
    // Calculate differential properties.
    GeomLProp_SLProps Props(surf, UV.X(), UV.Y(), 2, 1e-7);
    //
    if ( !Props.IsCurvatureDefined() )
    {
#if defined COUT_DEBUG
      std::cout << "Error: curvature is not defined" << std::endl;
#endif
      return false;
    }

    // Get differential properties.
    const gp_Vec Xu  = Props.D1U();
    const gp_Vec Xv  = Props.D1V();
    const gp_Vec Xuu = Props.D2U();
    const gp_Vec Xuv = Props.DUV();
    const gp_Vec Xvv = Props.D2V();
    const gp_Vec n   = Props.Normal();

    // Coefficients of the FFF.
    const double E = Xu.Dot(Xu);
    const double F = Xu.Dot(Xv);
    const double G = Xv.Dot(Xv);

    // Coefficients of the SFF.
    const double L = n.Dot(Xuu);
    const double M = n.Dot(Xuv);
    const double N = n.Dot(Xvv);

    // Calculate curvature using the coefficients of both fundamental forms.
    if ( Abs(angleRad - M_PI*0.5) < 1.0e-5 )
    {
      k = N / G;
    }
    else
    {
      const double lambda = Tan(angleRad);
      k = (L + 2*M*lambda + N*lambda*lambda) / (E + 2*F*lambda + G*lambda*lambda);
    }

    return true;
  }

  double TrimInf(const double val,
                 const double limit = INF_LIMIT)
  {
    double ret_val = val;
    if ( Precision::IsPositiveInfinite(val) )
      ret_val = limit;
    else if ( Precision::IsNegativeInfinite(val) )
      ret_val = -limit;

    return ret_val;
  }
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCanonical::CheckIsPlanar(const Handle(Geom_Surface)& surface,
                                               const double                toler,
                                               gp_Pln&                     pln)
{
  /* Based on OpenCascade's GeomLib_IsPlanarSurface */

  bool   isPlane = false;
  gp_Pln plane;

  // Prepare adaptor to avoid tons of downcasting.
  GeomAdaptor_Surface surfaceAdt(surface);
  GeomAbs_SurfaceType surfaceType = surfaceAdt.GetType();

  switch ( surfaceType )
  {
    /* Trivial processing for planes */
    case GeomAbs_Plane:
    {
      isPlane = true;
      plane   = surfaceAdt.Plane();
      break;
    }

    /* Even more trivial processing for other types */
    case GeomAbs_Cylinder:
    case GeomAbs_Cone:
    case GeomAbs_Sphere:
    case GeomAbs_Torus:
    {
      isPlane = false;
      break;
    }

    /* Free-form geometries require additional analysis */
    case GeomAbs_BezierSurface:
    case GeomAbs_BSplineSurface:
    {
      int ii, jj, kk;
      int NbU = surfaceAdt.NbUPoles();
      int NbV = surfaceAdt.NbVPoles();
      TColgp_Array1OfPnt Poles(1, NbU*NbV);

      // Collect poles.
      if (surfaceType == GeomAbs_BezierSurface)
      {
          Handle(Geom_BezierSurface) BZ;
        BZ = surfaceAdt.Bezier();
        //
        for ( ii = 1, kk = 1; ii <= NbU; ++ii )
          for ( jj = 1; jj <= NbV; ++jj, ++kk )
            Poles(kk) = BZ->Pole(ii, jj);
      }
      else
      {
        Handle(Geom_BSplineSurface) BS;
        BS = surfaceAdt.BSpline();
        //
        for ( ii = 1, kk = 1; ii <= NbU; ++ii )
          for ( jj = 1; jj <= NbV; ++jj, ++kk )
            Poles(kk) = BS->Pole(ii, jj);
      }

      // Check control points to deduce whether the surface is a plane.
      isPlane = ::CheckSurfacePlanar(Poles, toler, surface, plane);
      break;
    }

    /* A surface of revolution can be planar in the case when a planar
       profile is revolved around its orthogonal axis */
    case GeomAbs_SurfaceOfRevolution:
    {
      bool Essai = true;
      gp_Pnt P;
      gp_Vec DU, DV, Dn;
      gp_Dir axisDir = surfaceAdt.AxeOfRevolution().Direction();

      // Get partial derivatives.
      double Umin, Umax, Vmin, Vmax;
      surface->Bounds(Umin, Umax, Vmin, Vmax);
      surface->D1( (Umin + Umax)/2, (Vmin + Vmax)/2, P, DU, DV );

      // If we hit an irregular point, let's try a small shift.
      if ( DU.Magnitude() <= gp::Resolution() ||
           DV.Magnitude() <= gp::Resolution() )
      {
        double NewU = (Umin + Umax)/2 + (Umax - Umin)*0.1;
        double NewV = (Vmin + Vmax)/2 + (Vmax - Vmin)*0.1;
        surface->D1( NewU, NewV, P, DU, DV );
      }

      // Compute the norm vector.
      Dn = DU^DV;
      if ( Dn.Magnitude() > Precision::Confusion() )
      {
        double angle = axisDir.Angle(Dn);
        if ( angle > M_PI/2 )
        {
          angle = M_PI - angle;
          axisDir.Reverse();
        }
        Essai = (angle < 0.1);
      }

      if ( Essai )
      {
        gp_Ax3 axe(P, axisDir);
        axe.SetXDirection(DU);
        plane.SetPosition(axe);
        plane.SetLocation(P);
        //
        Handle(Geom_Curve) C = surface->UIso(Umin);
        isPlane = ::CheckCurveOnPlane(C, plane, toler);
      }
      else
      {
        isPlane = false;
      }

      break;
    }

    /* Check for special case of extrusion */
    case GeomAbs_SurfaceOfExtrusion:
    {
      bool Essai = false;
      double norm;
      gp_Vec Du, Dv, Dn;
      gp_Pnt P;

      // Get partial derivatives.
      double Umin, Umax, Vmin, Vmax;
      surface->Bounds(Umin, Umax, Vmin, Vmax);
      surface->D1( (Umin + Umax)/2, (Vmin + Vmax)/2, P, Du, Dv );

      // If we hit an irregular point, let's try a small shift.
      if ( Du.Magnitude() <= gp::Resolution() ||
           Dv.Magnitude() <= gp::Resolution() )
      {
        double NewU = (Umin + Umax)/2 + (Umax - Umin)*0.1;
        double NewV = (Vmin + Vmax)/2 + (Vmax - Vmin)*0.1;
        surface->D1(NewU, NewV, P, Du, Dv);
      }

      // Compute the norm vector.
      Dn = Du^Dv;
      norm = Dn.Magnitude();
      //
      if ( norm > 1.e-15 )
      {
        Dn /= norm;
        double angmax = toler / (Vmax - Vmin);
        gp_Dir D(Dn);
        Essai = ( D.IsNormal(surfaceAdt.Direction(), angmax) );
      }
      if ( Essai )
      {
        gp_Ax3 axe(P, Dn, Du);
        plane.SetPosition(axe);
        plane.SetLocation(P);
        Handle(Geom_Curve) C = surface->VIso( (Vmin + Vmax)/2 );
        isPlane = ::CheckCurveOnPlane(C, plane, toler);
      }
      else
      {
        isPlane = false;
      }

      break;
    }

    /* General case: sample the surface */
    default:
    {
      int NbU, NbV, ii, jj, kk;
        NbU = 8 + 3 * surfaceAdt.NbUIntervals(GeomAbs_CN);
      NbV = 8 + 3 * surfaceAdt.NbVIntervals(GeomAbs_CN);
      double Umin, Umax, Vmin, Vmax, du, dv, U, V;
      surface->Bounds(Umin, Umax, Vmin, Vmax);
      du = (Umax - Umin)  /(NbU - 1);
      dv = (Vmax - Vmin) / (NbV - 1);
      TColgp_Array1OfPnt Pnts(1, NbU*NbV);
      for ( ii = 0, kk = 1; ii < NbU; ++ii )
      {
        U = Umin + du*ii;
        for ( jj = 0; jj < NbV; ++jj, ++kk )
        {
          V = Vmin + dv*jj;
          surface->D0( U, V, Pnts(kk) );
        }
      }

      isPlane = ::CheckSurfacePlanar(Pnts, toler, surface, plane);
    }
  } // [end] switch by surface type

  pln = plane; // Set the output argument.
  return isPlane;
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::CheckIsCylindrical(const Handle(Geom_Surface)& surface,
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
                                                 double&                     vMaxCyl)
{
  // Prepare adaptor to avoid tons of downcasting.
  GeomAdaptor_Surface surfaceAdt(surface);
  GeomAbs_SurfaceType surfaceType = surfaceAdt.GetType();

  if (surfaceType == GeomAbs_Cylinder)
  {
    cyl = surfaceAdt.Cylinder();
    return true;
  }

  // Do not consider irrelevant surface types.
  switch ( surfaceType )
  {
      case GeomAbs_Plane:
      case GeomAbs_Cone:
      case GeomAbs_Sphere:
      case GeomAbs_Torus:
    {
      return false;
    }
    default: break;
  }

  /* ============================
   *  Analyze shape by curvature.
   * ============================ */

  const int stepsNum = 10;

  double uMin, uMax, vMin, vMax;
  //
  uMin = ::TrimInf(uMinSurf);
  uMax = ::TrimInf(uMaxSurf);
  vMin = ::TrimInf(vMinSurf);
  vMax = ::TrimInf(vMaxSurf);

  const double uStep = (uMax - uMin) / stepsNum;
  const double vStep = (vMax - vMin) / stepsNum;
  std::vector<double> uValues, vValues;

  // Generate u-isos.
  double u     = uMin;
  bool   uStop = false;
  while ( !uStop )
  {
    if ( u > uMax )
    {
      u     = uMax;
      uStop = true;
    }
    //
    uValues.push_back(u);

    u += uStep;
  }

  // Generate v-isos.
  double v     = vMin;
  bool   vStop = false;
  while ( !vStop )
  {
    if ( v > vMax )
    {
      v     = vMax;
      vStop = true;
    }
    //
    vValues.push_back(v);

    v += vStep;
  }

  // Collect curvatures.
  std::vector<double> k1, k2;
  for ( auto uu : uValues )
  {
    for ( auto vv : vValues )
    {
      gp_Pnt2d uv(uu, vv);
      double kk[2] = {0., 0.};

      ::EvaluateCurvature(surface, 0.,      uv, kk[0]);
      ::EvaluateCurvature(surface, M_PI/2., uv, kk[1]);

      k1.push_back(kk[0]);
      k2.push_back(kk[1]);
    }
  }

  /* Now that we have two arrays of curvatures, we can check if
     all values are the same. One array should contain all zeroes.
     The second array should contain reciprocals of radius. */

  if ( k1.empty() || k2.empty() )
    return false;

  /* The curvature values should not be compared with the tolerance
     directly as the tolerance comes from the modeling space and
     has Euclidian distance nature. */

  const double numericZero = 1.e-6;
  const double numericInf  = 1.e5;

  // Check k1.
  const double k1_ref = k1[0];
  const double r1_ref = (Abs(k1_ref) < numericZero) ? numericInf : Abs(1./k1_ref);
  //
  for ( size_t ii = 1; ii < k1.size(); ++ii )
  {
    const double r_ii = (Abs(k1[ii]) < numericZero) ? numericInf : Abs(1./k1[ii]);

    if ( Abs(r_ii - r1_ref) > toler )
      return false;
  }

  // Check k2.
  const double k2_ref = k2[0];
  const double r2_ref = (Abs(k2_ref) < numericZero) ? numericInf : Abs(1./k2_ref);
  //
  for ( size_t ii = 1; ii < k2.size(); ++ii )
  {
    const double r_ii = (Abs(k2[ii]) < numericZero) ? numericInf : Abs(1./k2[ii]);

    if ( Abs(r_ii - r2_ref) > toler )
      return false;
  }

  /* ===============
   *  Extract props.
   * =============== */

  bool   isCurvedU  = false;
  double convRadius = 0.;

  if ( (Abs(k1_ref) < numericZero) && (Abs(k2_ref) > numericZero) )
  {
    isCurvedU  = false;
    convRadius = 1./Abs(k2_ref);
    cyl.SetRadius(convRadius);
  }
  else if ( (Abs(k1_ref) > numericZero) && (Abs(k2_ref) < numericZero) )
  {
    isCurvedU  = true;
    convRadius = 1./Abs(k1_ref);
    cyl.SetRadius(convRadius);
  }
  else
  {
    return false;
  }

  // Get the midpoint.
  gp_Pnt2d P2d( (uMin + uMax)*0.5, (vMin + vMax)*0.5 );

  // Evaluate the differential props.
  gp_Pnt P3d;
  gp_Vec D1u, D1v;
  surface->D1(P2d.X(), P2d.Y(), P3d, D1u, D1v);

  // Get isoline in the probe point.
  Handle(Geom_Curve) iso = isCurvedU ? surface->VIso( P2d.Y() )
                                     : surface->UIso( P2d.X() );

  // Get the curve's differential props.
  const double t_onCurve = ( iso->FirstParameter() + iso->LastParameter() )*0.5;
  //
  GeomLProp_CLProps CLProp(iso, 2, Precision::Confusion());
  CLProp.SetParameter(t_onCurve);
  //
  if ( !CLProp.IsTangentDefined() )
    return false; // Degeneracy.

  const double K_onCurve = CLProp.Curvature();
  //
  if ( Abs(K_onCurve) < Precision::Confusion() )
    return false; // No real curvature.

  // Calculate the center of curvature.
  gp_Pnt center_onCurve;
  CLProp.CentreOfCurvature(center_onCurve);

#if defined DRAW_DEBUG
  plotter.DRAW_CURVE(iso,            Color_Blue, "iso");
  plotter.DRAW_POINT(center_onCurve, Color_Red,  "C");
#endif

  // Axis.
  cyl.SetPosition( gp_Ax3(center_onCurve, isCurvedU ? D1v : D1u) );

  // Stop here if no parametric ranges are to be extracted.
  if ( !checkRanges )
    return true;

  gp_Pnt Pmin = surface->Value(uMin, vMin);
  gp_Pnt Pmax = surface->Value(uMax, vMax);

  ElSLib::Parameters(cyl, Pmin, uMinCyl, vMinCyl);
  ElSLib::Parameters(cyl, Pmax, uMaxCyl, vMaxCyl);

  if ( uMinCyl > M_PI )
    uMinCyl -= 2*M_PI;

  if ( uMaxCyl > M_PI )
    uMaxCyl -= 2*M_PI;

  /* Resolve possible ambiguity of projection on periodic surface
     by adjusting the min/max U values according the curve length
     they induce. */

  // Compute the spline curve's length by Gauss integration.
  const double originalLen = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(iso));

  // Compute the converted arc's length by angle and radius.
  const double convLen[2] = { Abs(uMaxCyl - uMinCyl)*convRadius,
                              Abs(2*M_PI - uMaxCyl + uMinCyl)*convRadius };
  //
  const double lenDev[2] = { Abs(originalLen - convLen[0]),
                             Abs(originalLen - convLen[1]) };

  if ( (lenDev[0] > toler) && (lenDev[1] > toler) )
    return false; // Length is not well approximated.

  // Check if the lengths are matching.
  if ( lenDev[1] < lenDev[0] )
  {
    const double newUMin = uMaxCyl;
    const double newUMax = 2*M_PI + uMinCyl;

    uMinCyl = newUMin;
    uMaxCyl = newUMax;
  }

#if defined DRAW_DEBUG
  plotter.DRAW_POINT     (ElSLib::Value(uMinCyl, vMinCyl, cyl),  Color_Red, "PminCyl");
  plotter.DRAW_POINT     (ElSLib::Value(uMaxCyl, vMaxCyl, cyl),  Color_Red, "PmaxCyl");
  plotter.DRAW_VECTOR_AT (center_onCurve, isCurvedU ? D1v : D1u, Color_Red, "Ax");
#endif

  return true;
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::CheckIsCylindrical(const Handle(Geom_BSplineSurface)& surface,
                                                 const double                       toler,
                                                 gp_Cylinder&                       cyl)
{
  if ( surface.IsNull() ) return false; // Contract check.

  double uMinRec,  uMaxRec,  vMinRec,  vMaxRec;
  double uMinSurf, uMaxSurf, vMinSurf, vMaxSurf;
  surface->Bounds(uMinSurf, uMaxSurf, vMinSurf, vMaxSurf);

  if ( CheckIsCylindrical(surface,
                          uMinSurf, uMaxSurf, vMinSurf, vMaxSurf,
                          toler,
                          false, // Do not extract ranges, just check the shape.
                          cyl,
                          uMinRec, uMaxRec, vMinRec, vMaxRec) )
  {
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::CheckIsTorusSphere(const Handle(Geom_Surface)& surf,
                                                 const Handle(Geom_Circle)&  circle,
                                                 const Handle(Geom_Circle)&  otherCircle,
                                                 const double                param1,
                                                 const double                param2,
                                                 const double                param1ToCrv,
                                                 const double                param2ToCrv,
                                                 const double                toler,
                                                 const double                isTryUMajor,
                                                 Handle(Geom_Surface)&       resSurf)
{
  double             cf, cl;
  Handle(Geom_Curve) isoCrv1;
  Handle(Geom_Curve) isoCrv2;

  // Initial radius.
  double R = circle->Circ().Radius();

  // Isolines.
  if ( isTryUMajor )
  {
    isoCrv1 = surf->VIso( param1 + ( (param2-param1)/3. ) );
    isoCrv2 = surf->VIso( param1 + ( (param2-param1)*2./3 ) );
  }
  else
  {
    isoCrv1 = surf->UIso( param1 + ( (param2-param1)/3. ) );
    isoCrv2 = surf->UIso( param1 + ( (param2-param1)*2./3 ) );
  }

  // Check if the isos are circles.
  Handle(Geom_Curve) crv1 = FR_ConvertCanonicalCurve::ConvertCurve(isoCrv1, toler, param1ToCrv, param2ToCrv, cf, cl);
  Handle(Geom_Curve) crv2 = FR_ConvertCanonicalCurve::ConvertCurve(isoCrv2, toler, param1ToCrv, param2ToCrv, cf, cl);
  //
  if ( crv1.IsNull() || crv2.IsNull() ||
      !crv1->IsKind(STANDARD_TYPE(Geom_Circle) ) ||
      !crv2->IsKind(STANDARD_TYPE(Geom_Circle) ) )
  {
    return false;
  }

  Handle(Geom_Circle) circle1 = Handle(Geom_Circle)::DownCast(crv1);
  Handle(Geom_Circle) circle2 = Handle(Geom_Circle)::DownCast(crv2);
  double              R1      = circle1->Circ().Radius();
  double              R2      = circle2->Circ().Radius();

  // Check radii.
  if ( (Abs(R - R1) > toler) || (Abs(R - R2) > toler) )
    return false; // The requested tolerance is not respected.

  // Get centers of the major radius.
  gp_Pnt P1  = circle ->Circ().Location();
  gp_Pnt P2  = circle1->Circ().Location();
  gp_Pnt P3  = circle2->Circ().Location();
  double eps = 1.e-09; // Angular resolution.
  double d0  = P1.Distance(P2);
  double d1  = P1.Distance(P3);

  if ( d0 < toler || d1 < toler )
  {
    // compute sphere
    gp_Dir MainDir = otherCircle->Circ().Axis().Direction();
    resSurf = new Geom_SphericalSurface(gp_Ax3(circle->Circ().Location(), MainDir), R);
    return true;
  }

  gp_Circ circ;
  if ( !FR_ConvertCanonicalCurve::ConstructCircle(P1, P2, P3, d0, d1, eps, circ) )
    return false;

  double majorR = circ.Radius();
  gp_Pnt center = circ.Location();
  gp_Dir dir( ( P1.XYZ() - center.XYZ() )^( P3.XYZ() - center.XYZ() ) );
  //
  resSurf = new Geom_ToroidalSurface(gp_Ax3(center, dir), majorR, R);
  return true;
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::CheckIsLinearExtrusion(const Handle(Geom_Surface)& surf,
                                                     const double                tol,
                                                     Handle(Geom_Line)&          straightIso,
                                                     Handle(Geom_Curve)&         profileIso)
{
  // Check for infinite bounds.
  double U1, U2, V1, V2;
  surf->Bounds(U1, U2, V1, V2);
  //
  if ( Precision::IsInfinite(U1) && Precision::IsInfinite(U2) )
  {
    U1 = -1.;
    U2 =  1.;
  }
  if ( Precision::IsInfinite(V1) && Precision::IsInfinite(V2) )
  {
    V1 = -1.;
    V2 =  1.;
  }

  // Convert middle isos to canonical representation.
  double VMid = 0.5*(V1 + V2);
  double UMid = 0.5*(U1 + U2);
  //
  Handle(Geom_Surface) TrSurf = new Geom_RectangularTrimmedSurface(surf, U1, U2, V1, V2);
  Handle(Geom_Curve)   UIso   = TrSurf->UIso(UMid);
  Handle(Geom_Curve)   VIso   = TrSurf->VIso(VMid);

  // Try converting isos.
  double cuf, cul, cvf, cvl;
  Handle(Geom_Curve)
    umidiso = FR_ConvertCanonicalCurve::ConvertCurve(UIso, tol, V1, V2, cuf, cul);
  //
  Handle(Geom_Curve)
    vmidiso = FR_ConvertCanonicalCurve::ConvertCurve(VIso, tol, U1, U2, cvf, cvl);

  // Check lengths.
  const double uIsoLen = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(UIso), tol);
  const double vIsoLen = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(VIso), tol);
  //
  if ( uIsoLen > MAGIC_RATIO*vIsoLen || vIsoLen > MAGIC_RATIO*uIsoLen )
  {
    /* Suspicious surface: much longer in one parametric dimension compared to another. */
      
    // Get the direction of extrusion.
    std::optional<gp_Ax1> refAx;
    std::optional<gp_Pnt> refPos; // Reference position on the sweeping axis.
    bool                  straightIsoU = true;
    //
    if ( !umidiso.IsNull() && umidiso->IsKind( STANDARD_TYPE(Geom_Line) ) )
    {
      gp_Dir refDir = Handle(Geom_Line)::DownCast(umidiso)->Lin().Direction();

      refPos       = UIso->Value( ( UIso->FirstParameter() + UIso->LastParameter() )*0.5 );
      straightIsoU = true;

      refAx = gp_Ax1(*refPos, refDir);
    }
    else if ( !vmidiso.IsNull() && vmidiso->IsKind( STANDARD_TYPE(Geom_Line) ) )
    {
      gp_Dir refDir = Handle(Geom_Line)::DownCast(vmidiso)->Lin().Direction();

      refPos       = VIso->Value( ( VIso->FirstParameter() + VIso->LastParameter() )*0.5 );
      straightIsoU = false;

      refAx = gp_Ax1(*refPos, refDir);

    }
    //
    if ( !refAx.has_value() || !refPos.has_value() )
      return false;

    // Reference projection parameter to check elevation.
    const double refDot = 0.;

    const double P1 = straightIsoU ? U1 : V1;
    const double P2 = straightIsoU ? U2 : V2;

    // Add probe points in the curved parametric direction.
    const double tStep = (P2 - P1) / MAGIC;
    //
    std::vector<double> T;
    {
      double t     = P1;
      bool   tStop = false;
      //
      while ( !tStop )
      {
        if ( (t > P2) || Abs(t - P2) < 1e-6 )
        {
          t     = P2;
          tStop = true;
        }

        T.push_back(t);
        t += tStep;
      }
    }

    for ( const auto& t : T )
    {
      Handle(Geom_Curve) testIso = straightIsoU ? TrSurf->UIso(t) : TrSurf->VIso(t);
      gp_Pnt             testPos = testIso->Value( ( testIso->FirstParameter() + testIso->LastParameter() )*0.5 );

      // Test projection to check elevation.
      const double
        testDot = ( testPos.XYZ() - refAx->Location().XYZ() ).Dot( refAx->Direction().XYZ() );

      if ( Abs(testDot - refDot) > ELEVATION_TOL )
        return false;
    }
  }

  /* CASE 1: U iso is a straight line */
  if ( !umidiso.IsNull() && umidiso->IsKind( STANDARD_TYPE(Geom_Line) ) )
  {
    straightIso = Handle(Geom_Line)::DownCast(umidiso);

    const gp_Dir& refDir = straightIso->Lin().Direction();

    // Add more isolines in this parametric direction.
    const double uStep = (U2 - U1) / MAGIC;
    //
    std::vector<double> U;
    {
      double u     = U1;
      bool   uStop = false;
      //
      while ( !uStop )
      {
        if ( (u > U2) || Abs(u - U2) < 1e-6 )
        {
          u     = U2;
          uStop = true;
        }

        U.push_back(u);
        u += uStep;
      }
    }

    // All these isos should be straight and parallel to the reference one.
    for ( const auto& u : U )
    {
      Handle(Geom_Curve) probeIso = TrSurf->UIso(u);

      // Try converting.
      double probeF, probeL;
      //
      Handle(Geom_Curve)
        probeIsoConverted = FR_ConvertCanonicalCurve::ConvertCurve(probeIso, tol, V1, V2, probeF, probeL);
      //
      if ( probeIsoConverted.IsNull() || !probeIsoConverted->IsKind( STANDARD_TYPE(Geom_Line) ) )
        return false;

      Handle(Geom_Line)
        probeIsoLine = Handle(Geom_Line)::DownCast(probeIsoConverted);

      const gp_Dir& nextDir = probeIsoLine->Lin().Direction();

      if ( !nextDir.IsParallel(refDir, 1.*M_PI/180.) )
        return false;
    }

    profileIso = VIso;
    return true;
  }

  /* CASE 2: V iso is a straight line */
  if ( !vmidiso.IsNull() && vmidiso->IsKind( STANDARD_TYPE(Geom_Line) ) )
  {
    straightIso = Handle(Geom_Line)::DownCast(vmidiso);

    const gp_Dir& refDir = straightIso->Lin().Direction();

    // Add more isolines in this parametric direction.
    const double vStep = (V2 - V1) / MAGIC;
    //
    std::vector<double> V;
    {
      double v     = V1;
      bool   vStop = false;
      //
      while ( !vStop )
      {
        if ( (v > V2) || Abs(v - V2) < 1e-6 )
        {
          v     = V2;
          vStop = true;
        }

        V.push_back(v);
        v += vStep;
      }
    }

    // All these isos should be straight and parallel to the reference one.
    for ( const auto& v : V )
    {
      Handle(Geom_Curve) probeIso = TrSurf->VIso(v);

      // Try converting.
      double probeF, probeL;
      //
      Handle(Geom_Curve)
        probeIsoConverted = FR_ConvertCanonicalCurve::ConvertCurve(probeIso, tol, U1, U2, probeF, probeL);
      //
      if ( probeIsoConverted.IsNull() || !probeIsoConverted->IsKind( STANDARD_TYPE(Geom_Line) ) )
        return false;

      Handle(Geom_Line)
        probeIsoLine = Handle(Geom_Line)::DownCast(probeIsoConverted);

      const gp_Dir& nextDir = probeIsoLine->Lin().Direction();

      if ( !nextDir.IsParallel(refDir, 1.*M_PI/180.) )
        return false;
    }

    profileIso = UIso;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::CheckIsLinearExtrusion(const Handle(Geom_Surface)& surf,
                                                     const double                tol)
{
  Handle(Geom_Line)  straightIso;
  Handle(Geom_Curve) profileIso;
  //
  return CheckIsLinearExtrusion(surf, tol, straightIso, profileIso);
}

//-----------------------------------------------------------------------------

Handle(Standard_Type)
  FR_RecognizeCanonical::CheckType(const Handle(Geom_Surface)& surface,
                                        const double                uMinSurf,
                                        const double                uMaxSurf,
                                        const double                vMinSurf,
                                        const double                vMaxSurf,
                                        const double                toler,
                                        double&                     uMinRec,
                                        double&                     uMaxRec,
                                        double&                     vMinRec,
                                        double&                     vMaxRec)
{
  if ( surface.IsNull() ) return nullptr; // Contract check.

  // Check plane.
  {
    gp_Pln pln;
    //
    if ( CheckIsPlanar(surface, toler, pln) )
    {
      return STANDARD_TYPE(Geom_Plane);
    }
  }

  // Check cylinder.
  {
    gp_Cylinder cyl;
    //
    if ( CheckIsCylindrical(surface,
                            uMinSurf, uMaxSurf, vMinSurf, vMaxSurf,
                            toler, true, cyl,
                            uMinRec, uMaxRec, vMinRec, vMaxRec) )
    {
      return STANDARD_TYPE(Geom_CylindricalSurface);
    }
  }

  // Check linear extrusion.
  {
    Handle(Geom_Line)  straightIso;
    Handle(Geom_Curve) profileIso;
    //
    if ( CheckIsLinearExtrusion(surface,
                                toler,
                                straightIso,
                                profileIso) )
    {
        return STANDARD_TYPE(Geom_SurfaceOfLinearExtrusion);
    }
  }

  // ... add more recognized types here.

  return surface->DynamicType();
}

//-----------------------------------------------------------------------------

Handle(Standard_Type)
  FR_RecognizeCanonical::CheckType(const TopoDS_Face&   face,
                                        const double         toler)
{
  double uMin, uMax, vMin, vMax;
  double uMinRec, uMaxRec, vMinRec, vMaxRec;

  BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

  return CheckType(BRep_Tool::Surface(face),
                   uMin, uMax, vMin, vMax,
                   toler,
                   uMinRec, uMaxRec, vMinRec, vMaxRec);
}

//-----------------------------------------------------------------------------

bool
  FR_RecognizeCanonical::IsLinear(const TColgp_Array1OfPnt2d& pts,
                                       const double                toler,
                                       double&                     dev,
                                       gp_Lin2d&                   lin)
{
  const int nbPoles = pts.Length();
  //
  if ( nbPoles < 2 )
    return false;

  /* ========================
   *  Construct fitting line.
   * ======================== */

  double dMax = 0;
  int    iMax1 = 0, iMax2 = 0;
  int    i;

  for ( i = 1; i < nbPoles; ++i )
  {
    for ( int j = i + 1; j <= nbPoles; ++j )
    {
      const double dist = pts(i).SquareDistance( pts(j) );
      //
      if ( dist > dMax )
      {
        dMax  = dist;
        iMax1 = i;
        iMax2 = j;
      }
    }
  }

  double dPreci = Precision::Confusion()*Precision::Confusion();
  //
  if ( dMax < dPreci )
    return false;

  /* =========================================
   *  Test fitting line w.r.t. the input data.
   * ========================================= */

  // Prepare test line.
  gp_Vec2d vec(pts(iMax1), pts(iMax2));
  gp_Dir2d dir(vec);

  lin = gp_Lin2d( pts(iMax1), dir );

  const double tol2   = toler*toler;
  double       devMax = 0.;
  //
  for ( i = 1; i <= nbPoles; ++i )
  {
    const double
      dist = lin.SquareDistance( pts(i) );

    if ( dist > tol2 )
      return false; // Bad accuracy of approximation.

    if ( dist > devMax )
      devMax = dist;
  }

  // Return the reached deviation.
  dev = sqrt(devMax);
  return true;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCanonical::IsLinear(const TColgp_Array1OfPnt& pts,
                                          const double              toler,
                                          double&                   dev)
{
  gp_Lin lin;
  return FR_RecognizeCanonical::IsLinear(pts, toler, dev, lin);
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCanonical::IsLinear(const TColgp_Array1OfPnt& pts,
                                          const double              toler,
                                          double&                   dev,
                                          gp_Lin&                   lin)
{
  const int nbPoles = pts.Length();
  //
  if ( nbPoles < 2 )
    return false;

  double dMax = 0;
  int    iMax1 = 0, iMax2 = 0;
  int    i;

  for ( i = 1; i < nbPoles; ++i )
  {
    for ( int j = i + 1; j <= nbPoles; ++j )
    {
      const double dist = pts(i).SquareDistance( pts(j) );

      if ( dist > dMax )
      {
        dMax  = dist;
        iMax1 = i;
        iMax2 = j;
      }
    }
  }

  double dPreci = Precision::Confusion()*Precision::Confusion();
  //
  if ( dMax < dPreci )
    return false;

  double tol2 = toler*toler;
  gp_Vec avec( pts(iMax1), pts(iMax2) );
  gp_Dir adir( avec );
  gp_Lin alin( pts(iMax1), adir );

  double maxDist = 0.;
  for ( i = 1; i <= nbPoles; ++i )
  {
    double dist = alin.SquareDistance(pts(i));
    if ( dist > tol2 )
      return false;

    if ( dist > maxDist )
      maxDist = dist;
  }
  dev = sqrt(maxDist);

  lin = alin;
  return true;
}

//-----------------------------------------------------------------------------

Handle(Geom_Curve)
  FR_RecognizeCanonical::FitCircle(const Handle(Geom_Curve)& curve,
                                        const double              tol,
                                        const double              c1,
                                        const double              c2,
                                        double&                   cf,
                                        double&                   cl,
                                        double&                   dev)
{
  return fitCircle<Geom_Curve,
                   Geom_Circle,
                   gp_Circ,
                   gce_MakeCirc,
                   gp_Pnt,
                   gp_Vec>(curve, tol, c1, c2, cf, cl, dev);
}

//-----------------------------------------------------------------------------

Handle(Geom2d_Curve)
  FR_RecognizeCanonical::FitCircle(const Handle(Geom2d_Curve)& curve,
                                        const double                tol,
                                        const double                c1,
                                        const double                c2,
                                        double&                     cf,
                                        double&                     cl,
                                        double&                     dev)
{
  return fitCircle<Geom2d_Curve, Geom2d_Circle, gp_Circ2d, gce_MakeCirc2d,
                   gp_Pnt2d, gp_Vec2d>(curve, tol, c1, c2, cf, cl, dev);
}
