#include "./FR_ConvertCanonicalCurve.h"

// FR includes
#include "./FR_RecognizeCanonical.h"

// OpenCascade includes
#include <ElCLib.hxx>
#include <gce_MakeCirc.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>

//-----------------------------------------------------------------------------

#define numSamplePts 20

//-----------------------------------------------------------------------------

Handle(Geom_Curve)
  FR_ConvertCanonicalCurve::ConvertCurve(const Handle(Geom_Curve)& curve,
                                              const double              tolerance,
                                              const double              c1,
                                              const double              c2,
                                              double&                   cf,
                                              double&                   cl)
{
  Handle(Geom_Curve) c3d, newc3d;

  cf  = c1;
  cl  = c2;
  c3d = curve;
  //
  if ( c3d.IsNull() )
    return newc3d;

  gp_Pnt P1 = c3d->Value( c1 );
  gp_Pnt P2 = c3d->Value( c2 );
  gp_Pnt P3 = c3d->Value( (c1 + c2)*0.5 );
  double dLine, dCirc;

  // Untrim.
  if ( c3d->IsKind( STANDARD_TYPE(Geom_TrimmedCurve) ) )
  {
    c3d = Handle(Geom_TrimmedCurve)::DownCast(c3d)->BasisCurve();
  }

  // Recognition in case of small curve.
  if( (P1.Distance(P2) < 2*tolerance) && (P1.Distance(P3) < 2*tolerance) )
  {
    newc3d = ConvertToCircle(c3d, tolerance, c1, c2, cf, cl, dCirc);

    // Petya: because paraemters of line overwrite parameters of circle.
    double lf, ll;
    Handle(Geom_Curve) line = ConvertToLine(c3d, tolerance, c1, c2, lf, ll, dLine);
    //
    if ( newc3d.IsNull() || dLine < dCirc )
    {
      newc3d = line;
      cf     = lf;
      cl     = ll;
    }
  }
  else
  {
    newc3d = ConvertToLine(c3d, tolerance, c1, c2, cf, cl, dLine);

    if ( newc3d.IsNull() )
      newc3d = ConvertToCircle(c3d, tolerance, c1, c2, cf, cl, dCirc);
  }

  return newc3d;
}

//-----------------------------------------------------------------------------

Handle(Geom_Curve)
  FR_ConvertCanonicalCurve::ConvertToCircle(const Handle(Geom_Curve)& curve,
                                                 const double              tol,
                                                 const double              c1,
                                                 const double              c2,
                                                 double&                   cf,
                                                 double&                   cl,
                                                 double&                   deviation)
{
  if ( curve->IsKind( STANDARD_TYPE(Geom_Circle) ) )
  {
    cf = c1;
    cl = c2;
    return curve;
  }

  Handle(Geom_Circle) circ;
  gp_Pnt P0, P1, P2;
  double ca = (c1 + c1 + c2) / 3;
  double cb = (c1 + c2 + c2) / 3;
  P0 = curve->Value(c1);
  P1 = curve->Value(ca);
  P2 = curve->Value(cb);

  // Check if the points are not aligned.
  double eps = 1.e-09; // angular resolution
  double d0  = P0.Distance(P1);
  double d1  = P0.Distance(P2);
  //
  if ( d0 < eps || d1 < eps )
    return circ;

  gp_Circ crc;
  if ( !ConstructCircle(P0, P1, P2, d0, d1, eps, crc) )
    return circ;

  cf = 0;

  // Verify if that's a circle by sampling 20 points.
  double du = (c2 - c1)/numSamplePts;
  int    i;
  double distMax = 0.;
  //
  for ( i = 0; i <= numSamplePts; i++ )
  {
    double u    = c1 + du*i;
    gp_Pnt PP   = curve->Value(u);
    double dist = crc.Distance(PP);

    if ( dist > tol )
      return circ; // not done

    if ( dist > distMax )
      distMax = dist;
  }
  //
  deviation = distMax;

  // Define the parameters.
  const double PI2 = 2*M_PI;
  //
  cf  = ElCLib::Parameter( crc, curve->Value(c1) );
  cf += ShapeAnalysis::AdjustToPeriod(cf, 0., PI2);
  //
  if( Abs(cf) < Precision::PConfusion() || Abs(PI2 - cf) < Precision::PConfusion() )
    cf = 0.;

  double cm = ElCLib::Parameter( crc, curve->Value( (c1 + c2)/2.) );
  //
  cm  += ShapeAnalysis::AdjustToPeriod(cm, cf, cf + PI2);
  cl   = ElCLib::Parameter( crc, curve->Value(c2) );
  cl  += ShapeAnalysis::AdjustToPeriod(cl, cm, cm + PI2);
  circ = new Geom_Circle(crc);

  return circ;
}

//-----------------------------------------------------------------------------

Handle(Geom_Line)
  FR_ConvertCanonicalCurve::ConvertToLine(const Handle(Geom_Curve)& curve,
                                               const double              tolerance,
                                               const double              c1,
                                               const double              c2,
                                               double&                   cf,
                                               double&                   cl,
                                               double&                   deviation)
{
  Handle(Geom_Line) line;
  if ( curve.IsNull() )
    return line;

  gp_Pnt P1 = curve->Value(c1);
  gp_Pnt P2 = curve->Value(c2);

  double dPreci = Precision::Confusion()*Precision::Confusion();
  //
  if ( P1.SquareDistance(P2) < dPreci )
    return line;

  cf = c1;
  cl = c2;

  /* B-spline curve. */
  Handle(Geom_BSplineCurve) bsc = Handle(Geom_BSplineCurve)::DownCast(curve);
  //
  if ( !bsc.IsNull() )
  {
    TColgp_Array1OfPnt poles( 1, bsc->NbPoles() );
    bsc->Poles(poles);
    //
    if ( !FR_RecognizeCanonical::IsLinear(poles, tolerance, deviation) )
      return line;

    gp_Lin lin = ConstructLine(P1, P2, c1, cf, cl);
    return new Geom_Line(lin);
  }

  /* Bezier curve. */
  Handle(Geom_BezierCurve) bzc = Handle(Geom_BezierCurve)::DownCast(curve);
  //
  if ( !bzc.IsNull() )
  {
    TColgp_Array1OfPnt poles( 1, bzc->NbPoles() );
    bzc->Poles(poles);

    if ( !FR_RecognizeCanonical::IsLinear(poles, tolerance, deviation) )
      return line;

    gp_Lin lin = ConstructLine(P1, P2, c1, cf, cl);
    return new Geom_Line(lin);
  }

  line = Handle(Geom_Line)::DownCast(curve);
  //
  if ( !line.IsNull() )
  {
    cf        = c1;
    cl        = c2;
    deviation = 0.;
  }

  return line;
}


//-----------------------------------------------------------------------------

gp_Lin FR_ConvertCanonicalCurve::ConstructLine(const gp_Pnt& P1,
                                                    const gp_Pnt& P2,
                                                    const double  c1,
                                                    double&       cf,
                                                    double&       cl)
{
  gp_Vec vec(P1,P2);
  gp_Dir dir(vec);
  gp_Lin lin(P1, dir);

  lin.SetLocation( ElCLib::Value(c1, lin) );
  cf = ElCLib::Parameter(lin, P1);
  cl = ElCLib::Parameter(lin, P2);

  return lin;
}

//-----------------------------------------------------------------------------

bool FR_ConvertCanonicalCurve::ConstructCircle(const gp_Pnt& P0,
                                                    const gp_Pnt& P1,
                                                    const gp_Pnt& P2,
                                                    const double  d0,
                                                    const double  d1,
                                                    const double  epsang,
                                                    gp_Circ&      circ)
{
  // Check if points are not aligned.
  gp_Vec p0p1(P0, P1);
  gp_Vec p0p2(P0, P2);
  //
  double ang = p0p1.CrossSquareMagnitude(p0p2);
  if ( ang < d0*d1*epsang )
    return false;

  // Building the circle.
  gce_MakeCirc mkc(P0, P1, P2);
  if ( !mkc.IsDone() )
    return false;
  //
  circ = mkc.Value();

  gp_Pnt PC  = circ.Location();
  gp_Ax2 axe = circ.Position();
  gp_Vec VX(PC, P0);

  axe.SetXDirection(VX);
  circ.SetPosition(axe);
  return true;
}

//-----------------------------------------------------------------------------

FR_ConvertCanonicalCurve::FR_ConvertCanonicalCurve(const Handle(Geom_Curve)& C)
{
  m_curve = C;
}

//-----------------------------------------------------------------------------

bool FR_ConvertCanonicalCurve::Perform(const double        tol,
                                            Handle(Geom_Curve)& resultCurve,
                                            const double        F,
                                            const double        L,
                                            double&             newF,
                                            double&             newL)
{
  if ( m_curve.IsNull() )
    return false;

  Handle(Geom_Curve) curve = m_curve;

  // Untrim.
  while ( curve->IsKind(STANDARD_TYPE(Geom_TrimmedCurve) ) )
  {
    curve = Handle(Geom_TrimmedCurve)::DownCast(curve)->BasisCurve();
  }

  // Check if the basic curve is one of the two allowed types:
  // B-spline or Bezier. If not, we do not perform any conversion.
  Handle(Geom_BSplineCurve)
    bsplCurve = Handle(Geom_BSplineCurve)::DownCast(curve);
  //
  if ( bsplCurve.IsNull() )
  {
    Handle(Geom_BezierCurve)
      bezierCurve = Handle(Geom_BezierCurve)::DownCast(curve);

    if ( bezierCurve.IsNull() )
      return false;
  }

  // Compute the canonical curve.
  Handle(Geom_Curve)
    C = ConvertCurve(curve, tol, F, L, newF, newL);
  //
  if ( C.IsNull() )
    return false;

  if ( C != m_curve )
    resultCurve = C;

  return true;
}
