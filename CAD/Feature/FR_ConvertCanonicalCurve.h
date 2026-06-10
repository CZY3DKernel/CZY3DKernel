
#ifndef FR_ConvertCanonicalCurve_HeaderFile
#define FR_ConvertCanonicalCurve_HeaderFile

// FR includes
#include "./FR.h"

// OpenCascade includes
#include <TColgp_Array1OfPnt.hxx>

#include "../Common/Extension_Export.hxx"

class Geom_Curve;
class Geom_Line;
class gp_Lin;
class gp_Pnt;
class gp_Circ;

//-----------------------------------------------------------------------------

//! \ingroup ASI_MODELING
//!
//! Converts the input curve to a canonical form.
class FR_ConvertCanonicalCurve
{
public:

  //! Attempts to convert this curve to a canonical form.
  EXTENSION_EXPORT static Handle(Geom_Curve)
    ConvertCurve(const Handle(Geom_Curve)& curve,
                 const double              tol,
                 const double              c1,
                 const double              c2,
                 double&                   cf,
                 double&                   cl);

  //! Attempts to convert the passed curve to a circle.
  EXTENSION_EXPORT static Handle(Geom_Curve)
    ConvertToCircle(const Handle(Geom_Curve)& curve,
                    const double              tol,
                    const double              c1,
                    const double              c2,
                    double&                   cf,
                    double&                   cl,
                    double&                   deviation);

  //! Attempts to convert the passed curve to a straight line.
  EXTENSION_EXPORT static Handle(Geom_Line)
    ConvertToLine(const Handle(Geom_Curve)& curve,
                  const double              tolerance,
                  const double              c1,
                  const double              c2,
                  double&                   cf,
                  double&                   cl,
                  double&                   deviation);

  //! Constructs a line from the given two points and a parameter where
  //! to set the origin point. The parameters `cf` and `cl` will contain
  //! the parameters of `P1` and `P2` points on the constructed line.
  EXTENSION_EXPORT static gp_Lin
    ConstructLine(const gp_Pnt& P1,
                  const gp_Pnt& P2,
                  const double  c1,
                  double&       cf,
                  double&       cl);

  //! Constructs a circle for the given three points.
  EXTENSION_EXPORT static bool
    ConstructCircle(const gp_Pnt& P0,
                    const gp_Pnt& P1,
                    const gp_Pnt& P2,
                    const double  d0,
                    const double  d1,
                    const double  eps,
                    gp_Circ&      circ);

public:

  //! Complete ctor.
  EXTENSION_EXPORT
    FR_ConvertCanonicalCurve(const Handle(Geom_Curve)& C);

public:

  //! Attempts to convert this curve to a canonical form.
  EXTENSION_EXPORT bool
    Perform(const double        tol,
            Handle(Geom_Curve)& resultCurve,
            const double        F,
            const double        L,
            double&             newF,
            double&             newL);

private:

  Handle(Geom_Curve) m_curve; //!< The curve to process.

};

#endif
