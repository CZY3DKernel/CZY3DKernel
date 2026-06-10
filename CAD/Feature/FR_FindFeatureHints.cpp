#include "./FR_FindFeatureHints.h"

// FR includes
#include "./FR_CheckValidity.h"
#include "./FR_Utils.h"

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Lin2d.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>

//-----------------------------------------------------------------------------

FR_FindFeatureHints::FR_FindFeatureHints(const TopoDS_Face& face)
{
  m_face = face;
}

//-----------------------------------------------------------------------------

bool FR_FindFeatureHints::IsPuzzled()
{
    TopTools_IndexedMapOfShape wires;
  TopExp::MapShapes(m_face, TopAbs_WIRE, wires);
  //
  if ( wires.Extent() != 1 )
    return true;
  //
  TopoDS_Wire wire = TopoDS::Wire(wires.FindKey(1));

  // Check parametric portrait. The following check is simple for polygonal
  // domains. For the curved ones we have to apply something different
  const double prec              = FR_CheckValidity::MaxTolerance(m_face)*0.1;
  int          iter              = 0;
  int          nCorners          = 0;
  bool         isPolygonalDomain = true;
  gp_Dir2d prevDir;
  //
  for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next())
  {
    ++iter;

    // Take host line
    const TopoDS_Edge& edge = exp.Current();
    Handle(Geom2d_Line) line = this->edgeAsLine(edge);
    //
    if ( line.IsNull() )
    {
      isPolygonalDomain = false; // Not a line? No line, no check, man.
      break;
    }

    // Take traverse direction
    gp_Dir2d dir = line->Lin2d().Direction();
    if ( edge.Orientation() == TopAbs_REVERSED )
      dir.Reverse();

    // Check angles
    if ( iter > 1 )
    {
      const double angle = Abs( prevDir.Angle(dir) );
      if ( Abs(angle) > prec && Abs(angle - M_PI/2) > prec )
        return true;

      if ( Abs(angle - M_PI/2) < prec )
        ++nCorners;
    }

    prevDir = dir;
  }

  // For curved domains we have to apply a more sophisticated check. But
  // today we do not have it...
  if ( !isPolygonalDomain )
  {
    int nEdges = 0;
      for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next())
      ++nEdges;
    //
    if ( nEdges != 4 )
      return true;
  }
  else if ( nCorners != 3 )
    return true; // Polyline should not have more than 4 corners

  return false;
}

//-----------------------------------------------------------------------------

Handle(Geom2d_Line) FR_FindFeatureHints::edgeAsLine(const TopoDS_Edge& edge) const
{
  double f, l;
    Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(edge, m_face, f, l);
  //
    if (curve->IsKind(STANDARD_TYPE(Geom2d_TrimmedCurve)))
  {
        Handle(Geom2d_TrimmedCurve) tcurve = Handle(Geom2d_TrimmedCurve)::DownCast(curve);
    //
        if (!tcurve->BasisCurve()->IsInstance(STANDARD_TYPE(Geom2d_Line)))
      return NULL;

    curve = tcurve->BasisCurve();
  }
  Handle(Geom2d_Line) line = Handle(Geom2d_Line)::DownCast(curve);
  return line;
}
