#include "./FR_RecognitionRuleShaft.h"

// FR includes
#include "./FR_FeatureAttrAngle.h"
#include "./FR_FindFeatureHints.h"
#include "./FR_RecognizeCanonical.h"

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Lin2d.hxx>
#include <Precision.hxx>
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>

#undef DRAW_DEBUG
#if defined DRAW_DEBUG
  #pragma message("===== warning: DRAW_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

bool FR_RecognitionRuleShaft::recognize(TopTools_IndexedMapOfShape& featureFaces,
                                             TColStd_PackedMapOfInteger& featureIndices)
{
  // Extract suspected face and check if it is a cylinder.
    gp_Ax1 ref_ax;
  double    suspected_ang_min = 0.0;
  double    suspected_ang_max = 0.0;
  double    suspected_radius  = 0.0;
  const int suspected_face_id = m_it->GetFaceId();

  //---------------------------------------------------------------------------
  this->SetTraversed(suspected_face_id);
  //---------------------------------------------------------------------------

  FR_Feature neighbors;
  //
  if ( !m_it->GetNeighbors(neighbors) )
    return false;

  if ( neighbors.Extent() == 1 )
  {
    // If there is just one neighbor, it should be a periodic surface.
    const int nid = neighbors.GetMinimalMapped();
    //
    if ( !FR_Utils::IsCylindrical( m_it->GetGraph()->GetFace(nid) ) )
      return false;
  }

  // Check cylindricity.
  if ( !this->isCylindrical(suspected_face_id,
                            false, true,
                            suspected_radius,
                            suspected_ang_min,
                            suspected_ang_max,
                            ref_ax) )
    return false;

  // Check if this face is attributed. Attribution is normally done for
  // closed periodical faces in order to mark their "self-adjacency". Such
  // self-adjacency situation is somewhat very typical for OpenCascade's
  // B-Rep, but it is not maintained in Joshi's AAG, so here is the extension.
  const Handle(FR_FeatureAttrAngle)&
    faceAngleAttr = m_it->GetGraph()->ATTR_NODE<FR_FeatureAttrAngle>(suspected_face_id);
  //
  if ( !faceAngleAttr.IsNull() )
  {
    // A shaft has convex self-adjacency.
    if ( !FR_FeatureAngle::IsConvex( faceAngleAttr->GetAngleType() ) )
      return false;
  }

  // Check radius of the cylinder against the requested barrier value.
  if ( suspected_radius > m_fTargetRadius )
    return false;

  // Traverse other cylindrical neighbors as a cylindrical shaft might be
  // composed of several patches.
  FR_Feature cyls;
  double          sum_angle = Abs(suspected_ang_max - suspected_ang_min);
  //
  this->visitNeighborCylinders(suspected_face_id,
                               suspected_face_id,
                               suspected_radius,
                               ref_ax,
                               sum_angle,
                               cyls);
  //-------------------------------------------------------------------
  this->AddTraversed(cyls);
  //-------------------------------------------------------------------

  // The shaft should be complete (round).
  if ( (sum_angle < 2*M_PI) && (Abs(sum_angle - 2*M_PI) > m_fAngToler) )
    return false;

  // Fill collection of feature faces.
  featureIndices.Unite( this->JustTraversed() );
  //
  for ( FR_Feature::Iterator mit( this->JustTraversed() ); mit.More(); mit.Next() )
    featureFaces.Add( m_it->GetGraph()->GetFace( mit.Key() ) );

  // Set radius.
  m_fRadius = suspected_radius;
  return true;
}

//-----------------------------------------------------------------------------

bool FR_RecognitionRuleShaft::isCylindrical(const int fid) const
{
  double radius;
  double angle_min;
  double angle_max;
  gp_Ax1 ax;

  return this->isCylindrical(fid, false, true, radius, angle_min, angle_max, ax);
}

//-----------------------------------------------------------------------------

bool FR_RecognitionRuleShaft::isCylindrical(const int  fid,
                                                 const bool checkNoHints,
                                                 const bool checkOuter,
                                                 double&    radius,
                                                 double&    angle_min,
                                                 double&    angle_max,
                                                 gp_Ax1& ax) const
{
    const TopoDS_Face& face = m_it->GetGraph()->GetFace(fid);

  bool isCyl = FR_Utils::IsCylindrical(face, radius, ax, angle_min, angle_max);
  //
  if ( !isCyl )
  {
      Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    // Avoid computing UV bounds for any surface type except for splines which
    // are potentially non-canonical cylinders.
      GeomAdaptor_Surface surfaceAdt(surface);

    if (surfaceAdt.GetType() == GeomAbs_BSplineSurface)
    {
      double uMin = DBL_MAX, uMax = -DBL_MAX, vMin = DBL_MAX, vMax = -DBL_MAX;
      double uMinRec, uMaxRec, vMinRec, vMaxRec;

      // Take the UV values cached in vertices to avoid the expensive
      // UV-bounds computation with BRepTools.
      TopTools_IndexedMapOfShape faceVertices;
      TopExp::MapShapes(face, TopAbs_VERTEX, faceVertices);
      //
      for ( int v = 1; v <= faceVertices.Extent(); ++v )
      {
          const TopoDS_Vertex& V = TopoDS::Vertex(faceVertices(v));
          gp_Pnt2d uv = BRep_Tool::Parameters(V, face);

        uMin = Min( uMin, uv.X() );
        uMax = Max( uMax, uv.X() );
        vMin = Min( vMin, uv.Y() );
        vMax = Max( vMax, uv.Y() );
      }
      //
      if (Abs(uMin - uMax) < Precision::Confusion())
        return false;
      //
      if (Abs(vMin - vMax) < Precision::Confusion())
        return false;

      // Give a shot to canonical recognition. This function will do nothing for
      // non-freeform types, such as planes, conical surfaces, etc. For splines,
      // it will attempt to recognize a cylinder with some extra geometric checks.
      gp_Cylinder cyl;
      if ( !FR_RecognizeCanonical::CheckIsCylindrical(surface,
                                                           uMin, uMax, vMin, vMax,
                                                           m_fCanRecPrec,
                                                           true, // Extract parametric ranges.
                                                           cyl,
                                                           uMinRec, uMaxRec, vMinRec, vMaxRec) )
        return false;

      // Get the props.
      radius    = cyl.Radius();
      angle_min = uMinRec;
      angle_max = uMaxRec;
      ax        = cyl.Axis();
    }
    else
    {
      return false;
    }
  }

  if ( checkNoHints )
  {
    FR_FindFeatureHints hint(face);
    //
    if ( hint.IsPuzzled() )
      return false;
  }

  if ( checkOuter )
  {
    if ( FR_Utils::IsInternal(face, 2*radius, ax) )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

void FR_RecognitionRuleShaft::visitNeighborCylinders(const int        sid,
                                                          const int        fid,
                                                          const double     refRadius,
                                                          const gp_Ax1& refAxis,
                                                          double&          sumAng,
                                                          FR_Feature& collected)
{
    const TopoDS_Face& face = m_it->GetGraph()->GetFace(fid);

  // For cylindrical faces, we know that the neighbors of interest share with
  // the starting face its vertical edges. There is no such a cue for the spline
  // faces (for example), so we give them the default treatment.
  FR_Feature nids;
  //
  if ( FR_Utils::IsCylindrical(face) )
  {
      TColStd_PackedMapOfInteger
      verticalEids = FR_Utils::GetVerticalEdges( fid, m_it->GetGraph(), 1.*M_PI/180. );

    nids = m_it->GetGraph()->GetNeighborsThru(fid, verticalEids);
  }
  else
  {
    nids = m_it->GetGraph()->GetNeighbors(fid);
  }

  // Visit neighbors.
  for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
  {
    const int nid = nit.Key();
    //
    if ( collected.Contains(nid) || (nid == sid) )
      continue;

    // Props.
    gp_Ax1 ax;
    double angMin = 0.0;
    double angMax = 0.0;
    double nr     = 0.0;

    // Check cylindricity.
    if ( this->isCylindrical(nid, false, true, nr, angMin, angMax, ax) )
    {
      if ( Abs(nr - refRadius) > m_fLinToler )
      {
        continue; // If this neighbor cylinder is a patch of the primary hole's
                  // geometry, then we expect it to have the same radius.
      }

      // Another criterion is to have convex angle between patches. If the
      // angle is concave, then this hole is not a shaft, but a kind of a hole.
      Handle(FR_FeatureAttrAngle)
        attr = m_it->GetGraph()->ATTR_ARC<FR_FeatureAttrAngle>( FR_AAG::t_arc(fid, nid) );
      //
      if ( !FR_FeatureAngle::IsConvex( attr->GetAngleType() ) )
        continue;

      if ( ax.IsCoaxial  (refAxis, m_fAngToler, m_fLinToler) ||
           ax.IsOpposite (refAxis, m_fAngToler) )
      {
        // Coaxial but position of the axis can be different.
          const gp_Pnt& ax_P = ax.Location();
        const gp_Pnt& ref_ax_P = refAxis.Location();
        //
        if ( ax_P.Distance(ref_ax_P) < m_fLinToler )
        {
          collected.Add(nid);

          sumAng += Abs(angMax - angMin);

          // Continue recursively.
          this->visitNeighborCylinders(sid, nid, refRadius, refAxis, sumAng, collected);
        }
      }
    }
  }
}
