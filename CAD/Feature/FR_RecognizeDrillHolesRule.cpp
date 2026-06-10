#include "./FR_RecognizeDrillHolesRule.h"

// FR includes
#include "./FR_FeatureAttrAngle.h"
#include "./FR_FindFeatureHints.h"
#include "./FR_RecognizeCanonical.h"
#include "./FR_Utils.h"

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <Precision.hxx>
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>

#undef DRAW_DEBUG
#if defined DRAW_DEBUG
  #pragma message("===== warning: DRAW_DEBUG is enabled")
#endif

#define DefaultLinPrec 0.1

namespace {

  bool IsConcaveConcentric(const int                  fid,
                           const FR_Feature&     nids,
                           const gp_Ax1&              axis,
                           const Handle(FR_AAG)& aag,
                           const double               linPrec)
  {
    bool hasSoughtEdges = false;
    TColStd_PackedMapOfInteger soughtEdges;

    // Check angle types.
    for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
    {
      const int nid = nit.Key();

      FR_AAG::t_arc arc(fid, nid);

      // Get the dihedral angle.
      Handle(FR_FeatureAttrAngle)
        DA = aag->ATTR_ARC<FR_FeatureAttrAngle>(arc);
      //
      if ( DA.IsNull() )
        continue;

      const FR_FeatureAngleType vexity = DA->GetAngleType();

      if ( FR_FeatureAngle::IsConcave(vexity) )
      {
        if ( !hasSoughtEdges ) hasSoughtEdges = true;

        // Collect found edges.
        Handle(FR_FeatureAttrAdjacency)
          AA = aag->ATTR_ARC<FR_FeatureAttrAdjacency>(arc);
        //
        if ( !AA.IsNull() )
        {
          soughtEdges.Unite( AA->GetEdgeIndices() );
        }
      }
    }

    if ( !hasSoughtEdges )
      return false;

    // Check concentricity.
    bool hasImpossibleContour = false;
    gp_Lin axLin(axis);
    //
    for ( TColStd_PackedMapOfInteger::Iterator eit(soughtEdges);
          eit.More(); eit.Next() )
    {
      const int          eid  = eit.Key();
      const TopoDS_Edge& edge = TopoDS::Edge( aag->RequestMapOfEdges()(eid) );

      double f, l;
      Handle(Geom_Curve) c3d = BRep_Tool::Curve(edge, f, l);
      //
      if ( c3d.IsNull() )
        continue;

      gp_Circ circ;
      if ( FR_Utils::IsCircular(c3d, circ) )
      {
        const gp_Pnt& circCenter = circ.Location();
        const double  d          = axLin.Distance(circCenter);

        if ( d > linPrec )
        {
          hasImpossibleContour = true;

#if defined DRAW_DEBUG
          plotter.DRAW_SHAPE(edge, Color_Red, 1., true, "eccentricEdge");
          plotter.DRAW_POINT(circCenter, Color_Red, "circCenter");
#endif
        }
      }
      else
      {
#if defined DRAW_DEBUG
        plotter.DRAW_SHAPE(edge, Color_Red, 1., true, "soughtEdge");
#endif

        // Any non-circular edge from the found subset is assumed to be impossible.
        hasImpossibleContour = true;
      }
    }

    return !hasImpossibleContour;
  }

  bool IsConcaveConcentric(const int                  fid,
                           const TopoDS_Wire&         wire,
                           const gp_Ax1&              axis,
                           const Handle(FR_AAG)& aag,
                           const double               linPrec)
  {
    // Get neighbor faces.
    FR_Feature nids;
    //
    for ( BRepTools_WireExplorer wexp(wire); wexp.More(); wexp.Next() )
    {
      const TopoDS_Edge& edge  = wexp.Current(); // Outer edge.
      FR_Feature    enids = aag->GetNeighborsThru(fid, edge);
      //
      nids.Unite(enids);
    }

    return IsConcaveConcentric(fid, nids, axis, aag, linPrec);
  }

  //! Looks for concave inner wires to invalidate non-machinable
  //! bottom faces of holes.
  bool HasConcaveInnerWire(const int                  fid,
                           const Handle(FR_AAG)& aag)
  {
    const TopoDS_Face& F = aag->GetFace(fid);

    TopoDS_Wire owire = FR_Utils::CacheOuterWire(fid, aag);

    // Iterate over the internal contours.
    for ( TopExp_Explorer fexp(F, TopAbs_WIRE); fexp.More(); fexp.Next() )
    {
      const TopoDS_Wire& wire = TopoDS::Wire( fexp.Current() );

      // Skip outer wire.
      if ( wire.IsPartner(owire) )
        continue;

      // Find neighbors over the inner edges.
      for ( BRepTools_WireExplorer wexp(wire); wexp.More(); wexp.Next() )
      {
        const TopoDS_Edge& edge  = wexp.Current(); // Outer edge.
        FR_Feature    enids = aag->GetNeighborsThru(fid, edge);

        for ( FR_Feature::Iterator nit(enids); nit.More(); nit.Next() )
        {
          const int nid = nit.Key();

          FR_AAG::t_arc arc(fid, nid);

          // Get the dihedral angle.
          Handle(FR_FeatureAttrAngle)
            DA = aag->ATTR_ARC<FR_FeatureAttrAngle>(arc);
          //
          if ( DA.IsNull() )
            continue;

          if ( FR_FeatureAngle::IsConcave( DA->GetAngleType() ) )
          {
            return true;
          }
        }
      }
    } // by inner contours.

    return false;
  }

}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHolesRule::recognize(TopTools_IndexedMapOfShape& featureFaces,
                                                TColStd_PackedMapOfInteger& featureIndices)
{
  // Extract suspected face and check if it is a cylinder. Even if a hole
  // is composed of other types of surfaces (e.g. conical), we assume that
  // a cylinder is something primary.
  gp_Ax1      ref_ax;
  double      suspected_ang_min = 0.0;
  double      suspected_ang_max = 0.0;
  double      suspected_radius  = 0.0;
  const int   suspected_face_id = m_it->GetFaceId();
  TopoDS_Face suspected_face    = m_it->GetGraph()->GetFace(suspected_face_id);

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

  // In soft mode we do not allow any imprints in the parametric portrait
  // of the suspected face. Any imprint is a signal that there is a feature
  // interaction in that local zone. This situation is, however, absolutely,
  // normal for hard features.

  bool isBoreFace = true;

  if ( !m_bHardMode && !this->isCylindrical(suspected_face_id,
                                            true, true,
                                            suspected_radius,
                                            suspected_ang_min,
                                            suspected_ang_max,
                                            ref_ax) )
  {
    isBoreFace = false;
  }

  if ( m_bHardMode && !this->isCylindrical(suspected_face_id,
                                           false, true,
                                           suspected_radius,
                                           suspected_ang_min,
                                           suspected_ang_max,
                                           ref_ax) )
  {
    isBoreFace = false;
  }

  // Last chance for this face is to be a sink-only hole.
  if ( !isBoreFace && m_bPureConicalOn )
  {
    isBoreFace = this->isConical(suspected_face_id,
                                 false,
                                 suspected_radius,
                                 suspected_ang_min,
                                 suspected_ang_max,
                                 ref_ax);
  }

  if ( !isBoreFace )
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
    // A hole has concave self-adjacency.
    if ( !FR_FeatureAngle::IsConcave( faceAngleAttr->GetAngleType() ) )
      return false;
  }

  // Check radius of the cylinder against the requested barrier value.
  if ( suspected_radius > m_fTargetRadius )
    return false;

  // Traverse other cylindrical neighbors as a cylindrical hole is very often
  // composed of several patches.
  FR_Feature cyls;
  int             nbSuspectedSupports = 0;
  double          sum_angle           = Abs(suspected_ang_max - suspected_ang_min);
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

  neighbors.Subtract(cyls);
  nbSuspectedSupports = neighbors.Extent();

  // In soft feature mode we expect to have two support faces only.
  if ( !m_bHardMode && nbSuspectedSupports != 2 )
    return false;

  // The hole should be complete (round).
  if ( (sum_angle < 2*M_PI) && (Abs(sum_angle - 2*M_PI) > m_fAngToler) )
    return false;

  // Traverse toroidal surfaces.
  // Such surfaces are not drillable feature, but the external calling
  // feature solving algorithm relies on both drilling and hole milling operations
  // to find the most optimal solution.
  FR_Feature radii;
  //
  this->visitNeighborToruses(suspected_face_id,
                             suspected_face_id,
                             suspected_radius,
                             ref_ax,
                             radii);
  //
  this->AddTraversed(radii);
  neighbors.Subtract(radii);

  nbSuspectedSupports = neighbors.Extent();

  // From all feature face neighbors keep only those satisfying the hard-coded
  // rules below. Notice that graph pattern matching is a more general
  // approach, comparing to what we are doing here. But here we can
  // conduct a precise analysis of host geometry, so we prefer rule-based
  // approach of feature identification.
  FR_Feature suspected_endings;
  //
  for ( FR_Feature::Iterator fit( this->JustTraversed() ); fit.More(); fit.Next() )
  {
    const int              feature_face_id        = fit.Key();
    const FR_Feature& feature_face_neighbors = m_it->GetGraph()->GetNeighbors(feature_face_id);

    // Each neighbor is suspected to be an ending now.
    for ( FR_Feature::Iterator nit(feature_face_neighbors); nit.More(); nit.Next() )
    {
      const int          neighbor_id   = nit.Key();
      const TopoDS_Face& neighbor_face = m_it->GetGraph()->GetFace(neighbor_id);
      //
      if ( FR_Utils::IsConical(neighbor_face) ||
           FR_Utils::IsPlanar(neighbor_face, false) ||
           FR_Utils::IsToroidal(neighbor_face) )
        suspected_endings.Add(neighbor_id);
    }
  }

  // Validate bottom faces: they should not contain any concave inner wires. Also here
  // we need to be sure that the checked "suspected ending" is actually a bottom face,
  // so we add additional test for the outer wire.
  for ( FR_Feature::Iterator sit(suspected_endings); sit.More(); sit.Next() )
  {
    const int sid = sit.Key();

    TopoDS_Wire owire = FR_Utils::CacheOuterWire( sid, m_it->GetGraph() );

    std::set<FR_FeatureAngleType>
      outerVexities = {FeatureAngleType_Concave,
                       FeatureAngleType_SmoothConcave};

    // Check if that's a bottom face and not a base face where a hole is inserted.
    if ( ::IsConcaveConcentric(sid,
                               owire,
                               ref_ax,
                               m_it->GetGraph(),
                               DefaultLinPrec) )
    {
      // Check for obstructions.
      if ( ::HasConcaveInnerWire( sid, m_it->GetGraph() ) )
      {
        return false;
      }
    }
  }

  // Now check that each suspected ending has neighbors which are feature
  // faces or other endings. If not (i.e. there is at least one neighbor
  // which is not enumerated neither in the list of feature faces, nor in the
  // list of other endings), then such a suspected item is rejected.
  for ( FR_Feature::Iterator eit(suspected_endings); eit.More(); eit.Next() )
  {
    const int              ending_id        = eit.Key();
    const TopoDS_Face&     ending_face      = m_it->GetGraph()->GetFace(ending_id);
    const FR_Feature& ending_neighbors = m_it->GetGraph()->GetNeighbors(ending_id);

    bool isRealEnding = true;
    for ( FR_Feature::Iterator enit(ending_neighbors); enit.More(); enit.Next() )
    {
      const int ending_neighbor_id = enit.Key();
      if ( !this->JustTraversed().Contains(ending_neighbor_id) && !suspected_endings.Contains(ending_neighbor_id) )
      {
        isRealEnding = false;
        break;
      }
    }

    if ( isRealEnding )
    {
      //-----------------------------------------------------------------------
      this->SetTraversed(ending_id);
      //-----------------------------------------------------------------------
    }
    else
    {
      gp_Ax1 end_ax;
      if ( FR_Utils::IsConical(ending_face, end_ax) ||
           FR_Utils::IsToroidal(ending_face, end_ax))
      {
        if (end_ax.IsCoaxial(ref_ax, m_fAngToler, m_fLinToler) ||
            end_ax.IsOpposite(ref_ax, m_fAngToler) )
        {
          // Co-axial but position of the axis can be different.
          const gp_Pnt& end_ax_P = end_ax.Location();
          const gp_Pnt& ref_ax_P  = ref_ax.Location();

#if defined DRAW_DEBUG
          this->GetPlotter().DRAW_POINT(end_ax_P, Color_Green);
          this->GetPlotter().DRAW_POINT(ref_ax_P,  Color_Red);
#endif

          const double axes_dist = end_ax_P.Distance(ref_ax_P);
          if ( axes_dist < m_fLinToler )
          {
            //-----------------------------------------------------------------
            this->SetTraversed(ending_id);
            //-----------------------------------------------------------------
          }
          else
          {
            gp_Ax1 sample_ax( ref_ax_P, gp_Vec( end_ax_P.XYZ() - ref_ax_P.XYZ() ) );
            //
            if ( sample_ax.IsParallel(ref_ax, m_fAngToler) )
            {
              //-----------------------------------------------------------------
              this->SetTraversed(ending_id);
              //-----------------------------------------------------------------
            }
          }
        }
      }
    }
  }

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

bool FR_RecognizeDrillHolesRule::isCylindrical(const int fid) const
{
  double radius;
  double angle_min;
  double angle_max;
  gp_Ax1 ax;

  return this->isCylindrical(fid, false, true, radius, angle_min, angle_max, ax);
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHolesRule::isCylindrical(const int  fid,
                                                    const bool checkNoHints,
                                                    const bool checkBore,
                                                    double&    radius,
                                                    double&    angle_min,
                                                    double&    angle_max,
                                                    gp_Ax1&    ax) const
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

    if ( surfaceAdt.GetType() == GeomAbs_BSplineSurface )
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
        const TopoDS_Vertex& V = TopoDS::Vertex( faceVertices(v) );
        gp_Pnt2d uv = BRep_Tool::Parameters(V, face);

        uMin = Min( uMin, uv.X() );
        uMax = Max( uMax, uv.X() );
        vMin = Min( vMin, uv.Y() );
        vMax = Max( vMax, uv.Y() );
      }
      //
      if ( Abs(uMin - uMax) < Precision::Confusion() )
        return false;
      //
      if ( Abs(vMin - vMax) < Precision::Confusion() )
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
      {
        return false;
      }

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

  if ( checkBore )
  {
    if ( !FR_Utils::IsInternal(face, 2*radius, ax) )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHolesRule::isConical(const int  fid,
                                                const bool checkNoHints,
                                                double&    radius,
                                                double&    angle_min,
                                                double&    angle_max,
                                                gp_Ax1&    ax) const
{
  const TopoDS_Face& face = m_it->GetGraph()->GetFace(fid);

  double hMin, hMax, rMin, rMax;
  bool isCone = FR_Utils::IsConical(face, ax, true,
                                         angle_min, angle_max,
                                         hMin, hMax,
                                         rMin, rMax);

  radius = rMin;

  if ( !isCone )
    return false;

  if ( checkNoHints )
  {
    FR_FindFeatureHints hint(face);
    //
    if ( hint.IsPuzzled() )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHolesRule::visitNeighborCylinders(const int        sid,
                                                             const int        fid,
                                                             const double     refRadius,
                                                             const gp_Ax1&    refAxis,
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
    if ( ( !m_bHardMode && this->isCylindrical(nid, true,  true, nr, angMin, angMax, ax) ) ||
         (  m_bHardMode && this->isCylindrical(nid, false, true, nr, angMin, angMax, ax) ) )
    {
      if ( Abs(nr - refRadius) > m_fLinToler )
      {
        continue; // If this neighbor cylinder is a patch of the primary hole's
                  // geometry, then we expect it to have the same radius.
      }

      // Another criterion is to have concave angle between patches. If the
      // angle is convex, then this hole is not a cavity, but a kind of a boss.
      Handle(FR_FeatureAttrAngle)
        attr = m_it->GetGraph()->ATTR_ARC<FR_FeatureAttrAngle>( FR_AAG::t_arc(fid, nid) );
      //
      if ( !FR_FeatureAngle::IsConcave( attr->GetAngleType() ) )
        continue; // Might be a cylindrical support, so let's go further.

      if ( ax.IsCoaxial  (refAxis, m_fAngToler, m_fLinToler) ||
           ax.IsOpposite (refAxis, m_fAngToler) )
      {
        // Co-axial but position of the axis can be different.
        const gp_Pnt& ax_P     = ax.Location();
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

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHolesRule::visitNeighborToruses(const int        sid,
                                                           const int        fid,
                                                           const double     refRadius,
                                                           const gp_Ax1&    refAxis,
                                                           FR_Feature& collected)
{
  FR_Feature nids = m_it->GetGraph()->GetNeighbors(fid);

  // Iterate over neighbors.
  for (FR_Feature::Iterator nit(nids); nit.More(); nit.Next())
  {
    const int nid = nit.Key();
    //
    if (collected.Contains(nid) || (nid == sid))
      continue;

    // Props.
    gp_Ax1 ax;
    double rmin = 0.0;
    double rmax = 0.0;

    // Check whether face is toridal and extract its props.
    const TopoDS_Face& face = m_it->GetGraph()->GetFace(nid);
    if (!FR_Utils::IsToroidal(face, rmin, rmax, ax))
    {
      continue;
    }

    if (Abs((rmax + rmin) - refRadius) > m_fLinToler)
    {
      continue; // If this neighbor torus is a patch of the primary hole's
                // geometry, then we expect it to have the same radius.
    }

    if (ax.Direction().IsParallel(refAxis.Direction(), m_fAngToler))
    {
      // Check whether torus location belongs the direction of cylinder.
      const gp_Pnt& ax_P = ax.Location();

#if defined DRAW_DEBUG
      const gp_Pnt& ref_ax_P = refAxis.Location();
      this->GetPlotter().DRAW_POINT(ax_P, Color_Green);
      this->GetPlotter().DRAW_POINT(ref_ax_P, Color_Red);
#endif

      //
      gp_Lin pln(refAxis);
      if (pln.Distance(ax_P) < m_fLinToler)
      {
        collected.Add(nid);
        this->visitNeighborToruses(sid,
                                   nid,
                                   refRadius,
                                   refAxis,
                                   collected);
      }
    }
  }
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHolesRule::isInternal(const int     fid,
                                                 const double  diameter,
                                                 const gp_Ax1& ax) const
{
  const TopoDS_Face& face = m_it->GetGraph()->GetFace(fid);

  double uMin, uMax, vMin, vMax;
  BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

  return this->isInternal(fid, diameter, (uMin + uMax)*0.5, (vMin + vMax)*0.5, ax);
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHolesRule::isInternal(const int     fid,
                                                 const double  diameter,
                                                 const double  u,
                                                 const double  v,
                                                 const gp_Ax1& ax) const
{
  const TopoDS_Face& face = m_it->GetGraph()->GetFace(fid);

  BRepAdaptor_Surface bas(face);
  BRepLProp_SLProps lprops( bas, u, v, 1, Precision::Confusion() );
  //
  if ( !lprops.IsNormalDefined() )
    return false;

  const gp_Pnt& cylPt   = lprops.Value();
  gp_Dir        cylNorm = lprops.Normal();
  //
  if ( face.Orientation() == TopAbs_REVERSED )
    cylNorm.Reverse();

  // Take a probe point along the normal.
  gp_Pnt normProbe = cylPt.XYZ() + cylNorm.XYZ()*diameter*0.05;

#if defined DRAW_DEBUG
  m_plotter.DRAW_POINT(cylPt, Color_Red, "cylPt");
  m_plotter.DRAW_POINT(normProbe, Color_Green, "normProbe");
#endif

  // Compute the distance to the axis.
  gp_Lin axisLin(ax);
  //
  const double probeDist = axisLin.Distance(normProbe);
  const double cylDist   = axisLin.Distance(cylPt);
  //
  if ( probeDist < cylDist )
  {
    return true;
  }

  return false;
}
