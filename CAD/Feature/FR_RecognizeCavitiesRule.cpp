#include "./FR_RecognizeCavitiesRule.h"

// Analysis Situs includes
#include "./FR_FeatureAttrAdjacency.h"
#include "./FR_FeatureAttrAngle.h"

// OpenCascade includes
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

#undef DRAW_DEBUG
#if defined DRAW_DEBUG
  #pragma message("===== warning: DRAW_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

// Defines maximum number of elements in the feature.
const static int MaxNumOfFeatureFaces = 100;

namespace
{
  //! Checks if the passed `edges` all belongs to the outer wire.
  bool AreOnOuterWire(const TopTools_IndexedMapOfShape& edges,
                      const TopoDS_Wire&                outerWire)
  {
    TopTools_IndexedMapOfShape edgesOW; // Edges on outer wire.
    TopExp::MapShapes(outerWire, TopAbs_EDGE, edgesOW);

    // Check that all edges are on outer wire of neighbor.
    bool isOnOuter = true;
    //
    for ( TopTools_IndexedMapOfShape::Iterator eit(edges); eit.More(); eit.Next() )
    {
      if ( !edgesOW.Contains( eit.Value() ) )
      {
        isOnOuter = false;
        break;
      }
    }
    //
    if ( isOnOuter )
    {
      return true;
    }

    return false;
  }
}

//-----------------------------------------------------------------------------

FR_RecognizeCavitiesRule::FR_RecognizeCavitiesRule(const Handle(FR_AAGIterator)& it,
                                                             const double                       maxSize)
//
: FR_RecognitionRule (it),
  m_fMaxSize              (maxSize)
{
  this->init();
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCavitiesRule::recognize(TopTools_IndexedMapOfShape& featureFaces,
                                              FR_Feature&            featureIndices)
{
  // Get seed face and iterate over its internal wires.
  const int          sid      = m_it->GetFaceId();
  const TopoDS_Face& seedFace = m_it->GetGraph()->GetFace(sid);

  // Skip already recognized faces.
  if ( featureIndices.Contains(sid) )
    return true;

  const TopoDS_Wire* wirePtr   = m_mapFaceOuterWire.Seek(seedFace);
  const TopoDS_Wire  outerWire = ( wirePtr ) ? (*wirePtr)
                                             : FR_Utils::CacheOuterWire( sid, m_it->GetGraph() );

  // Explore inner wires.
  for ( TopExp_Explorer wit(seedFace, TopAbs_WIRE); wit.More(); wit.Next() )
  {
    const TopoDS_Wire& seedWire = TopoDS::Wire( wit.Current() );

    // Skip outer wire.
    if ( seedWire.IsPartner(outerWire) )
      continue;

    /* Here we work with inner wires only */

    if ( !this->isConvex(sid, seedWire) )
      continue;

    TopTools_IndexedMapOfShape foundFaces;
    FR_Feature            foundIds;

    /* Try to complete the candidate cavity feature by recursive propagation */
    {
      TopExp_Explorer eexp(seedWire, TopAbs_EDGE);
      const TopoDS_Edge& anyEdge = TopoDS::Edge( eexp.Current() );

      const FR_Feature&
        nids = m_it->GetGraph()->GetNeighborsThru(sid, anyEdge);

      bool isOk = true;
      this->propagate(sid, nids, foundFaces, foundIds, isOk);

      if ( !isOk )
        continue; // Finishing face does not have convex adjacency on the inner wire.
    }

    // Check that feature is found.
    if ( (foundIds.Extent() > MaxNumOfFeatureFaces) || !foundIds.Extent() )
    {
      continue; // Protection from incomplete and incorrect features.
    }

    // Check that we do not detect the full solid as a "feature".
    if ( !this->isNotEntireShape(foundFaces, foundIds) )
      continue;

    // Check feature size.
    if ( !this->isSizeOk(foundFaces) )
      continue;

    // Compose the result.
    if ( foundFaces.Size() )
    {
      for ( TopTools_IndexedMapOfShape::Iterator fit(foundFaces); fit.More(); fit.Next() )
        featureFaces.Add( fit.Value() );

      for ( FR_Feature::Iterator fit(foundIds); fit.More(); fit.Next())
        featureIndices.Add( fit.Key() );
    }
  }

  return true;
}

//-----------------------------------------------------------------------------

void FR_RecognizeCavitiesRule::init()
{
  // Fill map of faces vs their outer wires.
  Handle(FR_AAGRandomIterator)
    it = new FR_AAGRandomIterator( m_it->GetGraph() );
  //
  for ( ; it->More(); it->Next() )
  {
    const int          faceId    = it->GetFaceId();
    const TopoDS_Face& face      = m_it->GetGraph()->GetFace(faceId);
    TopoDS_Wire        outerWire = FR_Utils::CacheOuterWire( faceId, m_it->GetGraph() );

    m_mapFaceOuterWire.Bind(face, outerWire);
  }
}

//-----------------------------------------------------------------------------

void FR_RecognizeCavitiesRule::propagate(const int                   startId,
                                              const FR_Feature&      seedIds,
                                              TopTools_IndexedMapOfShape& featureFaces,
                                              FR_Feature&            featureIndices,
                                              bool&                       isOk)
{
  FR_Feature nextIterIds;

  for ( FR_Feature::Iterator sit(seedIds); sit.More(); sit.Next() )
  {
    const int fid = sit.Key();

    featureFaces  .Add( m_it->GetGraph()->GetFace(fid) );
    featureIndices.Add(fid);

    // Iterate over neighbors and find new candidates for the recursive call.
    const FR_Feature&
      nids = m_it->GetGraph()->GetNeighbors(fid);
    //
    for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
    {
      const int          nid   = nit.Key();
      const TopoDS_Face& nFace = m_it->GetGraph()->GetFace(nid);

      FR_AAG::t_arc arc(fid, nid);

      // Connecting edges.
      Handle(FR_FeatureAttrAdjacency)
        adjacencyEdges = m_it->GetGraph()->ATTR_ARC<FR_FeatureAttrAdjacency>(arc);
      //
      TopTools_IndexedMapOfShape edges;
      adjacencyEdges->GetEdges(edges);

      // Check that we are not getting back to the seed face via its outer wire.
      // Such a transition would mean that our feature badly propagates over the
      // surface of the model, e.g. like in `recognize-cavities/A34.tcl` for
      // the `start` face 1 and `fid` 22.
      if ( nid == startId )
      {
        const TopoDS_Wire* wirePtr   = m_mapFaceOuterWire.Seek(nFace);
        const TopoDS_Wire  outerWire = ( wirePtr ) ? (*wirePtr)
                                                   : FR_Utils::CacheOuterWire( nid, m_it->GetGraph() );
        //
        if ( ::AreOnOuterWire(edges, outerWire) )
        {
          isOk = false;
          return;
        }
      }

      if ( (startId == nid) || seedIds.Contains(nid) )
        continue; // Do not look back.

      if ( featureIndices.Contains(nid) )
        continue; // Skip checked.

      if ( nextIterIds.Contains(nid) )
        continue; // Skip faces that will be checked later on recursively.

      const TopoDS_Wire* wirePtr   = m_mapFaceOuterWire.Seek(nFace);
      const TopoDS_Wire  outerWire = ( wirePtr ) ? (*wirePtr)
                                                 : FR_Utils::CacheOuterWire( nid, m_it->GetGraph() );

      // Check if the edges are all included to the outer wire of neighbor.
      if ( ::AreOnOuterWire(edges, outerWire) )
      {
        nextIterIds.Add(nid);
      }
      else
      {
        // We in internal wire. Arc in AAG should have Convex type.
        Handle(FR_FeatureAttrAngle)
          angAttr = m_it->GetGraph()->ATTR_ARC<FR_FeatureAttrAngle>( FR_AAG::t_arc(fid, nid) );

        if ( !FR_FeatureAngle::IsConvex( angAttr->GetAngleType() ) )
          isOk = false;
      }
    }
  }

  if ( (nextIterIds.Extent() != 0) &&
       (featureIndices.Extent() < MaxNumOfFeatureFaces) ) // Protection from infinite recursion.
  {
    // Continue recursively.
    this->propagate(startId, nextIterIds, featureFaces, featureIndices, isOk);
  }
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCavitiesRule::isConvex(const int          fid,
                                             const TopoDS_Wire& wire)
{
  bool isConvex = true;

  for ( TopExp_Explorer eexp(wire, TopAbs_EDGE); eexp.More() && isConvex; eexp.Next() )
  {
    const TopoDS_Edge& edgeOnWire = TopoDS::Edge( eexp.Current() );

    // Check that starting inner wire have convex adjacency.
    // Iterate over adjacent faces and check their convexity.
    const FR_Feature&
      nids = m_it->GetGraph()->GetNeighborsThru(fid, edgeOnWire);
    //
    for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
    {
      const int nid = nit.Key();

      FR_AAG::t_arc arc(fid, nid);

      // Check the dihedral angle.
      Handle(FR_FeatureAttrAngle)
        angAttr = m_it->GetGraph()->ATTR_ARC<FR_FeatureAttrAngle>(arc);
      //
      if ( !FR_FeatureAngle::IsConvex( angAttr->GetAngleType() ) )
      {
        isConvex = false;
        break;
      }
    }
  }

  return isConvex;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCavitiesRule::isNotEntireShape(const TopTools_IndexedMapOfShape& newFeatureFaces,
                                                     const FR_Feature&            newFeatureIndices)
{
  const TopTools_IndexedMapOfShape&
    allFaces = m_it->GetGraph()->GetMapOfFaces();

  // Check that not full solid is found.
  if ( allFaces.Size() != (newFeatureIndices.Extent() + 1) )
    return true; // Full part except base face is found.

  bool allFacesFound = true;

  // Check feature faces.
  for ( TopTools_IndexedMapOfShape::Iterator fit(newFeatureFaces); fit.More(); fit.Next() )
  {
    if ( !allFaces.Contains( fit.Value() ) )
    {
      allFacesFound = false;
      break;
    }
  }

  return !allFacesFound;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCavitiesRule::isSizeOk(const TopTools_IndexedMapOfShape& newFeatureFaces)
{
  if ( Precision::IsInfinite(m_fMaxSize) || ( Abs(m_fMaxSize) < gp::Resolution() ) )
    return true; // Early exit when maximal size is not set.

  // Iterate over faces and check the size criteria.
  TopoDS_Compound comp;
  BRep_Builder builder; builder.MakeCompound(comp);
  TopTools_IndexedMapOfShape::Iterator facesIter(newFeatureFaces);
  //
  for ( ; facesIter.More(); facesIter.Next() )
    builder.Add( comp, facesIter.Value() );

  double xMin, yMin, zMin, xMax, yMax, zMax;
  FR_Utils::Bounds(comp, xMin, yMin, zMin, xMax, yMax, zMax, 0., false);

  const double dim[3] = { Abs(xMax - xMin), Abs(yMax - yMin), Abs(zMax - zMin) };

  gp_Pnt pntMin(xMin, yMin, zMin);
  gp_Pnt pntMax(xMax, yMax, zMax);
  //
  const double size = Max( dim[0], Max(dim[1], dim[2]) );

  if ( size - m_fMaxSize > Precision::Confusion() )
    return false;

  return true;
}
