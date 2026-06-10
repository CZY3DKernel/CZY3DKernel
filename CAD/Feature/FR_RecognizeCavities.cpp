#include "./FR_RecognizeCavities.h"

// FR includes
#include "./FR_RecognizeCavitiesRule.h"

// Analysis Situs includes
#include "./FR_AAG.h"
#include "./FR_AAGIterator.h"
#include "./FR_FeatureAttrAngle.h"

// OpenCascade includes
#include <TopExp_Explorer.hxx>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

FR_RecognizeCavities::FR_RecognizeCavities(const TopoDS_Shape&  shape)
//
: FR_Recognizer ( shape, nullptr ),
  m_fMaxSize         ( Precision::Infinite() )
{}

//-----------------------------------------------------------------------------

FR_RecognizeCavities::FR_RecognizeCavities(const Handle(FR_AAG)& aag)
//
: FR_Recognizer ( aag ),
  m_fMaxSize         ( Precision::Infinite() )
{}

//-----------------------------------------------------------------------------

void FR_RecognizeCavities::SetMaxSize(const double maxSize)
{
  m_fMaxSize = maxSize;
}

//-----------------------------------------------------------------------------

double FR_RecognizeCavities::GetMaxSize() const
{
  return m_fMaxSize;
}

//-----------------------------------------------------------------------------

const FR_RecognizeCavities::t_cavities&
  FR_RecognizeCavities::GetCavities() const
{
  return m_cavities;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeCavities::Perform()
{
  // Clean up the result.
  m_result.faces.Clear();
  m_result.ids.Clear();

  /* ====================
   *  Stage 1: build AAG
   * ==================== */

  // Build AAG if not available.
  if ( m_aag.IsNull() )
  {
#if defined COUT_DEBUG
    TIMER_NEW
    TIMER_GO
#endif

    m_aag = new FR_AAG(m_master, false);

#if defined COUT_DEBUG
    TIMER_FINISH
    TIMER_COUT_RESULT_MSG("Construct AAG")
#endif
  }

  /* ===========================
   *  Stage 2: recognition loop
   * =========================== */

#if defined COUT_DEBUG
  TIMER_NEW
  TIMER_GO
#endif

  // Find promising seed faces to start recognition from.
  FR_Feature seeds, traversed;
  this->findSeeds(seeds);
  //
  Handle(FR_AAGSetIterator)
    sit = new FR_AAGSetIterator(m_aag, seeds);

  // Recognize in a loop having the rule as a cursor.
  Handle(FR_RecognizeCavitiesRule)
    rule = new FR_RecognizeCavitiesRule(sit,
                                             m_fMaxSize);
  //
  for ( ; sit->More(); sit->Next() )
  {
    // Recognizer iterates some faces internally. We don't want to
    // use such faces as seeds, so we skip them here.
    if ( traversed.Contains( sit->GetFaceId() ) )
      continue;

    // Attempt to recognize locally.
    if ( rule->Recognize(m_result.faces, m_result.ids) )
    {
      // Pick up those faces iterated by the recognizer and exclude them
      // from the list to iterate.
      traversed.Unite( rule->JustTraversed() );
    }

    // The recognition process might have been cancelled, so let's check
    // it here in return just in case.
    /*if ( m_progress.IsCancelling() )
    {
      return false;
    }*/
  }

  // Initialize cavities.
  this->collectCavities();

#if defined COUT_DEBUG
  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Cavity recognition")
#endif

  return true; // Success.
}

//-----------------------------------------------------------------------------

void FR_RecognizeCavities::findSeeds(FR_Feature& seeds)
{
  seeds.Clear();

  // Iterate in random order.
  Handle(FR_AAGRandomIterator)
    it = new FR_AAGRandomIterator(m_aag);
  //
  FR_Feature traversed;
  //
  for ( ; it->More(); it->Next() )
  {
    const int fid = it->GetFaceId();
    //
    if ( traversed.Contains(fid) )
      continue;

    const TopoDS_Face& face = m_aag->GetFace(fid);

    // Loop over the inner wires.
    TopoDS_Wire outerWire = FR_Utils::CacheOuterWire(fid, m_aag);
    //
    for( TopExp_Explorer wexp(face, TopAbs_WIRE); wexp.More(); wexp.Next() )
    {
      const TopoDS_Wire& wire = TopoDS::Wire( wexp.Current() );
      //
      if ( wire.IsPartner(outerWire) )
        continue;

      /* The following code is dealing with inner contours only */

      bool isConvexOnly = true;
      FR_Feature nids;

      // Loop over the inner edges to check vexity.
      for ( TopExp_Explorer eexp(wire, TopAbs_EDGE); eexp.More(); eexp.Next() )
      {
        const TopoDS_Edge& edgeOnWire = TopoDS::Edge( eexp.Current() );

        // Get neighbors across the current edge.
        const FR_Feature&
          edgeNids = m_aag->GetNeighborsThru(fid, edgeOnWire);
        //
        if ( !edgeNids.Extent() ) // Protection, just in case.
        {
          isConvexOnly = false;
          break;
        }

        // All neighbors have to be convex-adjacent to the base face.
        for ( FR_Feature::Iterator nit(edgeNids); nit.More(); nit.Next() )
        {
          const int nid = nit.Key();

          FR_AAG::t_arc arc(fid, nid);

          // Get angle to check for vexity.
          Handle(FR_FeatureAttrAngle)
            attrAngle = m_aag->ATTR_ARC<FR_FeatureAttrAngle>(arc);
          //
          if ( attrAngle.IsNull() )
            continue;

          // Check vexity.
          if ( !FR_FeatureAngle::IsConvex( attrAngle->GetAngleType() ) )
          {
            isConvexOnly = false;
            break;
          }
        }

        // Add to the collection of traversed faces.
        nids.Unite(edgeNids);
      }

      // If all inner wires are convex, take current face as another seed.
      if ( isConvexOnly )
      {
        traversed.Unite(nids);
        seeds.Add(fid);
      }
    }
  }
}

//-----------------------------------------------------------------------------

void FR_RecognizeCavities::collectCavities()
{
  std::vector<FR_Feature> ccomps;

  m_aag->PushSubgraph(m_result.ids);
  {
    m_aag->GetConnectedComponents(ccomps);
  }
  m_aag->PopSubgraph();

  for ( const auto& ccomp : ccomps )
  {
    FR_Feature bases;

    // Collect neighbor faces that are not cavities. These would be the bases.
    for ( FR_Feature::Iterator fit(ccomp); fit.More(); fit.Next() )
    {
      const int fid = fit.Key();

      // Collect base faces by traversing all the neighbors.
      const FR_Feature& nids = m_aag->GetNeighbors(fid);
      //
      for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
      {
        const int nid = nit.Key();

        // Skip feature faces.
        if ( m_result.ids.Contains(nid) || bases.Contains(nid) )
          continue;

        bases.Add(nid);
      }
    } // by connected components

    // Add to the result.
    m_cavities.push_back({ccomp, bases});
  }
}
