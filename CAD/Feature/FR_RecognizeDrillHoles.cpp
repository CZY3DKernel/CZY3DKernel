#include "./FR_RecognizeDrillHoles.h"

// FR includes
#include "./FR_AAG.h"
#include "./FR_Isomorphism.h"
#include "./FR_RecognizeDrillHolesRule.h"

// OCCT includes
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

FR_RecognizeDrillHoles::FR_RecognizeDrillHoles(const TopoDS_Shape&  shape,
                                                         const bool           doHard)
//
: FR_Recognizer (shape, nullptr),
  m_fLinToler        (0.),
  m_fCanRecPrec      (1.e-3),
  m_bHardMode        (doHard),
  m_bPureConicalOn   (true)
{}

//-----------------------------------------------------------------------------

FR_RecognizeDrillHoles::FR_RecognizeDrillHoles(const Handle(FR_AAG)& aag,
                                                         const bool                 doHard)
//
: FR_Recognizer (aag),
  m_fLinToler        (0.),
  m_fCanRecPrec      (1.e-3),
  m_bHardMode        (doHard),
  m_bPureConicalOn   (true)
{}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetLinearTolerance(const double tol)
{
  m_fLinToler = tol;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetCanRecPrecision(const double prec)
{
  m_fCanRecPrec = prec;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetHardFeatureMode(const bool isOn)
{
  m_bHardMode = isOn;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetHardFeatureModeOn()
{
  m_bHardMode = true;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetHardFeatureModeOff()
{
  m_bHardMode = false;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetPureConicalAllowed(const bool isOn)
{
  m_bPureConicalOn = isOn;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetSeedFaceIds(const TColStd_PackedMapOfInteger& fids)
{
  m_seeds = fids;
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::SetFaceIdsToExclude(const TColStd_PackedMapOfInteger& fids)
{
  m_xSeeds = fids;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHoles::Perform(const double radius)
{
  if ( !this->performInternal(radius) )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeDrillHoles::performInternal(const double radius)
{
  // Clean up the result.
  m_result.faces.Clear();
  m_result.ids.Clear();

  /* ====================
   *  Stage 1: build AAG
   * ==================== */

  // Build master AAG.
  if ( m_aag.IsNull() )
  {
#if defined COUT_DEBUG
    TIMER_NEW
    TIMER_GO
#endif

    // We do not allow smooth transitions here. Indeed, holes are very often
    // represented by several pieces of smoothly joint cylindrical
    // surfaces. Such paving allows getting rid of periodic geometry,
    // and that is why many modelers prefer to use it.
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

  TColStd_PackedMapOfInteger traversed;

  // Iterate over the entire AAG in a random manner looking for
  // specific patterns.
  // ...

  Handle(FR_AAGIterator) seed_it;
  //
  if ( !m_seeds.IsEmpty() )
  {
    seed_it = new FR_AAGSetIterator(m_aag, m_seeds);
  }
  else
  {
    seed_it = new FR_AAGRandomIterator(m_aag);
  }

  // Linear tolerance to use.
  double linToler;
  //
  if ( m_fLinToler < Precision::Confusion() )
    linToler = BRep_Tool::MaxTolerance(m_aag->GetMasterShape(), TopAbs_EDGE);
  else
    linToler = m_fLinToler;

  Handle(FR_RecognizeDrillHolesRule)
    rule = new FR_RecognizeDrillHolesRule(seed_it,
                                               radius);
  //
  rule->SetLinearTolerance    (linToler);
  rule->SetCanRecPrecision    (m_fCanRecPrec);
  rule->SetHardFeatureMode    (m_bHardMode); // Turn on/off hard feature detection.
  rule->SetPureConicalAllowed (m_bPureConicalOn);

  // Run the main recognition loop. Any face is a seed, we let the rule decide.
  for ( ; seed_it->More(); seed_it->Next() )
  {
    const int fid = seed_it->GetFaceId();

    // Recognizer iterates some faces internally. We don't want to
    // use such faces as seeds, so we skip them here. Alternatively,
    // a face might have been forcible excluded from the consideration.
    if ( traversed.Contains(fid) || m_xSeeds.Contains(fid) )
      continue;

    // Attempt to recognize.
    if ( rule->Recognize(m_result.faces, m_result.ids) )
    {
      // Pick up those faces iterated by the recognizer and exclude them
      // from the list to iterate.
      traversed.Unite( rule->JustTraversed() );
    }

  }

  /* ======================================================
   *  Stage 3: complete detection with floating isolations
   * ====================================================== */

  // Reset iterator.
  if ( !m_seeds.IsEmpty() )
  {
    Handle(FR_AAGSetIterator)::DownCast(seed_it)->Init(m_aag, m_seeds);
  }
  else
  {
    Handle(FR_AAGRandomIterator)::DownCast(seed_it)->Init(m_aag);
  }

  // Complete recognition.
  for ( ; seed_it->More(); seed_it->Next() )
  {
    const int currentFid = seed_it->GetFaceId();
    //
    if ( m_result.ids.Contains(currentFid) )
      continue;

    const TopoDS_Face& currentFace = m_aag->GetFace(currentFid);

    // Get neighbors and check if all them have been detected as feature
    // faces. If so, then current face is in "floating isolation", so we
    // add it to the feature list.
    TColStd_PackedMapOfInteger current_neighbors;
    seed_it->GetNeighbors(current_neighbors);
    //
    bool isFloatingIsolation = true;
    //
    for ( TColStd_MapIteratorOfPackedMapOfInteger nit(current_neighbors); nit.More(); nit.Next() )
    {
      const int neighbor_id = nit.Key();
      //
      if ( !m_result.ids.Contains(neighbor_id) )
      {
        isFloatingIsolation = false;
        break;
      }
    }

    if ( isFloatingIsolation )
    {
      bool isPlateauEnding = false;
      //
      if ( FR_Utils::IsPlanar(currentFace, false) && current_neighbors.Extent() )
      {
        // For a planar face, its outer wire should have hole feature faces attached.
        // If that's not the case, there's something strange with such a planar ending
        // and we do not want to have it in the recognition result.

        TColStd_PackedMapOfInteger eids;
        TopoDS_Wire                W = FR_Utils::CacheOuterWire(currentFid, m_aag);
        //
        for ( TopExp_Explorer eexp(W, TopAbs_EDGE); eexp.More(); eexp.Next() )
        {
          const int eid = m_aag->RequestMapOfEdges().FindIndex( eexp.Current() );
          eids.Add(eid);
        }

        FR_Feature currentNids = m_aag->GetNeighborsThru(currentFid, eids);
        //
        if ( currentNids.IsEmpty() )
        {
          isPlateauEnding = true;
        }
      }

      if ( !isPlateauEnding )
      {
        m_result.faces.Add( seed_it->GetGraph()->GetFace(currentFid) );
        m_result.ids.Add(currentFid);
      }
    }

  }

#if defined COUT_DEBUG
  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Recognize holes")
#endif

  return true; // Success.
}

//-----------------------------------------------------------------------------

void FR_RecognizeDrillHoles::matchConnectedComponent(const Handle(FR_AAG)& FR_NotUsed(cc),
                                                          FR_Feature&           FR_NotUsed(feature))
{
  // Left for customization in app-specific code.
}
