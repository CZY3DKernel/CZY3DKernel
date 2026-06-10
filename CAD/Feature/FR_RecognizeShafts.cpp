#include "./FR_RecognizeShafts.h"

// FR includes
#include "./FR_RecognitionRuleShaft.h"

// FR includes
#include "./FR_AAGIterator.h"

// OpenCascade includes
#include <gp_Cylinder.hxx>
#include <TopExp.hxx>

//-----------------------------------------------------------------------------

FR_RecognizeShafts::FR_RecognizeShafts(const Handle(FR_AAG)& aag)
//
: FR_Recognizer (aag),
  m_fLinToler        (0.)
{}

//-----------------------------------------------------------------------------

void FR_RecognizeShafts::SetLinearTolerance(const double tol)
{
  m_fLinToler = tol;
}

//-----------------------------------------------------------------------------

void FR_RecognizeShafts::SetDomain(const FR_Feature& domain)
{
  m_domain = domain;
}

//-----------------------------------------------------------------------------

void FR_RecognizeShafts::SetExcludedFaces(const FR_Feature& fids)
{
  m_xSeeds = fids;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeShafts::Perform(const double radius)
{
  if ( !this->performInternal(radius) )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

bool FR_RecognizeShafts::performInternal(const double radius)
{
  // Clean up the result.
  m_result.faces.Clear();
  m_result.ids.Clear();

  /* ====================
   *  Stage 1: build AAG.
   * ==================== */

  // Build master AAG.
  if ( m_aag.IsNull() )
  {
    // We do not allow smooth transitions here.
    m_aag = new FR_AAG(m_master, false);
  }

  /* ===========================
   *  Stage 2: recognition loop.
   * =========================== */

  FR_Feature traversed;

  // Iterate over the entire AAG in a random manner looking for
  // specific patterns.
  // ...

  Handle(FR_AAGIterator) seed_it = new FR_AAGRandomIterator(m_aag);

  // Linear tolerance to use.
  double linToler;
  //
  if (m_fLinToler < Precision::Confusion())
      linToler = BRep_Tool::MaxTolerance(m_aag->GetMasterShape(), TopAbs_EDGE);
  else
    linToler = m_fLinToler;

  // Prepare local recognition cursor.
  Handle(FR_RecognitionRuleShaft)
    rule = new FR_RecognitionRuleShaft(seed_it,
                                            radius);
  //
  rule->SetLinearTolerance(linToler);

  // Main recognition loop. Any face is a seed, we let the rule decide.
  for ( ; seed_it->More(); seed_it->Next() )
  {
    const int fid = seed_it->GetFaceId();

    // If domain is set, check that the next face is in the domain.
    if ( !m_domain.IsEmpty() && !m_domain.Contains(fid) )
      continue;

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

  /* =====================================
   *  Stage 3: compose feature structures.
   * ===================================== */

  // Get the connected components.
  std::vector<FR_Feature> groups;
  m_aag->GetConnectedComponents(m_result.ids, groups);

  // Iterate over groups.
  for ( auto& group : groups )
  {
    Handle(FR_Shaft) shaft = new FR_Shaft;
    //
    this->populateShaft(group, shaft);

    // Add to the result.
    m_shafts.push_back(shaft);
  }

  return true; // Success.
}

//-----------------------------------------------------------------------------

void FR_RecognizeShafts::populateShaft(const FR_Feature& fids,
                                            Handle(FR_Shaft)& shaft) const
{
  // Get all cylindrical faces.
  FR_Feature            allCyls;
  std::optional<gp_Cylinder> refCyl;
  //
  for ( FR_Feature::Iterator fit(fids); fit.More(); fit.Next() )
  {
    const int          fid  = fit.Key();
      const TopoDS_Face& face = m_aag->GetFace(fid);

    gp_Cylinder cyl;
    if ( !FR_Utils::IsCylindrical(face, cyl) )
      continue;

    if ( !refCyl.has_value() )
      refCyl = cyl;

    allCyls.Add(fid);
  }

  if ( !refCyl.has_value() )
    return;

  gp_Ax1 axis = refCyl->Axis();

  /* ================
   *  Compute length.
   * ================ */

  // Get all vertices of the cylindrical faces.
  std::vector<gp_XYZ> pts;
  //
  for ( FR_Feature::Iterator fit(allCyls); fit.More(); fit.Next() )
  {
    const int          fid  = fit.Key();
      const TopoDS_Face& face = m_aag->GetFace(fid);

    TopTools_IndexedMapOfShape faceVerts;
    TopExp::MapShapes(face, TopAbs_VERTEX, faceVerts);

    for ( int v = 1; v <= faceVerts.Extent(); ++v )
        pts.push_back(BRep_Tool::Pnt(TopoDS::Vertex(faceVerts(v))).XYZ());
  }

  // Project to axis to get the range.
  const gp_XYZ axisDir = axis.Direction().XYZ();
  double       rangeMin =  DBL_MAX;
  double       rangeMax = -DBL_MAX;
  //
  for ( const auto& pnt : pts )
  {
    const double dot = ( pnt - axis.Location().XYZ() ).Dot(axisDir);

#if defined DRAW_DEBUG
    m_plotter.DRAW_POINT(pnt, Color_White, "rangePt");
#endif

    if ( dot < rangeMin )
      rangeMin = dot;
    if ( dot > rangeMax )
      rangeMax = dot;
  }

  /* =====================
   *  Populate the result.
   * ===================== */

  shaft->axis     = axis;
  shaft->diameter = refCyl->Radius()*2;
  shaft->fids     = fids;
  shaft->length   = Abs(rangeMax - rangeMin);
}
