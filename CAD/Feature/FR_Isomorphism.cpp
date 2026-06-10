#include "./FR_Isomorphism.h"

// FR includes
#include "./FR_AAGIterator.h"
#include "./FR_AdjacencyMxEigen.h"
#include "./FR_FeatureAttrAngle.h"
//#include <FR_Timer.h>

#include <TopExp.hxx>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

//! Helper struct that allows keeping Eigen completely inside.
struct FR_Isomorphism::EigenData
{
  FR_Isomorphism*            m_owner;                            //!< Owner object (the algorithm).
  Eigen::MatrixXd                 m_G, m_P;                           //!< Problem and pattern matrices.
  std::vector<Eigen::MatrixXd>    m_Ms;                               //!< Bijection matrices (found isomorphisms).
  FR_AdjacencyMx::t_indexMap m_G_eigenMapping, m_P_eigenMapping; //!< Mappings.

  //! Ctor.
  EigenData(FR_Isomorphism* owner) : m_owner(owner) {}

  //! Initializes bijection matrix `M0`.
  //! \return the initialized matrix.
  Eigen::MatrixXd
    init_M0() const;

  //! Checks if the passed matrix `M` encodes some solution.
  //! Each row in the matrix `M` should contain at least one
  //! element equal to 1.
  //! \param[in] M bijection matrix to check.
  //! \return true/false.
  bool
    solutionExists(const Eigen::MatrixXd& M) const;

  //! Checks if the passed matrix `M` encodes isomorphism
  //! of the graph `P` w.r.t. any subgraph of the problem
  //! graph `G`.
  //! \param[in] M bijection matrix to check.
  //! \return true/false.
  bool
    isIsomorphism(const Eigen::MatrixXd& M);

  //! Prunes the passed bijection candidate matrix.
  //! \param[in,out] M the matrix to prune.
  void
    prune(Eigen::MatrixXd& M);

  //! Recursive routine to find isomorphisms.
  //! \param[in]     curRow   the currently processed row.
  //! \param[in]     M        the current state of the `M` bijection matrix.
  //! \param[in,out] usedCols the used columns.
  void
    recurse(const int                   curRow,
            const Eigen::MatrixXd&      M,
            TColStd_PackedMapOfInteger& usedCols);

  //! Returns the domain image (1-based face ID) of the passed `V_P` vertex of the pattern
  //! graph w.r.t. to the given bijection specified by the `M` matrix.
  //!
  //! \param[in] V_P_eigenIdx the zero-based index of a vertex in the pattern graph.
  //! \param[in] M            the bijection matrix (isomorphism).
  //! \return index of the `V_P` vertex in the problem graph `G`.
  t_topoId
    getDomainImage(const int              V_P_eigenIdx,
                   const Eigen::MatrixXd& M) const;
};

//-----------------------------------------------------------------------------

FR_Isomorphism::FR_Isomorphism()
{
  m_iNumTests = 0;
  m_iModes    = Mode_MatchCardinals;
  m_eigenData = new EigenData(this);
}

//-----------------------------------------------------------------------------

FR_Isomorphism::~FR_Isomorphism()
{
  delete m_eigenData;
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::SetMatchGeomProps(const bool on)
{
  if ( on )
    m_iModes |= Mode_MatchGeometry;
  else
    m_iModes &= (~Mode_MatchGeometry);
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::GetMatchGeomProps() const
{
  return (m_iModes & Mode_MatchGeometry) > 0;
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::SetMatchDimensions(const bool on)
{
  if ( on )
    m_iModes |= Mode_MatchDimensions;
  else
    m_iModes &= (~Mode_MatchDimensions);
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::GetMatchDimensions() const
{
  return (m_iModes & Mode_MatchDimensions) > 0;
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::SetMatchCardinals(const bool on)
{
  if ( on )
    m_iModes |= Mode_MatchCardinals;
  else
    m_iModes &= (~Mode_MatchCardinals);
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::GetMatchCardinals() const
{
  return (m_iModes & Mode_MatchCardinals) > 0;
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::InitGraph(const Handle(FR_AAG)& G_aag)
{
  m_G_aag = G_aag;

  this->fillFacesInfo(m_G_aag, m_faceInfo_G);
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::Perform(const Handle(FR_AAG)& P_aag)
{
  if ( this->GetMatchDimensions() )
  {
    // If the dimensions of the graphs are requested to match, then we're
    // in the graph isomorphism mode (i.e., not in the SUBgraph isomorphism
    // mode).
    if ( P_aag->GetNumberOfNodes() != m_G_aag->GetNumberOfNodes() )
      return false;
  }

  // Initialize the AAG of the pattern.
  m_P_aag = P_aag;
  //
  this->fillFacesInfo(m_P_aag, m_faceInfo_P);

  // Clean up the results.
  m_eigenData->m_Ms.clear();

  // Reset the number of tests.
  m_iNumTests = 0;

#if defined COUT_DEBUG
  TIMER_NEW
  TIMER_GO
#endif

  // Convert to Eigen matrices. The mappings between the indices are preserved in the member-field maps.
  m_eigenData->m_P = FR_AdjMxToEigen(m_P_aag->GetNeighborhood(), m_eigenData->m_P_eigenMapping);
  m_eigenData->m_G = FR_AdjMxToEigen(m_G_aag->GetNeighborhood(), m_eigenData->m_G_eigenMapping);

  // Convert to standard matrices. The mappings between the indices are preserved in the member-field maps.
  m_P_std = m_P_aag->GetNeighborhood().AsStandard(m_P_stdMapping);
  m_G_std = m_G_aag->GetNeighborhood().AsStandard(m_G_stdMapping);

#if defined COUT_DEBUG
  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Convert G and P to the computation-ready matrices")

  TIMER_RESET
  TIMER_GO
#endif

  // Initial bijection.
  Eigen::MatrixXd M0 = m_eigenData->init_M0();

#if defined COUT_DEBUG
  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Initialize M0")

  TIMER_RESET
  TIMER_GO
#endif

#if defined COUT_DEBUG
  const int K = int( M0.rows() );
  const int N = int( M0.cols() );

  std::cout << "Rows (K): "      << K         << std::endl;
  std::cout << "Columns (N): "   << N         << std::endl;
  std::cout << "N^K threshold: " << Pow(N, K) << std::endl;
#endif

  // Find isomorphisms recursively.
  TColStd_PackedMapOfInteger usedCols;
  m_eigenData->recurse(0, M0, usedCols);

#if defined COUT_DEBUG
  std::cout << "Num. of conducted tests for isomorphism: " << m_iNumTests << std::endl;
  std::cout << "Num. of found isomorphisms: " << m_eigenData->m_Ms.size() << std::endl;

  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Find isomorphisms recursively")

  TIMER_RESET
  TIMER_GO
#endif

  // Collect the indices of the feature faces in `G`.
  this->collectFeatures();

#if defined COUT_DEBUG
  TIMER_FINISH
  TIMER_COUT_RESULT_MSG("Collect feature faces")
#endif

  return true; // Even if there are no isomorphisms, we return `true` to indicate success.
}

//-----------------------------------------------------------------------------

int FR_Isomorphism::GetNumIsomorphisms() const
{
  return int( m_eigenData->m_Ms.size() );
}

//-----------------------------------------------------------------------------

const std::vector<TColStd_PackedMapOfInteger>&
  FR_Isomorphism::GetFeatures() const
{
  return m_features;
}

//-----------------------------------------------------------------------------

TColStd_PackedMapOfInteger FR_Isomorphism::GetAllFeatures() const
{
  TColStd_PackedMapOfInteger result;
  //
  for ( size_t k = 0; k < m_features.size(); ++k )
    result.Unite(m_features[k]);

  return result;
}

//-----------------------------------------------------------------------------

void
  FR_Isomorphism::GetDomainImages(const int                   V_P,
                                       TColStd_PackedMapOfInteger& images) const
{
  // Loop over the found isomorphisms.
  for ( const auto& M : m_eigenData->m_Ms )
  {
    const t_topoId im = m_eigenData->getDomainImage(m_eigenData->m_P_eigenMapping.Find2(V_P), M);
    images.Add(im);
  }
}

//-----------------------------------------------------------------------------

int FR_Isomorphism::GetDomainImageFromFirst(const int V_P) const
{
  // Check if there are any isomorphisms.
  if ( m_eigenData->m_Ms.empty() )
    return 0;

  // Get the image from the first isomorphism.
  const int V_P_eigenIdx = m_eigenData->m_P_eigenMapping.Find2(V_P);
  return m_eigenData->getDomainImage(V_P_eigenIdx, m_eigenData->m_Ms[0]);
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::fillFacesInfo(const Handle(FR_AAG)&            aag,
                                        NCollection_DataMap<int, t_faceInfo>& map)
{
  // Iterate AAG in whatever order.
  Handle(FR_AAGRandomIterator)
    it = new FR_AAGRandomIterator(aag);
  //
  for ( ; it->More(); it->Next() )
  {
    const t_topoId     fid = it->GetFaceId();
    const TopoDS_Face& F   = aag->GetFace(fid);

    // Do not collect topologies if cardinal numbers are not requested
    // to match.
    TopTools_IndexedMapOfShape verts, edges, wires;
    //
    if ( this->GetMatchCardinals() )
    {
      TopExp::MapShapes(F, TopAbs_VERTEX, verts);
      TopExp::MapShapes(F, TopAbs_EDGE,   edges);
      TopExp::MapShapes(F, TopAbs_WIRE,   wires);
    }

    t_faceInfo info;
    info.surf   = BRep_Tool::Surface(F);
    info.nVerts = verts.Extent();
    info.nEdges = edges.Extent();
    info.nWires = wires.Extent();

    // For the trimmed surfaces, use the basis ones.
    if ( info.surf->IsInstance( STANDARD_TYPE(Geom_RectangularTrimmedSurface) ) )
    {
      Handle(Geom_RectangularTrimmedSurface)
        RT = Handle(Geom_RectangularTrimmedSurface)::DownCast(info.surf);

      // Use basis surface.
      info.surf = RT->BasisSurface();
    }

    map.Bind(fid, info);
  }
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::areMatching(const int V_P_eigenIdx,
                                      const int V_G_eigenIdx) const
{
  const t_topoId V_P = m_eigenData->m_P_eigenMapping.Find1(V_P_eigenIdx);
  const t_topoId V_G = m_eigenData->m_G_eigenMapping.Find1(V_G_eigenIdx);

  const t_faceInfo& info_G = m_faceInfo_G(V_G);
  const t_faceInfo& info_P = m_faceInfo_P(V_P);

  const TColStd_PackedMapOfInteger& row_P     = m_P_aag->GetNeighbors(V_P);
  const int                         valence_P = row_P.Extent();

  const TColStd_PackedMapOfInteger& row_G     = m_G_aag->GetNeighbors(V_G);
  const int                         valence_G = row_G.Extent();

  /* ==============
   *  Heuristic 01.
   * ============== */

  // Check degrees.
  if ( valence_P > valence_G )
    return false;

  /* ==============
   *  Heuristic 02.
   * ============== */

  // If degrees are Ok, we can go further and check that the arc attributes
  // in G contain the arc attribute in P as a subset.

  int P_angles[FeatureAngleType_LAST];
  //
  for ( int k = 0; k < FeatureAngleType_LAST; ++k ) P_angles[k] = 0;
  //
  for ( TColStd_MapIteratorOfPackedMapOfInteger nit(row_P); nit.More(); nit.Next() )
  {
    const t_topoId nid = nit.Key();

    Handle(FR_FeatureAttrAngle)
      P_angleAttr = m_P_aag->ATTR_ARC<FR_FeatureAttrAngle>( FR_AAG::t_arc(V_P, nid) );

    if ( !P_angleAttr.IsNull() )
      P_angles[ P_angleAttr->GetAngleType() ]++;
  }

  int G_angles[FeatureAngleType_LAST];
  //
  for ( int k = 0; k < FeatureAngleType_LAST; ++k ) G_angles[k] = 0;
  //
  for ( TColStd_MapIteratorOfPackedMapOfInteger nit(row_G); nit.More(); nit.Next() )
  {
    const t_topoId nid = nit.Key();

    Handle(FR_FeatureAttrAngle)
      G_angleAttr = m_G_aag->ATTR_ARC<FR_FeatureAttrAngle>( FR_AAG::t_arc(V_G, nid) );

    if ( !G_angleAttr.IsNull() )
      G_angles[ G_angleAttr->GetAngleType() ]++;
  }

  for ( int k = 0; k < FeatureAngleType_LAST; ++k )
    if ( G_angles[k] - P_angles[k] < 0 ) // Not a subset.
      return false;

  // Check topology.
  {
    /* ==============
     *  Heuristic 03.
     * ============== */

    if ( info_G.nVerts != info_P.nVerts )
      return false;

    if ( info_G.nEdges != info_P.nEdges )
      return false;

    if ( info_G.nWires != info_P.nWires )
      return false;
  }

  // Check geometry.
  {
    /* ==============
     *  Heuristic 04.
     * ============== */

    // Surface type.
    if ( info_G.surf->DynamicType() != info_P.surf->DynamicType() )
      return false;

    /* ==================
     *  Extra heuristics.
     * ================== */

    // Match props.
    if ( this->GetMatchGeomProps() )
    {
      if ( info_G.surf->IsKind( STANDARD_TYPE(Geom_CylindricalSurface) ) )
      {
        Handle(Geom_CylindricalSurface) cyl_P = Handle(Geom_CylindricalSurface)::DownCast(info_P.surf);
        Handle(Geom_CylindricalSurface) cyl_G = Handle(Geom_CylindricalSurface)::DownCast(info_G.surf);

        if ( Abs( cyl_P->Radius() - cyl_G->Radius() ) > 1e-3 )
          return false;
      }

      // TODO: add more checks (to implement them in FR_Utils?)

      // TODO: add abstract rules instead of fully-fledged matching.
    }
  }

  return true;
}

//-----------------------------------------------------------------------------

Eigen::MatrixXd FR_Isomorphism::EigenData::init_M0() const
{
  const int nRows = m_owner->m_P_aag->GetNumberOfNodes();
  const int nCols = m_owner->m_G_aag->GetNumberOfNodes();

  Eigen::MatrixXd M0(nRows, nCols);

  // Iterate rows of P.
  for ( int idx_P = 0; idx_P < nRows; ++idx_P )
  {
    // Iterate rows of G.
    for ( int idx_G = 0; idx_G < nCols; ++idx_G )
    {
      if ( m_owner->areMatching(idx_P, idx_G) )
        M0(idx_P, idx_G) = 1;
      else
        M0(idx_P, idx_G) = 0;
    }
  }

  return M0;
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::EigenData::solutionExists(const Eigen::MatrixXd& M) const
{
  for ( int r = 0; r < M.rows(); ++r )
  {
    bool hasOneElem = false;
    for ( int c = 0; c < M.cols(); ++c )
    {
      if ( M(r, c) == 1 )
      {
        hasOneElem = true;
        break;
      }
    }

    if ( !hasOneElem )
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

bool FR_Isomorphism::EigenData::isIsomorphism(const Eigen::MatrixXd& M)
{
  m_owner->m_iNumTests++;

  Eigen::MatrixXd MG = M*(M*m_G).transpose();

  return ( MG == m_P );
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::EigenData::prune(Eigen::MatrixXd& M)
{
  // The order of iteration does matter. Simply swapping the following
  // two loops (so that to iterate columns first, instead of rows like
  // it is now) may slow down runtime twice for some cases.
  for ( int r = 0; r < M.rows(); ++r ) // P graph.
  {
    const std::vector<int>& neighbors_P = m_owner->m_P_std[r];

    for ( int c = 0; c < M.cols(); ++c ) // G graph.
    {
      const std::vector<int>& neighbors_G = m_owner->m_G_std[c];

      if ( M(r, c) == 1 )
      {
        // The neighbors should also match in M. If not, then
        // the current M(r,c) = 1 matching is impossible, and
        // we have to cancel it.

        bool neighborsMatch = true;
        for ( std::vector<int>::const_iterator P_it = neighbors_P.cbegin();
              P_it != neighbors_P.cend(); P_it++ )
        {
          const int nr = *P_it;

          bool foundNeighborInG = false;
          for ( std::vector<int>::const_iterator G_it = neighbors_G.cbegin();
                G_it != neighbors_G.cend(); G_it++ )
          {
            const int nc = *G_it;

            if ( M(nr, nc) == 1 )
            {
              foundNeighborInG = true;
              break;
            }
          }

          if ( !foundNeighborInG )
          {
            neighborsMatch = false;
            break;
          }
        }

        if ( !neighborsMatch )
          M(r, c) = 0;
      }
    }
  }
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::EigenData::recurse(const int                   curRow,
                                             const Eigen::MatrixXd&      M,
                                             TColStd_PackedMapOfInteger& usedCols)
{
  // Check the stopping criterion.
  const int numRows = int( M.rows() );
  //
  if ( curRow == numRows )
  {
#if defined COUT_DEBUG
    // Dump M.
    {
      std::stringstream buff;
      buff << M;
      m_owner->m_progress.SendLogMessage( LogInfo(Normal) << "Checking isomorphism for M:\n%1"
                                                  << buff.str() );
    }
#endif

    const bool iso = this->isIsomorphism(M);

#if defined COUT_DEBUG
    m_owner->m_progress.SendLogMessage( LogInfo(Normal) << "Isomorphism: %1."
                                               << iso );
#endif

    if ( iso )
    {
      m_Ms.push_back(M);
    }

    return; // Do not allow further checks.
  }

  const int numCols = int( M.cols() );

  // Adjust matrix M.
  Eigen::MatrixXd MM = M;
  this->prune(MM);

  // After pruning, some matrices may encode no bijections.
  if ( !this->solutionExists(MM) )
    return;

  Eigen::MatrixXd M_next = MM;

  // Eliminate 1's.
  for ( int c = 0; c < numCols; ++c )
  {
    if ( usedCols.Contains(c) )
      continue;

    if ( M_next(curRow, c) == 0 )
      continue;

    // Modify matrix.
    if ( M_next(curRow, c) == 1 )
    {
      // Nullify other columns.
      for ( int cc = 0; cc < numCols; ++cc )
        if ( cc != c )
          M_next(curRow, cc) = 0;

      // Nullify other rows.
      for ( int rr = 0; rr < numRows; ++rr )
        if ( rr != curRow )
          M_next(rr, c) = 0;
    }

    // Proceed recursively.
    usedCols.Add(c);
    //
    this->recurse(curRow + 1, M_next, usedCols);
    //
    usedCols.Remove(c);

    // Reset the modifications done on the matrix M.
    for ( int cc = 0; cc < numCols; ++cc )
      M_next(curRow, cc) = M(curRow, cc);
    //
    for ( int rr = 0; rr < numRows; ++rr )
      if ( rr != curRow )
        M_next(rr, c) = M(rr, c);
  }
}

//-----------------------------------------------------------------------------

t_topoId FR_Isomorphism::EigenData::getDomainImage(const int              V_P_eigenIdx,
                                                        const Eigen::MatrixXd& M) const
{
  const int numCols = int( M.cols() );
  for ( int c = 0; c < numCols; ++c )
  {
    if ( M(V_P_eigenIdx, c) == 1 )
      return m_G_eigenMapping.Find1(c); // Use G mapping as G is a universum.
  }

  return 0; // Invalid index.
}

//-----------------------------------------------------------------------------

void FR_Isomorphism::collectFeatures()
{
  // Loop over the found isomorphisms.
  for ( size_t i = 0; i < m_eigenData->m_Ms.size(); ++i )
  {
    bool isConfirmed = true;

    TColStd_PackedMapOfInteger candidates;

    // Loop over the arcs of the pattern graph.
    const FR_AAG::t_arc_attributes& P_arcAttrs = m_P_aag->GetArcAttributes();
    //
    for ( FR_AAG::t_arc_attributes::Iterator ait(P_arcAttrs); ait.More(); ait.Next() )
    {
      // Get the arc from the pattern graph.
      const FR_AAG::t_arc&          P_arc        = ait.Key();
      const FR_AAG::t_arc_attr_set& P_arcAttrSet = ait.Value();

      // Get the adjacency attribute.
      const Handle(FR_FeatureAttrAngle)
        P_angleAttr = Handle(FR_FeatureAttrAngle)::DownCast(P_arcAttrSet.AngleAttr);

      const t_topoId imF1 = m_eigenData->getDomainImage(m_eigenData->m_P_eigenMapping.Find2(P_arc.F1), m_eigenData->m_Ms[i]);
      const t_topoId imF2 = m_eigenData->getDomainImage(m_eigenData->m_P_eigenMapping.Find2(P_arc.F2), m_eigenData->m_Ms[i]);

      candidates.Add(imF1);
      candidates.Add(imF2);

      // Get the image of the arc, i.e. the arc in the problem graph.
      FR_AAG::t_arc G_arc(imF1, imF2);
      //
      Handle(FR_FeatureAttrAngle)
        G_angleAttr = m_G_aag->ATTR_ARC<FR_FeatureAttrAngle>(G_arc);

      // Compare the attributes.
      if ( ( P_angleAttr.IsNull() || G_angleAttr.IsNull() ) ||
           ( P_angleAttr->GetAngleType() != G_angleAttr->GetAngleType() ) )
      {
        isConfirmed = false;
        break;
      }
    }

    // Treat special case of one-node pattern.
    if ( m_P_aag->GetNumberOfNodes() == 1 )
    {
      Handle(FR_FeatureAttrAngle)
        P_angleAttr = m_P_aag->ATTR_NODE<FR_FeatureAttrAngle>(1);

      const t_topoId imF = m_eigenData->getDomainImage(m_eigenData->m_P_eigenMapping.Find2(1), m_eigenData->m_Ms[i]);

      candidates.Add(imF);

      // Get the corresponding node attribute from the image in G.
      Handle(FR_FeatureAttrAngle)
        G_angleAttr = m_G_aag->ATTR_NODE<FR_FeatureAttrAngle>(imF);

      // Compare the attributes.
      if ( ( P_angleAttr.IsNull() || G_angleAttr.IsNull() ) ||
           ( P_angleAttr->GetAngleType() != G_angleAttr->GetAngleType() ) )
      {
        isConfirmed = false;
      }
    }

    if ( isConfirmed && !candidates.IsEmpty() )
    {
      m_features.push_back(candidates);
    }
  }
}
