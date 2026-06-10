#include "./FR_AdjacencyMx.h"
#include "./FR_AdjacencyMxEigen.h"

// FR includes
#include "./FR_Utils.h"

// OpenCascade includes
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>

//-----------------------------------------------------------------------------

void FR_AdjacencyMx::Dump(std::ostream& out) const
{
  for ( t_mx::Iterator rowIt(this->mx); rowIt.More(); rowIt.Next() )
  {
    const t_topoId         fid  = rowIt.Key();
    const FR_Feature& nids = rowIt.Value();

    out << fid << " |";

    for ( FR_Feature::Iterator nit(nids); nit.More(); nit.Next() )
    {
      const int nid = nit.Key();

      out << " " << nid;
    }
    out << "\n";
  }
}

//-----------------------------------------------------------------------------

Eigen::MatrixXd
  FR_AdjMxToEigen(const FR_AdjacencyMx&       mx,
                       FR_AdjacencyMx::t_indexMap& idxMap)
{
  // Get dimensions.
  const int nbRows = mx.mx.Extent();
  int       nbCols = nbRows;

  // Allocate Eigen matrix and initialize it with zeroes as no
  // initialization of elements is done by default.
  Eigen::MatrixXd emx(nbRows, nbCols);
  //
  for ( int r = 0; r < nbRows; ++r )
    for ( int c = 0; c < nbCols; ++c )
      emx(r, c) = 0.;

  // Fill mapping.
  auto eigenRowIdx = 0;
  for ( FR_AdjacencyMx::t_mx::Iterator rowIt(mx.mx); rowIt.More(); rowIt.Next(), ++eigenRowIdx )
  {
    const t_topoId rowFaceId = rowIt.Key();

    // Set mapping.
    idxMap.Bind(eigenRowIdx, rowFaceId);
  }

  // Populate the matrix.
  eigenRowIdx = 0;
  for ( FR_AdjacencyMx::t_mx::Iterator rowIt(mx.mx); rowIt.More(); rowIt.Next(), ++eigenRowIdx )
  {
    const TColStd_PackedMapOfInteger& row = rowIt.Value();

    // Populate the adjacency matrix.
    for ( TColStd_MapIteratorOfPackedMapOfInteger cit(row); cit.More(); cit.Next() )
    {
      const t_topoId nid = cit.Key();

      emx( eigenRowIdx, idxMap.Find2(nid) ) = 1;
    }
  }

  return emx;
}

//-----------------------------------------------------------------------------

FR_AdjacencyMx::t_std_mx
  FR_AdjacencyMx::AsStandard(t_indexMap& idxMap) const
{
  t_std_mx smx;

  // Fill mapping.
  auto stdRowIdx = 0;
  for ( t_mx::Iterator rowIt(this->mx); rowIt.More(); rowIt.Next(), ++stdRowIdx )
  {
    const t_topoId rowFaceId = rowIt.Key();

    // Set mapping.
    idxMap.Bind(stdRowIdx, rowFaceId);
  }

  // Populate the matrix.
  stdRowIdx = 0;
  for ( t_mx::Iterator rowIt(this->mx); rowIt.More(); rowIt.Next(), ++stdRowIdx )
  {
    const TColStd_PackedMapOfInteger& row = rowIt.Value();

    smx.push_back( std::vector<int>() );
    //
    std::vector<int>& stdRow = smx[stdRowIdx];

    // Populate the adjacency matrix.
    for ( TColStd_MapIteratorOfPackedMapOfInteger cit(row); cit.More(); cit.Next() )
    {
      const t_topoId nid = cit.Key();

      stdRow.push_back( idxMap.Find2(nid) );
    }
  }

  return smx;
}
