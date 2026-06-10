
#ifndef FR_AdjacencyMx_h
#define FR_AdjacencyMx_h

// FR includes
#include "./FR.h"

// OpenCascade includes
#include <NCollection_DataMap.hxx>
#include <NCollection_DoubleMap.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

// Standard includes
#include <vector>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Adjacency matrix for all sorts of directed and undirected topology graphs,
//! such as face adjacency graphs (including AAG). In this matrix, we store
//! adjacency lists by rows:
//!
//! face 1 : <neighbor 1, neighbor 2, etc.>
//! face 2 : <neighbor 1, neighbor 2, etc.>
//! ...
class FR_AdjacencyMx
{
public:

  //! Type definition for the internal data structure.
  typedef NCollection_DataMap<t_topoId, TColStd_PackedMapOfInteger> t_mx;

  //! Standard collections-driven adjacency matrix.
  typedef std::vector< std::vector<int> > t_std_mx;

  //! Map of indices for graph labeling.
  typedef NCollection_DoubleMap<int, t_topoId> t_indexMap;

public:

  //! Default ctor.
  //! \param[in] alloc optional heap memory allocator.
  FR_AdjacencyMx(const Handle(NCollection_BaseAllocator)& alloc = nullptr) : mx(64, alloc)
  {}

  //! Assignment ctor.
  //! \param[in] _amx other adjacency matrix to copy into this one.
  FR_AdjacencyMx(const FR_AdjacencyMx& _amx)
  {
    this->mx = _amx.mx;
  }

public:

  //! Dumps this adjacency matrix to the passed output stream.
  EXTENSION_EXPORT void
    Dump(std::ostream& out) const;

public:

  t_mx mx; //!< Adjacency rows.

public:

  //! Converts the adjacency matrix to the standard C++ matrix.
  //! \param[out] idxMap the mapping between the original face IDs and their corresponding indices
  //!                    in the output standard C++ matrix.
  //! \return equivalent matrix driven by the standard C++ collections.
  EXTENSION_EXPORT t_std_mx
    AsStandard(t_indexMap& idxMap) const;
};

#endif
