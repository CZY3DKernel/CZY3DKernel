
#ifndef FR_AdjacencyMxEigen_h
#define FR_AdjacencyMxEigen_h

// FR includes
#include "./FR_AdjacencyMx.h"

// Eigen includes
#pragma warning(push, 0)
#pragma warning(disable : 4702 4701)
#include <Eigen/Dense>

#pragma warning(default : 4702 4701)
#pragma warning(pop)

//! Converts the adjacency matrix to the Eigen matrix.
//! \param[in]  mx     the adjacency matrix to convert.
//! \param[out] idxMap the mapping between the original face IDs and their corresponding indices
//!                    in the output Eigen matrix.
//! \return equivalent Eigen matrix.
EXTENSION_EXPORT Eigen::MatrixXd
  FR_AdjMxToEigen(const FR_AdjacencyMx&       mx,
                       FR_AdjacencyMx::t_indexMap& idxMap);

#endif
