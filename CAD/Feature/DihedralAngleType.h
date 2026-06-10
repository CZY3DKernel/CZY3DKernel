#pragma once

enum class DihedralAngleType
{
  Undefined     = -1,
  Concave       =  0,
  Convex        =  1,
  Smooth        =  2,
  SmoothConcave =  3,
  SmoothConvex  =  4,
  NonManifold   =  5,
  //
  DihedralAngleType_LAST
};
