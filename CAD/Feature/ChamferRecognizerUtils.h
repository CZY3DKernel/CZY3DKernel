#pragma once

#include <vector>

#include <Standard_Handle.hxx>
#include <gp_Ax1.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

#include "./CompEdge.h"

class Geom_Curve;
class Geom_Surface;
class Geom_ToroidalSurface;
class Geom_CylindricalSurface;
class Geom_ConicalSurface;
class Geom_SurfaceOfRevolution;
class Geom_SphericalSurface;
class Geom_Plane;
class Geom_BSplineSurface;
class AttributeAdjacentGraph;

namespace ChamferRecognizerUtils
{
	/*
	* @brief filter non smooth CompEdge
	* @param[in/out] compEdges
	*/
	void FilterNonSmoothCompEdges(const TopoDS_Face& face, std::shared_ptr<AttributeAdjacentGraph> aag,
                              std::vector<CompEdge>& compEdges);

	bool CheckBeltLengthWidthRatio(const double beltLength, const double beltWidth);
};
