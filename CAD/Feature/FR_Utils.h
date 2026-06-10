#ifndef FR_Utils_h
#define FR_Utils_h

// FR includes
//#include <FR_BaseCloud.h>
//#include <FR_BorderTrihedron.h>
#include "./FR_Collections.h"
//#include <FR_ConvertCanonicalSummary.h>
#include "./FR_FeatureAngleType.h"
#include "./FR_FeatureFaces.h"
//#include <FR_FileFormat.h>
//#include <FR_IntersectionCurveSS.h>
//#include <FR_Naming.h>
//#include <FR_TopoSummary.h>

// Active Data includes
//#include <ActData_Mesh.h>

// OCCT includes
#include <Bnd_Box.hxx>
#include <BRep_Tool.hxx>
//#include <CskBRepAlgoAPI_Cut.hxx>
//#include <CskBRepAlgoAPI_Fuse.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
//#include <GeCskom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_SweptSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Trsf.hxx>
//#include <CskGraphic3d_TypeOfShadingModel.hxx>
#include <math_BullardGenerator.hxx>
#include <math_Function.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_SequenceOfShape.hxx>

// Standard includes
#include <limits>
#include <optional>
#include <unordered_set>
#include <set>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

#define FR_TooSmallValue 1.0e-4
#define FR_RangeLinPrec  0.01
#define FR_SlashStr      "/"
#define FR_QuoteStr      "\""
#define FR_IsPlanarToler 1.0e-3

//-----------------------------------------------------------------------------

// Forward declarations.
class FR_AAG;

//-----------------------------------------------------------------------------

//! \ingroup ASI_MODELING
//!
//! Auxiliary functions facilitating working with OCCT topological shapes.
namespace FR_Utils
{
    //! Checks whether the given shape contains the given sub-shape. This method
    //! builds a map of sub-shapes, so it is quite slow.
    //! \param[in] shape    master shape to check.
    //! \param[in] subShape sub-shape in question.
    //! \return true/false.
    EXTENSION_EXPORT bool Contains(const TopoDS_Shape& shape, const TopoDS_Shape& subShape);

    //! Computes the two-dimensional bounding box of a wire `W`
    //! on the face `F`. This function makes sure to compute
    //! tight bounding rectangles.
    //!
    //! \sa https://quaoar.su/blog/page/outerwire-problem-of-opencascade
    //!
    //! \param[in]  F       the face in question.
    //! \param[in]  W       the wire in question.
    //! \param[out] umin    the computed U min value.
    //! \param[out] umax    the computed U max value.
    //! \param[out] vmin    the computed V min value.
    //! \param[out] vmax    the computed V max value.
    //! \param[in]  plotter the optional imperative plotter for visual dumps.
    EXTENSION_EXPORT void ComputeWireUVBounds(const TopoDS_Face& F, const TopoDS_Wire& W, double& umin,
                                            double& umax, double& vmin, double& vmax);

    //! Fixed version of BRepTools::OuterWire (see #31172 in the OpenCascade
    //! bugtracker).
    //! \param[in] face    the face in question.
    //! \param[in] plotter the imperative plotter for diagnostic visual dumps.
    //! \return outer wire.
    EXTENSION_EXPORT TopoDS_Wire ComputeOuterWire(const TopoDS_Face& face);
    //! Computes and caches the outer wire for the passed face. If the outer wire
    //! is already computed, it is returned from the corresponding attribute.
    //! \param[in] fid the ID of the face in question.
    //! \param[in] aag the attribute adjacency graph.
    //! \return the computed or cached outer wire.
    EXTENSION_EXPORT TopoDS_Wire CacheOuterWire(const int fid, const Handle(FR_AAG) & aag);

    //! This method is the analogue of TopExp::MapShapes() except the fact that
    //! it populates a data map instead of an indexed map. Data maps can be
    //! used to store the same shapes under different indices (indexed map will
    //! throw an exception if you try to use its Substitute() method passed
    //! the already contained shape).
    //! \param[in]     S        shape to decompose.
    //! \param[out]    M        populated map of sub-shapes.
    //! \param[in,out] startIdx starting index (1-based indexation is the default).
    //! \param[out]    IM       indexed map to avoid duplicates and maintain
    //!                         equal identifiers in the data map and indexed map.
    void MapShapes(const TopoDS_Shape& S, FR_DataMapOfShape& M, int& startIdx,
                              TopTools_IndexedMapOfShape& IM);

    //! This method is the analogue of TopExp::MapShapes() except the fact that
    //! it populates a data map instead of an indexed map. Data maps can be
    //! used to store the same shapes under different indices (indexed map will
    //! throw an exception if you try to use its Substitute() method passed
    //! the already contained shape).
    //! \param[in]  S shape to decompose.
    //! \param[out] M populated map of sub-shapes.
    void MapShapes(const TopoDS_Shape& S, FR_DataMapOfShape& M);

    //! This method is the analogue of TopExp::MapShapes() except the fact that
    //! it does not distinguish sub-shapes with different locations.
    //! \param[in]  S shape to decompose.
    //! \param[out] M populated map of sub-shapes.
    void MapTShapes(const TopoDS_Shape& S, FR_IndexedMapOfTShape& M);

    //! This method is the analogue of TopExp::MapShapes() except the fact that
    //! it does not distinguish sub-shapes with different locations.
    //! \param[in]  S shape to decompose.
    //! \param[in]  T sub-shape's type of interest.
    //! \param[out] M populated map of sub-shapes.
    void MapTShapes(const TopoDS_Shape& S, const TopAbs_ShapeEnum T,
                                   FR_IndexedMapOfTShape& M);

    //! This method is the analogue of TopExp::MapShapesAndAncestors() except
    //! the fact that it does not distinguish sub-shapes with different
    //! locations.
    //! \param[in]  S  shape to decompose.
    //! \param[in]  TS sub-shape's type of interest.
    //! \param[in]  TA ancestor's type of interest.
    //! \param[out] M  populated map of sub-shapes.
    void MapTShapesAndAncestors(const TopoDS_Shape& S, const TopAbs_ShapeEnum TS,
                                               const TopAbs_ShapeEnum TA,
                                               FR_IndexedDataMapOfTShapeListOfShape& M);

    //! Checks curve type.
    //! \param[in]  curve     the curve to check.
    //! \param[out] basecurve the extracted basis curve if the originally
    //!                       passed one is trimmed.
    //! \return true/false.
    template <typename TCurve>
    bool IsTypeOf(const Handle(Geom2d_Curve) & curve, Handle(TCurve) & basecurve)
    {
        if (curve.IsNull())
        {
            return false;
        }

        if (curve->IsInstance(STANDARD_TYPE(TCurve)))
        {
            basecurve = Handle(TCurve)::DownCast(curve);
            return true;
        }

        if (curve->IsInstance(STANDARD_TYPE(Geom2d_TrimmedCurve)))
        {
            Handle(Geom2d_TrimmedCurve) trimmed = Handle(Geom2d_TrimmedCurve)::DownCast(curve);

            if (trimmed.IsNull())
            {
                return false;
            }

            Handle(Geom2d_Curve) basis = trimmed->BasisCurve();

            if (IsTypeOf<TCurve>(basis, basecurve))
            {
                return true;
            }
        }

        return false;
    }

    //! Checks curve type.
    //! \param[in]  curve     the curve to check.
    //! \param[out] basecurve the extracted basis curve if the originally
    //!                       passed one is trimmed.
    //! \return true/false.
    template <typename TCurve>
    bool IsTypeOf(const Handle(Geom_Curve) & curve, Handle(TCurve) & basecurve)
    {
        if (curve.IsNull())
        {
            return false;
        }

        if (curve->IsInstance(STANDARD_TYPE(TCurve)))
        {
            basecurve = Handle(TCurve)::DownCast(curve);
            return true;
        }

        if (curve->IsInstance(STANDARD_TYPE(Geom_TrimmedCurve)))
        {
            Handle(Geom_TrimmedCurve) trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve);

            if (trimmed.IsNull())
            {
                return false;
            }

            Handle(Geom_Curve) basis = trimmed->BasisCurve();

            if (IsTypeOf<TCurve>(basis, basecurve))
            {
                return true;
            }
        }

        return false;
    }

    //! Checks curve type.
    //! \param[in] curve the curve to check.
    //! \return true/false.
    template <typename TCurve>
    bool IsTypeOf(const Handle(Geom_Curve) & curve)
    {
        Handle(TCurve) basecurve;
        return IsTypeOf<TCurve>(curve, basecurve);
    }

    //! Checks edge type.
    //! \param[in]  edge      edge to check.
    //! \param[out] basecurve base curve.
    //! \return true/false.
    template <typename TCurve>
    bool IsTypeOf(const TopoDS_Edge& edge, Handle(TCurve) & basecurve)
    {
        if (edge.IsNull())
            return false;

        double f, l;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
        //
        if (curve.IsNull())
            return false;

        // Check host curve directly.
        if (curve->IsInstance(STANDARD_TYPE(TCurve)))
        {
            basecurve = Handle(TCurve)::DownCast(curve);
            return true;
        }

        // Check trimmed curve which may encapsulate the curve type we're looking for.
        if (curve->IsInstance(STANDARD_TYPE(Geom_TrimmedCurve)))
        {
            Handle(Geom_TrimmedCurve) TC = Handle(Geom_TrimmedCurve)::DownCast(curve);

            if (TC.IsNull())
            {
                return false;
            }

            // Check basis curve.
            basecurve = Handle(TCurve)::DownCast(TC->BasisCurve());
            //
            if (!basecurve.IsNull())
                return true;
        }

        return false;
    }

    //! Checks edge type.
    //! \param[in] edge edge to check.
    //! \return true/false.
    template <typename TCurve>
    bool IsTypeOf(const TopoDS_Edge& edge)
    {
        Handle(TCurve) basecurve;
        return IsTypeOf<TCurve>(edge, basecurve);
    }

    //! Checks face type.
    //! \param[in]  face     face to check.
    //! \param[out] basesurf base surface.
    //! \return true/false.
    template <typename TSurf>
    bool IsTypeOf(const TopoDS_Face& face, Handle(TSurf) & basesurf)
    {
        if (face.IsNull())
            return false;

        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

        if (surf.IsNull())
        {
            return false;
        }

        // Check host surface directly.
        if (surf->IsInstance(STANDARD_TYPE(TSurf)))
        {
            basesurf = Handle(TSurf)::DownCast(surf);
            return true;
        }

        // Check trimmed surface which may encapsulate the surface type we're looking for.
        if (surf->IsInstance(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
        {
            Handle(Geom_RectangularTrimmedSurface) RT =
                Handle(Geom_RectangularTrimmedSurface)::DownCast(surf);

            if (RT.IsNull())
            {
                return false;
            }
            // Check basis surface.
            basesurf = Handle(TSurf)::DownCast(RT->BasisSurface());
            //
            if (!basesurf.IsNull())
                return true;
        }

        return false;
    }

    //! Checks face type.
    //! \param[in] face face to check.
    //! \return true/false.
    template <typename TSurf>
    bool IsTypeOf(const TopoDS_Face& face)
    {
        Handle(TSurf) basesurf;
        return IsTypeOf<TSurf>(face, basesurf);
    }

  //! Checks surface type.
    //! \param[in] surf surface to check.
    //! \return true/false.
    template <typename TSurf>
    static bool IsTypeOf(const Handle(Geom_Surface) & surf)
    {
        if (surf.IsNull())
        {
            return false;
        }

        Handle(TSurf) basesurf;

        // Check host surface directly.
        if (surf->IsInstance(STANDARD_TYPE(TSurf)))
        {
            return true;
        }

        // Check trimmed surface which may encapsulate the surface type we're looking for.
        if (surf->IsInstance(STANDARD_TYPE(CskCskGeom_RectangularTrimmedSurface)))
        {
            Handle(CskCskGeom_RectangularTrimmedSurface) RT =
                Handle(CskCskGeom_RectangularTrimmedSurface)::DownCast(surf);

            if (RT.IsNull())
            {
                return false;
            }

            // Check basis surface.
            basesurf = Handle(TSurf)::DownCast(RT->BasisSurface());
            //
            if (!basesurf.IsNull())
                return true;
        }

        return false;
    }

    //! Checks if the passed edge is circular.
    //! \param[in] edge edge to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCircular(const TopoDS_Edge& edge);

    //! Checks if the passed edge is circular and returns the circle props.
    //! \param[in]  edge the edge to check.
    //! \param[out] circ the extracted circle.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCircular(const TopoDS_Edge& edge, gp_Circ& circ);

    //! Checks if the passed curve is circular.
    //! \param[in] curve curve to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCircular(const Handle(Geom_Curve) & curve);

    //! Checks if the passed curve is circular.
    //! \param[in]  curve     the curve to check.
    //! \param[out] circ      the extracted circle.
    //! \param[in]  canrec    the Boolean flag indicating whether to try
    //!                       canonical recognition.
    //! \param[in]  canrecTol the canonical recognition tolerance (if this
    //!                       mode is activated with the previous argument).
    //! \return true/false.
    EXTENSION_EXPORT bool IsCircular(const Handle(Geom_Curve) & curve, gp_Circ& circ,
                                   const bool canrec = false, const double canrecTol = 0.01);

    //! Checks if the passed edge is circular with canonical conversion for freeform
    //! curves plugged in.
    //! \param[in]  edge   the edge to check.
    //! \param[out] circ   the extracted circle.
    //! \param[in]  canrec whether to try canonical recognition.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCircular(const TopoDS_Edge& edge, gp_Circ& circ, const bool canrec);

    
  //! Checks if the passed face has planar support.
    //!
    //! The passed tolerance is used for recognizing as planar
    //! those surfaces that are not defined as analytic planes.
    //!
    //! \param[in] face   face to check.
    //! \param[in] canrec whether to try canonical recognition.
    //! \param[in] toler  the tolerance to use.
    //!
    //! \return true/false.
    EXTENSION_EXPORT bool IsPlanar(const TopoDS_Face& face, const bool canrec,
                                 const double tol = FR_IsPlanarToler);

    //! Checks if the passed face has planar support.
    //!
    //! The passed tolerance is used for recognizing as planar
    //! those surfaces that are not defined as analytic planes.
    //!
    //! \param[in]  face   face to check.
    //! \param[out] plane  planar support.
    //! \param[in]  canrec whether to try canonical recognition.
    //! \param[in]  toler  the tolerance to use.
    //!
    //! \return true/false.
    EXTENSION_EXPORT bool IsPlanar(const TopoDS_Face& face, Handle(Geom_Plane) & plane,
                                 const bool canrec, const double tol = FR_IsPlanarToler);

    //! Checks if the passed face has cylindrical support.
    //! \param[in] face face to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face);

    //! Checks if the passed face has cylindrical support and returns the
    //! extracted cylindrical primitive.
    //! \param[in]  face face to check.
    //! \param[out] cyl  extracted cylindrical primitive.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face, gp_Cylinder& cyl);

    //! Checks if the passed face has cylindrical support.
    //! \param[in]  face face to check.
    //! \param[out] ax   axis of the cylindrical surface.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face, gp_Ax1& ax);

    //! Checks if the passed face has cylindrical support and returns the
    //! cylinder's radius.
    //! \param[in]  face   face to check.
    //! \param[out] radius radius of the underlying cylinder.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face, double& radius);

    //! Checks if the passed face has cylindrical support and returns the
    //! basic properties of a cylinder.
    //! \param[in]  face      face to check.
    //! \param[out] radius    radius of the underlying cylinder.
    //! \param[out] ax        axis of the cylinder.
    //! \param[out] angle_min min angle.
    //! \param[out] angle_max max angle.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face, double& radius, gp_Ax1& ax,
                                      double& angle_min, double& angle_max);

    //! Checks if the passed face has cylindrical support and returns the
    //! basic properties of a cylinder.
    //! \param[in]  face          face to check.
    //! \param[out] radius        radius of the underlying cylinder.
    //! \param[out] ax            axis of the cylinder.
    //! \param[in]  computeBounds indicates whether to compute UV bounds of the cylindrical face.
    //!                           This parameter should be `true` in most cases. When it is `false`,
    //!                           the props `angle_min`, `angle_max`, `h_min`, and `h_max` remain
    //!                           uninitialized, but the computations get faster (as not bounding
    //!                           box is computed). Set this argument `false` if you are only
    //!                           interested in the radius and axis of the cylinder.
    //! \param[out] angle_min     min angle.
    //! \param[out] angle_max     max angle.
    //! \param[out] h_min         min longitudal parameter value.
    //! \param[out] h_max         max longitudal parameter value.
    //! \return true/false.
    EXTENSION_EXPORT bool IsCylindrical(const TopoDS_Face& face, double& radius, gp_Ax1& ax,
                                      const bool computeBounds, double& angle_min,
                                      double& angle_max, double& h_min, double& h_max);

    //! Checks whether the passed face has conical support.
    //! \param[in] face face to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsConical(const TopoDS_Face& face);

    //! Checks whether the passed face has conical support.
    //! \param[in]  face face to check.
    //! \param[out] ax1  cone axis.
    //! \return true/false.
    EXTENSION_EXPORT bool IsConical(const TopoDS_Face& face, gp_Ax1& ax1);

    //! Checks if the passed face is conical and, if so, extract its
    //! properties.
    //! \param[in]  face          the face to check.
    //! \param[out] ax            the cone axis.
    //! \param[in]  computeBounds the Boolean flag indicating whether to compute
    //!                           UV bounds of a face. This operation can be computationally
    //!                           costly, so it should be asked specifically.
    //! \param[out] angle_min     the min angle (U min).
    //! \param[out] angle_max     the max angle (U max).
    //! \param[out] h_min         the min height parameter (V min).
    //! \param[out] h_max         the max height parameter (V max).
    //! \param[out] minRadius     the min extracted radius.
    //! \param[out] maxRadius     the max extracted radius.
    //! \return true if the passed face is conical, false -- otherwise.
    EXTENSION_EXPORT bool IsConical(const TopoDS_Face& face, gp_Ax1& ax, const bool computeBounds,
                                  double& angle_min, double& angle_max, double& h_min,
                                  double& h_max, double& minRadius, double& maxRadius);

    //! Checks whether the passed face has spherical support.
    //! \param[in] face face to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsSpherical(const TopoDS_Face& face);

    //! Checks whether the passed face has toroidal support.
    //! \param[in] face face to check.
    //! \return true/false.
    EXTENSION_EXPORT bool IsToroidal(const TopoDS_Face& face);

    //! Checks whether the passed face is a torus.
    //! \param[in]  face face to check.
    //! \param[out] rmax major radius of the host torus.
    //! \return true/false.
    EXTENSION_EXPORT bool IsToroidal(const TopoDS_Face& face, double& rmax);

    //! Checks whether the passed face is a torus.
    //! \param[in]  face face to check.
    //! \param[out] ax   axis of the torus.
    //! \return true/false.
    EXTENSION_EXPORT bool IsToroidal(const TopoDS_Face& face, gp_Ax1& ax);

    //! Checks whether the passed face is a torus.
    //! \param[in]  face face to check.
    //! \param[out] rmin outer minor radius of the host torus.
    //! \param[out] rmax major radius of the host torus.
    //! \param[out] ax   axis of the torus.
    //! \return true/false.
    EXTENSION_EXPORT bool IsToroidal(const TopoDS_Face& face, double& rmin, double& rmax, gp_Ax1& ax);

    //! Checks if the passed face is internal or external, depending
    //! on its norm field orientation.
    //! \param[in] face     the face to check.
    //! \param[in] diameter the presumably known diameter.
    //! \param[in] ax       the presumably known axis.
    //! \return true for internal, false for external.
    EXTENSION_EXPORT bool IsInternal(const TopoDS_Face& face, const double diameter,
                                   const gp_Ax1& ax);

    //! Checks if the passed face is internal or external, depending
    //! on its norm field orientation.
    //! \param[in] face     the face to check.
    //! \param[in] diameter the presumably known diameter.
    //! \param[in] u        the U coordinate of the probe point on the face of interest.
    //! \param[in] v        the V coordinate of the probe point on the face of interest.
    //! \param[in] ax       the presumably known axis.
    //! \return true for internal, false for external.
    EXTENSION_EXPORT bool IsInternal(const TopoDS_Face& face, const double diameter,
                                   const double u, const double v, const gp_Ax1& ax);

    //! Returns the indices of all edges parallel to the passed direction
    //! `dir` in the UV space of the given face `fid`.
    //! \param[in]  fid       the 1-based index of the face to inspect.
    //! \param[in]  aag       the attributed adjacency graph.
    //! \param[in]  dir       the direction of interest.
    //! \param[in]  tolAngDeg the angular tolerance (in degrees) to use.
    //! \param[out] eids      the collected edge IDs.
    EXTENSION_EXPORT void GetEdgesParallelTo(const int fid, const Handle(FR_AAG) & aag,
                                           const gp_Dir2d& dir, const double tolAngDeg,
                                           TColStd_PackedMapOfInteger& eids);

    //! Returns the indices of all vertical edges in the UV space of the
    //! passed face `fid`.
    //! \param[in] fid       the 1-based index of the face to inspect.
    //! \param[in] aag       the attributed adjacency graph.
    //! \param[in] tolAngDeg the angular tolerance (in degrees) to use.
    //! \return the collection of 1-based edge indices.
    EXTENSION_EXPORT TColStd_PackedMapOfInteger GetVerticalEdges(const int fid,
                                                               const Handle(FR_AAG) & aag,
                                                               const double tolAngDeg);

    //! Checks if the passed pcurve is a straight line.
    //! \param[in] pcu    pcurve to check.
    //! \param[in] canrec whether to try canonical recognition.
    //! \return true/false.
    EXTENSION_EXPORT bool IsStraightPCurve(const Handle(Geom2d_Curve) & pcu, const bool canrec);

    //! Checks if the passed pcurve is a straight line.
    //! \param[in]  pcu    pcurve to check.
    //! \param[out] lin    straight line primitive.
    //! \param[in]  canrec whether to try canonical recognition.
    //! \return true/false.
    EXTENSION_EXPORT bool IsStraightPCurve(const Handle(Geom2d_Curve) & pcu, gp_Lin2d& lin,
                                         const bool canrec);

    //! Calculates bounding box for the given shape.
    //! \param shape     [in]  input shape.
    //! \param XMin      [out] min X.
    //! \param YMin      [out] min Y.
    //! \param ZMin      [out] min Z.
    //! \param XMax      [out] max X.
    //! \param YMax      [out] max Y.
    //! \param ZMax      [out] max Z.
    //! \param tolerance [in]  tolerance to enlarge the bounding box with.
    //! \param optimize  [in]  whether to optimize bbox with numerical methods.
    //! \return false if bounding box is void.
    EXTENSION_EXPORT bool Bounds(const TopoDS_Shape& shape, double& XMin, double& YMin, double& ZMin,
                               double& XMax, double& YMax, double& ZMax,
                               const double tolerance = 0.0, const bool optimize = true);

    //! Computes the axis-aligned bounding box for the passed shape.
    //! \param[in]  shape     the shape of interest.
    //! \param[in]  useFacets indicates whether to compute bounding box on facets.
    //! \param[in]  keepGap   indicates whether to preserve "gap" of AABB set by OpenCascade.
    //! \param[out] bndBox    the computed axis-aligned bounding box.
    //! \return true in case of success, false -- otherwise.
    EXTENSION_EXPORT bool Bounds(const TopoDS_Shape& shape, const bool useFacets, const bool keepGap,
                               Bnd_Box& bndBox);

    //! Calculates bounding box for the given triangulation.
    //! \param[in]  mesh        input mesh.
    //! \param[in]  loc         transformation to apply to the input mesh.
    //! \param[out] bndBox      bounding box.
    //! \return true if operation is successful and false otherwise.
    EXTENSION_EXPORT bool Bounds(const Handle(Poly_Triangulation) & mesh, const TopLoc_Location& loc,
                               Bnd_Box& bndBox);

    //! Calculates bounding box for the given triangulation.
    //! \param tris      [in]  input triangulation.
    //! \param XMin      [out] min X.
    //! \param YMin      [out] min Y.
    //! \param ZMin      [out] min Z.
    //! \param XMax      [out] max X.
    //! \param YMax      [out] max Y.
    //! \param ZMax      [out] max Z.
    //! \param tolerance [in]  tolerance to enlarge the bounding box with.
    //! \return false if bounding box is void.
    EXTENSION_EXPORT bool Bounds(const Handle(Poly_Triangulation) & tris, double& XMin, double& YMin,
                               double& ZMin, double& XMax, double& YMax, double& ZMax,
                               const double tolerance = 0.0);

    //! Functions for verification of results.
    namespace Verify
    {
        //! This method compares two optional axes given that they can have different states, i.e.
        //! initialized (has value) versus uninitialized. While on Windows direct comparison of the
        //! dereferenced values was quite Ok, on Linux we have to take special care of the optionals'
        //! states to avoid surprises at runtime.
        //! \param[in] a           the first optional to compare.
        //! \param[in] b           the second optional to compare.
        //! \param[in] angTolerDeg the angular precision to use in the case when both optionals are
        //! initialized.
        //! \return true in case of equality, false -- otherwise.
    EXTENSION_EXPORT bool AreEqualAxes(const std::optional<gp_Ax1>& a,
                                     const std::optional<gp_Ax1>& b,
                                         const double angTolerDeg);
    }  // namespace Verify

} // FR_Utils namespace.

#endif
