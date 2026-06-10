
#ifndef FR_CheckDihedralAngle_h
#define FR_CheckDihedralAngle_h

// FR includes
#include "./FR_FeatureAngleType.h"
#include "./FR_Utils.h"
#include "./FR_Collections.h"

// OCCT includes
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include "../Common/Extension_Export.hxx"

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Utility to analyze dihedral angles.
class FR_CheckDihedralAngle : public Standard_Transient
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_CheckDihedralAngle, Standard_Transient)

public:

  //! Constructor.
  //! \param[in] progress Progress Entry.
  //! \param[in] plotter  Imperative Plotter.
  EXTENSION_EXPORT
    FR_CheckDihedralAngle(const Handle(NCollection_BaseAllocator)& alloc = nullptr);

public:

  //! Calculates angle between two faces using midpoints to query the normal
  //! vectors. Orientation of faces is also taken into account.
  //! \param[in]  F                first face.
  //! \param[in]  G                second face.
  //! \param[in]  allowSmooth      indicates whether to process C1 smooth joints.
  //! \param[in]  smoothAngularTol angular tolerance used for recognition
  //!                              of smooth dihedral angles. A smooth angle
  //!                              may appear to be imperfect by construction,
  //!                              but still smooth by the design intent. With
  //!                              this parameter you're able to control it.
  //! \param[out] commonEdges      common edges.
  //! \param[out] angRad           angle in radians.
  //! \param[out] FP               sample point at F.
  //! \param[out] GP               sample point at G.
  //! \param[out] FN               norm at F.
  //! \param[out] GN               norm at G.
  //! \return angle between faces.
  EXTENSION_EXPORT FR_FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face&          F,
                      const TopoDS_Face&          G,
                      const bool                  allowSmooth,
                      const double                smoothAngularTol,
                      TopTools_IndexedMapOfShape& commonEdges,
                      double&                     angleRad,
                      gp_Pnt&                     FP,
                      gp_Pnt&                     GP,
                      gp_Vec&                     FN,
                      gp_Vec&                     GN) const;

  //! Calculates angle between two faces using midpoints to query the normal
  //! vectors. Orientation of faces is also taken into account.
  //! \param[in]  F                first face.
  //! \param[in]  G                second face.
  //! \param[in]  allowSmooth      indicates whether to process C1 smooth joints.
  //! \param[in]  smoothAngularTol angular tolerance used for recognition
  //!                              of smooth dihedral angles. A smooth angle
  //!                              may appear to be imperfect by construction,
  //!                              but still smooth by the design intent. With
  //!                              this parameter you're able to control it.
  //! \param[out] commonEdges      common edges.
  //! \param[out] angRad           angle in radians.
  //! \return angle between faces.
  EXTENSION_EXPORT FR_FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face&          F,
                      const TopoDS_Face&          G,
                      const bool                  allowSmooth,
                      const double                smoothAngularTol,
                      TopTools_IndexedMapOfShape& commonEdges,
                      double&                     angleRad) const;

  //! Calculates angle between two faces using midpoints to query the normal
  //! vectors. Orientation of faces is also taken into account.
  //! \param[in]  F                first face.
  //! \param[in]  G                second face.
  //! \param[in]  allowSmooth      indicates whether to process C1 smooth joints.
  //! \param[in]  smoothAngularTol angular tolerance used for recognition
  //!                              of smooth dihedral angles. A smooth angle
  //!                              may appear to be imperfect by construction,
  //!                              but still smooth by the design intent. With
  //!                              this parameter you're able to control it.
  //! \param[out] commonEdges      common edges.
  //! \return angle between faces.
  EXTENSION_EXPORT FR_FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face&          F,
                      const TopoDS_Face&          G,
                      const bool                  allowSmooth,
                      const double                smoothAngularTol,
                      TopTools_IndexedMapOfShape& commonEdges) const;

  //! Calculates angle between two faces using midpoints to query the normal
  //! vectors. Orientation of faces is also taken into account.
  //! \param[in] F                first face.
  //! \param[in] G                second face.
  //! \param[in] allowSmooth      indicates whether to process C1 smooth joints.
  //! \param[in] smoothAngularTol angular tolerance used for recognition
  //!                             of smooth dihedral angles. A smooth angle
  //!                             may appear to be imperfect by construction,
  //!                             but still smooth by the design intent. With
  //!                             this parameter you're able to control it.
  //! \return angle between faces.
  EXTENSION_EXPORT FR_FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face& F,
                      const TopoDS_Face& G,
                      const bool         allowSmooth,
                      const double       smoothAngularTol) const;

  //! Calculates angle between two faces using midpoints to query the normal
  //! vectors. Orientation of faces is also taken into account.
  //! \param[in]  F                first face.
  //! \param[in]  G                second face.
  //! \param[in]  allowSmooth      indicates whether to process C1 smooth joints.
  //! \param[in]  smoothAngularTol angular tolerance used for recognition
  //!                              of smooth dihedral angles. A smooth angle
  //!                              may appear to be imperfect by construction,
  //!                              but still smooth by the design intent. With
  //!                              this parameter you're able to control it.
  //! \param[out] angRad           angle in radians.
  //! \return angle between faces.
  EXTENSION_EXPORT FR_FeatureAngleType
    AngleBetweenFaces(const TopoDS_Face& F,
                      const TopoDS_Face& G,
                      const bool         allowSmooth,
                      const double       smoothAngularTol,
                      double&            angleRad) const;

public:

  //! \return non-const reference to the used cache of face-edges.
  FR_IndexedDataMapOfTShapeIndexedMapOfShape&
    ChangeCache()
  {
    return m_FECache;
  }

public:

  //! Sets a Boolean flag that indicates whether to use cache or not.
  //! \param[in] on the Boolean flag to set.
  void SetUseCache(const bool on)
  {
    m_bUseCache = on;
  }

  //! Sets a priori known common edge.
  //! \param[in] commonEdge common edge.
  void SetCommonEdge(const TopoDS_Edge& commonEdge)
  {
    m_commonEdge = commonEdge;
  }

protected:

  //! Given that the passed `face` is rotational and the vexity analysis is being
  //! done at its seam edge, this method extracts the corresponding rotational diameter
  //! and checks if this diameter increases or decreases along the face normal.
  //! \param[in]  pt        the point on surface.
  //! \param[in]  norm      the face norm including the orientation effect.
  //! \param[in]  face      the face in question.
  //! \param[out] isConcave the Boolean flag indicating whether the seam edge
  //!                       is concave or convex.
  //! \return false if the check fails, so the resulting `isConcave` flag should not
  //!         be consulted in the caller code.
  bool
    checkSeamVexity(const gp_Pnt&      pt,
                    const gp_Vec&      norm,
                    const TopoDS_Face& face,
                    bool&              isConcave) const;

protected:

  TopoDS_Edge m_commonEdge; //!< Optional a priori known common edge.

  //! Cache of faces vs edges.
  mutable FR_IndexedDataMapOfTShapeIndexedMapOfShape m_FECache;

  //! Indicates whether to use cache.
  bool m_bUseCache;

};

#endif
