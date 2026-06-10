#include "./FR_CheckDihedralAngle.h"

#include <optional>

// FR includes
#include "./FR_Utils.h"

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <ElSLib.hxx>
#include <Geom_Curve.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

FR_CheckDihedralAngle::FR_CheckDihedralAngle(const Handle(NCollection_BaseAllocator)& alloc)
:
  m_FECache         (1024, alloc),
  m_bUseCache       (true)
{}

//-----------------------------------------------------------------------------

FR_FeatureAngleType
  FR_CheckDihedralAngle::AngleBetweenFaces(const TopoDS_Face&          F,
                                                const TopoDS_Face&          G,
                                                const bool                  allowSmooth,
                                                const double                smoothAngularTol,
                                                TopTools_IndexedMapOfShape& commonEdges,
                                                double&                     angleRad,
                                                gp_Pnt&                     FP,
                                                gp_Pnt&                     GP,
                                                gp_Vec&                     FN,
                                                gp_Vec&                     GN) const
{
  TopoDS_Edge commonEdge = m_commonEdge;
  bool        isSeam     = false;

  // Common edge may be known a priori. If not, we have to find some
  if ( commonEdge.IsNull() )
  {
    if ( F.IsSame(G) )
    {
      // Try to find a seam edge
      for ( TopExp_Explorer exp(F, TopAbs_EDGE); exp.More(); exp.Next() )
      {
        const TopoDS_Edge& edge = TopoDS::Edge( exp.Current() );
        ShapeAnalysis_Edge sae;
        //
        if ( sae.IsSeam(edge, F) )
        {
          isSeam     = true;
          commonEdge = edge;
          commonEdges.Add(edge);
          break;
        }
      }
    }
    else
    {
      // Extract edges for faces
      TopTools_IndexedMapOfShape EdgesF, EdgesG;

      // Use cached map of F edges.
      if ( !m_bUseCache || !m_FECache.Contains(F) )
      {
        TopExp::MapShapes(F, TopAbs_EDGE, EdgesF);

        if ( m_bUseCache )
          m_FECache.Add( F, EdgesF );
      }
      else
      {
        EdgesF = m_FECache.FindFromKey(F);
      }

      // Use cached map of G edges.
      if ( !m_bUseCache || !m_FECache.Contains(G) )
      {
        TopExp::MapShapes(G, TopAbs_EDGE, EdgesG);

        if ( m_bUseCache )
          m_FECache.Add( G, EdgesG );
      }
      else
      {
        EdgesG = m_FECache.FindFromKey(G);
      }

      // Collect common edges
      for ( int ef = 1; ef <= EdgesF.Extent(); ++ef )
      {
        for ( int eg = 1; eg <= EdgesG.Extent(); ++eg )
        {
          if ( EdgesF(ef).IsSame( EdgesG(eg) ) )
          {
            commonEdges.Add( EdgesF(ef) );
            //
            if ( commonEdge.IsNull() ) // A single common edge is enough for analysis
              commonEdge = TopoDS::Edge( EdgesF(ef) );
          }
        }
      }
    }
  }
  else
  {
    // Common edge is defined, let's only check if it's a seam or not.
    // We need this information to apply special rules for seam edges
    // on rotational surfaces.
    isSeam = ShapeAnalysis_Edge().IsSeam(commonEdge, F);
  }

  if ( commonEdge.IsNull() )
    return FeatureAngleType_Undefined;

  // Check whether the edge of interest possesses "same parameter" property.
  // If so, we may use its host spatial curves in conjunction with p-curves
  // to sample points on adjacent surfaces. If not, then direction evaluation
  // is not legal anymore, so point inversion problem has to be solved. The
  // latter is quite heavy operation, so it is good that we run it for
  // invalid model only
  const bool isSameParam = BRep_Tool::SameParameter(commonEdge);

  // Get orientation of co-edges on their host faces
  TopAbs_Orientation CumulOriOnF = TopAbs_EXTERNAL, CumulOriOnG = TopAbs_EXTERNAL;
  //
  for ( TopExp_Explorer exp(F, TopAbs_EDGE); exp.More(); exp.Next() )
    if ( exp.Current().IsSame(commonEdge) )
    {
      CumulOriOnF = exp.Current().Orientation();
      break;
    }
  //
  if ( !isSeam )
  {
    for ( TopExp_Explorer exp(G, TopAbs_EDGE); exp.More(); exp.Next() )
      if ( exp.Current().IsSame(commonEdge) )
      {
        CumulOriOnG = exp.Current().Orientation();
        break;
      }
  }
  else
    CumulOriOnG = ( (CumulOriOnF == TopAbs_FORWARD) ? TopAbs_REVERSED : TopAbs_FORWARD );

  // Get host (shared) curve
  double f, l;
  Handle(Geom_Curve) probeCurve = BRep_Tool::Curve(commonEdge, f, l);
  //
  if ( probeCurve.IsNull() )
    return FeatureAngleType_Undefined;

  // Pick up two points on the curve
  const double midParam  = (f + l)*0.5;
  const double paramStep = (l - f)*0.01;
  const double A_param   = midParam - paramStep;
  const double B_param   = midParam + paramStep;
  //
  gp_Pnt A = probeCurve->Value(A_param);
  gp_Pnt B = probeCurve->Value(B_param);

  // Get host surfaces
  Handle(Geom_Surface) S1 = BRep_Tool::Surface(F),
                       S2 = isSeam ? S1 : BRep_Tool::Surface(G);
  Handle(Geom2d_Curve) c2d_F, c2d_G;

  gp_Pnt2d UV;
  gp_Vec TF, Ref; // Tangent for F
  {
    // Vx
    gp_Vec Vx = B.XYZ() - A.XYZ();
    if ( Vx.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    if ( CumulOriOnF == TopAbs_REVERSED )
      Vx.Reverse();

    //this->Plotter().DRAW_VECTOR_AT(A, Vx, Color_Red, "Vx");

    // Say (u, v) is the parametric space of S1. Now we take parameters
    // of the point A on this surface
    if ( isSameParam )
    {
      double cons_f, cons_l;

      // Catch as for invalid curves it raises exception.
      try
      {
        c2d_F = BRep_Tool::CurveOnSurface(commonEdge, F, cons_f, cons_l);
      }
      catch ( ... ) {}

      //
      if ( c2d_F.IsNull() )
        return FeatureAngleType_Undefined; // Face is invalid

      UV = c2d_F->Value(A_param);
    }
    else
    {
      ShapeAnalysis_Surface SAS(S1);
      UV = SAS.ValueOfUV(A, 1.0e-1);
    }

    // N (Vz)
    gp_Pnt P;
    gp_Vec D1U, D1V;
    S1->D1(UV.X(), UV.Y(), P, D1U, D1V);
    //
    gp_Vec N = D1U.Crossed(D1V);
    if ( N.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    if ( F.Orientation() == TopAbs_REVERSED )
      N.Reverse();

    // Set sampling props.
    FP = P;
    FN = N;

    // Vy
    gp_Vec Vy = N.Crossed(Vx);
    if ( Vy.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    TF = Vy.Normalized();

    //this->Plotter().DRAW_VECTOR_AT(A, Vy, Color_Green, "Vy");

    // Ref
    Ref = Vx.Normalized();
  }

  // Apply special rules for the seam edges on canonical rotational surfaces.
  // For such cases, no point inversion is necessary as we can deduce the
  // edge vexity right from the normal field.
  if ( isSeam )
  {
    bool isConcave = false;
    //
    if ( this->checkSeamVexity(FP, FN, F, isConcave) )
    {
      angleRad = M_PI;
      return isConcave ? FeatureAngleType_SmoothConcave : FeatureAngleType_SmoothConvex;
    }
  }

  gp_Pnt2d ST;
  gp_Vec TG; // Tangent for G
  {
    // Vx
    gp_Vec Vx = B.XYZ() - A.XYZ();
    if ( Vx.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    if ( CumulOriOnG == TopAbs_REVERSED )
      Vx.Reverse();

    // Say (s, t) is the parametric space of S2. Now we take parameters
    // of the point A on this surface
    if ( isSameParam )
    {
      double cons_f, cons_l;
      c2d_G = BRep_Tool::CurveOnSurface(commonEdge, G, cons_f, cons_l);
      //
      if ( c2d_G.IsNull() )
        return FeatureAngleType_Undefined; // Face is invalid

      ST = c2d_G->Value(A_param);
    }
    else
    {
      ShapeAnalysis_Surface SAS(S2);
      ST = SAS.ValueOfUV(A, 1.0e-1);
    }

    // N (Vz)
    gp_Pnt P;
    gp_Vec D1S, D1T;
    S2->D1(ST.X(), ST.Y(), P, D1S, D1T);
    //
    gp_Vec N = D1S.Crossed(D1T);
    if ( N.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    if ( G.Orientation() == TopAbs_REVERSED )
      N.Reverse();

    // Set sampling props.
    GP = P;
    GN = N;

    // Vy
    gp_Vec Vy = N.Crossed(Vx);
    if ( Vy.Magnitude() < RealEpsilon() )
      return FeatureAngleType_Undefined;
    //
    TG = Vy.Normalized();

    //this->Plotter().DRAW_VECTOR_AT(A, Vy, Color_Violet, "S2_Vy");
  }

  // Calculate dihedral angle
  angleRad = TF.AngleWithRef(TG, Ref);

#if defined COUT_DEBUG
  std::cout << "Angle is " << angleRad << std::endl;
#endif

  bool isSmooth = false;

  // 3 degrees is the default angular tolerance to recognize smooth angles.
  const double ang_tol = Max(smoothAngularTol, 3.0/180.0*M_PI);
  //
  if ( Abs(Abs(angleRad) - M_PI) < ang_tol )
  {
    isSmooth = true;

    if ( allowSmooth )
      return FeatureAngleType_Smooth;

    // C1 joint requires additional analysis
    gp_Pnt2d UV_shifted, ST_shifted;
    gp_Pnt   S1_P, S2_P;
    gp_Vec   TF_precised_vec, TG_precised_vec;
    gp_Dir   TF_precised, TG_precised;

    // Let's make small offsets along the in-plane vectors and
    // check geometries in additional sample points. Notice that here
    // we prefer to solve point inversion problem instead of walking in
    // 2D. The matter is that even though evaluation from (u, v) to
    // (x, y, z) is very efficient (far more efficient than point
    // inversion), it is not easy to take into account the inherent
    // 2D vs 3D distortion. If we start from 3D, we ensure that the
    // vectors we check are collinear. If we start from 2D there is no
    // such guarantee.

    // Take average by several samples to be more robust. Initially, we do not
    // know how far we have to go into the face interior to make reliable
    // measurement.
    int    inSamples   = 1;
    double inStep      = 0.4;
    double avrAngleRad = 0.0;
    //
    for ( int s = 1; s <= inSamples; ++s )
    {
      const double inDelta = inStep*s;

      S1_P = A.XYZ() + TF.XYZ() * inDelta;
      S2_P = A.XYZ() + TG.XYZ() * inDelta;

      ShapeAnalysis_Surface SAS1(S1), SAS2(S2);
      UV_shifted = SAS1.ValueOfUV(S1_P, 1.0e-1);
      ST_shifted = SAS2.ValueOfUV(S2_P, 1.0e-1);
      S1_P       = SAS1.Value(UV_shifted);
      S2_P       = SAS2.Value(ST_shifted);

      // For invalid models, we need to protect against null vectors.
      TF_precised_vec = ( S1_P.XYZ() - A.XYZ() );
      TG_precised_vec = ( S2_P.XYZ() - A.XYZ() );
      //
      if ( TF_precised_vec.Magnitude() < Precision::Confusion() ||
           TG_precised_vec.Magnitude() < Precision::Confusion() )
        return FeatureAngleType_Undefined;
      //
      TF_precised = TF_precised_vec;
      TG_precised = TG_precised_vec;

      avrAngleRad += TF_precised.AngleWithRef(TG_precised, Ref);
    }
    //
    avrAngleRad /= inSamples;
    angleRad = avrAngleRad;
  }

  // Classify angle
  FR_FeatureAngleType angleType = FeatureAngleType_Undefined;
  if ( angleRad < 0 )
    angleType = isSmooth ? FeatureAngleType_SmoothConvex : FeatureAngleType_Convex;
  else
    angleType = isSmooth ? FeatureAngleType_SmoothConcave : FeatureAngleType_Concave;

  return angleType;
}

//-----------------------------------------------------------------------------

FR_FeatureAngleType
  FR_CheckDihedralAngle::AngleBetweenFaces(const TopoDS_Face&          F,
                                                const TopoDS_Face&          G,
                                                const bool                  allowSmooth,
                                                const double                smoothAngularTol,
                                                TopTools_IndexedMapOfShape& commonEdges,
                                                double&                     angleRad) const
{
  gp_Pnt FP, GP;
  gp_Vec FN, GN;
  return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angleRad,
                                 FP, GP, FN, GN);
}

//-----------------------------------------------------------------------------

FR_FeatureAngleType
  FR_CheckDihedralAngle::AngleBetweenFaces(const TopoDS_Face&          F,
                                                const TopoDS_Face&          G,
                                                const bool                  allowSmooth,
                                                const double                smoothAngularTol,
                                                TopTools_IndexedMapOfShape& commonEdges) const
{
  double angRad = 0.0;
  return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angRad);
}

//-----------------------------------------------------------------------------

FR_FeatureAngleType
  FR_CheckDihedralAngle::AngleBetweenFaces(const TopoDS_Face& F,
                                                const TopoDS_Face& G,
                                                const bool         allowSmooth,
                                                const double       smoothAngularTol) const
{
  TopTools_IndexedMapOfShape commonEdges;
  double angRad = 0.0;
  return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angRad);
}

//-----------------------------------------------------------------------------

FR_FeatureAngleType
  FR_CheckDihedralAngle::AngleBetweenFaces(const TopoDS_Face& F,
                                                const TopoDS_Face& G,
                                                const bool         allowSmooth,
                                                const double       smoothAngularTol,
                                                double&            angRad) const
{
  TopTools_IndexedMapOfShape commonEdges;
  return this->AngleBetweenFaces(F, G, allowSmooth, smoothAngularTol, commonEdges, angRad);
}

//-----------------------------------------------------------------------------

bool FR_CheckDihedralAngle::checkSeamVexity(const gp_Pnt&      pt,
                                                 const gp_Vec&      norm,
                                                 const TopoDS_Face& face,
                                                 bool&              isConcave) const
{
  std::optional<gp_Vec> axis;
  std::optional<gp_Pnt> origin;
  std::optional<double> diameter;
  bool                  isLabeled = false;

  /* Cylindrical surface */
  {
    Handle(Geom_CylindricalSurface) surfCyl;
    //
    if ( FR_Utils::IsTypeOf<Geom_CylindricalSurface>(face, surfCyl) )
    {
      axis      = surfCyl->Axis().Direction();
      origin    = surfCyl->Axis().Location();
      diameter  = 2*surfCyl->Radius();
      isLabeled = true;
    }
  }

  /* Conical surface */
  if ( !isLabeled )
  {
    Handle(Geom_ConicalSurface) coneSurf;
    //
    if ( FR_Utils::IsTypeOf<Geom_ConicalSurface>(face, coneSurf) )
    {
      gp_Cone cone = coneSurf->Cone();

      // Invert point on a cone.
      double ucone, vcone;
      ElSLib::ConeParameters( cone.Position(),
                              cone.RefRadius(),
                              cone.SemiAngle(),
                              pt,
                              ucone, vcone );

      Handle(Geom_Curve) viso = coneSurf->VIso(vcone);
      //
      if ( !viso->IsInstance( STANDARD_TYPE(Geom_Circle) ) )
        return false;

      axis      = coneSurf->Axis().Direction();
      origin    = coneSurf->Axis().Location();
      diameter  = 2*Handle(Geom_Circle)::DownCast(viso)->Radius();
      isLabeled = true;
    }
  }

  /* Toroidal surface */
  if ( !isLabeled )
  {
    Handle(Geom_ToroidalSurface) torusSurf;
    //
    if ( FR_Utils::IsTypeOf<Geom_ToroidalSurface>(face, torusSurf) )
    {
      axis      = torusSurf->Axis().Direction();
      origin    = torusSurf->Axis().Location();
      diameter  = 2*( torusSurf->MajorRadius() + torusSurf->MinorRadius() );
      isLabeled = true;
    }
  }

  /* Surface of revolution */
  if ( !isLabeled )
  {
    Handle(Geom_SurfaceOfRevolution) revolSurf;
    //
    if ( FR_Utils::IsTypeOf<Geom_SurfaceOfRevolution>(face, revolSurf) )
    {
      axis      = revolSurf->Axis().Direction();
      origin    = revolSurf->Axis().Location();
      isLabeled = true;

      // Derive the diameter from extreme values along generatrix.
      gp_Lin             lin(*origin, *axis);
      Handle(Geom_Curve) gnx      = revolSurf->BasisCurve();
      const double       radii[2] = { lin.Distance( gnx->Value( gnx->FirstParameter() ) ),
                                      lin.Distance( gnx->Value( gnx->LastParameter() ) ) };
      //
      diameter = 2*Max(radii[0], radii[1]);
    }
  }

  /* Sphere */
  if ( !isLabeled )
  {
    Handle(Geom_SphericalSurface) sphereSurf;
    //
    if ( FR_Utils::IsTypeOf<Geom_SphericalSurface>(face, sphereSurf) )
    {
      axis      = sphereSurf->Axis().Direction();
      origin    = sphereSurf->Axis().Location();
      diameter  = 2*sphereSurf->Radius();
      isLabeled = true;
    }
  }

  if ( !isLabeled )
    return false;

  // Take a probe point along the normal.
  gp_Pnt normProbe = pt.XYZ() + norm.Normalized().XYZ()*(*diameter)*0.05;

  // Compute the distance to the axis.
  gp_Lin axisLin(*origin, *axis);
  //
  const double probeDist = axisLin.Distance(normProbe);
  const double origDist  = axisLin.Distance(pt);
  //
  if ( probeDist < origDist )
  {
    isConcave = true;
  }
  else
  {
    isConcave = false;
  }

  return true;
}
