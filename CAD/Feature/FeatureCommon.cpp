#include "./FeatureCommon.h"

#include <assert.h>
#include <vector>

#include <Geom_Curve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Standard_Type.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS_Edge.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Tool.hxx>
#include <math_Matrix.hxx>
#include <math_Vector.hxx>
#include <math_Jacobi.hxx>
#include <Geom2d_Curve.hxx>
#include <math_SVD.hxx>
#include <gp_Pln.hxx>
#include <math_Gauss.hxx>
#include <GeomLProp_CLProps.hxx>
#include <math_GaussLeastSquare.hxx>
#include <GC_MakeCircle.hxx>
#include <TopExp_Explorer.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <TopoDS.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <ShapeFix_Edge.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_OffsetSurface.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <ShapeAnalysis_Edge.hxx>

double FeatureCommon::LargeAngle() {
    static double value = DegreesToRadians(165);

    return value;
}

// 计算边长度
double FeatureCommon::CalculateEdgeLength(const TopoDS_Edge& edge)
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

double FeatureCommon::CalculateFaceArea(const TopoDS_Face& face)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();
}

int FeatureCommon::DoubleCompare(const double a, const double b) {
    if (fabs(a - b) < Confusion())
    {
        return 0;
    }

    return a < b ? -1 : 1;
}

static bool GetCircle(const std::vector<gp_Pnt>& points, gp_Pnt& outCenter,
                                   double& outR)
{
    const int n = (int) points.size();
    if (n < 3)
    {
        return false;
    }

    
    GC_MakeCircle maker(points.front(), points[points.size() / 2], points.back());
    if (maker.IsDone())
    {
        gp_Circ circle = maker.Value()->Circ();

        outCenter = circle.Location();
        outR = circle.Radius();

        return true;
    }
    
    return false;
}

//对于一个环，首尾有共点
void FeatureCommon::SampleCurvePoints(const Handle(Geom_Curve) & C, double u0, double u1, int nSamples,
                              std::vector<gp_Pnt>& outPts,
                              std::vector<double>* outParams)
{
    outPts.clear();
    outPts.reserve(nSamples);
    if (outParams)
    {
        outParams->clear();
        outParams->reserve(nSamples);
    }

    for (int i = 0; i < nSamples; ++i)
    {
        const double t = (nSamples == 1) ? 0.0 : double(i) / double(nSamples - 1);
        const double u = u0 + (u1 - u0) * t;
        outPts.push_back(C->Value(u));
        if (outParams)
            outParams->push_back(u);
    }
}

// 用 SVD 拟合平面：法向 = 最小奇异值对应方向
static bool FitPlaneBySVD(const std::vector<gp_Pnt>& pts, gp_Pln& outPln, double& outMaxDist)
{
    outMaxDist = 0.0;
    if (pts.size() < 3)
        return false;

    // 1) 质心
    gp_XYZ c(0, 0, 0);
    for (const auto& p : pts)
        c += p.XYZ();
    c /= (double) pts.size();
    gp_Pnt center(c);

    // 2) 协方差矩阵 S = sum (d d^T)
    double sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
    for (const auto& p : pts)
    {
        gp_XYZ d = p.XYZ() - c;
        sxx += d.X() * d.X();
        sxy += d.X() * d.Y();
        sxz += d.X() * d.Z();
        syy += d.Y() * d.Y();
        syz += d.Y() * d.Z();
        szz += d.Z() * d.Z();
    }

    // 3) 对称 3x3 矩阵
    math_Matrix S(1, 3, 1, 3);
    S(1, 1) = sxx;
    S(1, 2) = sxy;
    S(1, 3) = sxz;
    S(2, 1) = sxy;
    S(2, 2) = syy;
    S(2, 3) = syz;
    S(3, 1) = sxz;
    S(3, 2) = syz;
    S(3, 3) = szz;

    // 4) Jacobi 求特征值/特征向量（专门针对对称矩阵） :contentReference[oaicite:2]{index=2}
    math_Jacobi jac(S);
    if (!jac.IsDone())
    {
        return false;
    }

    // 取最小特征值对应的特征向量作为平面法向
    // 注意：math_Jacobi 会对特征值排序（一般从小到大），保险起见我们自己找最小
    double evalMin = std::numeric_limits<double>::max();
    int idxMin = 1;
    for (int i = 1; i <= 3; ++i)
    {
        const double ev = jac.Value(i);  // 第 i 个特征值
        if (ev < evalMin)
        {
            evalMin = ev;
            idxMin = i;
        }
    }

    // 第 idxMin 个特征向量
    math_Vector v(1, 3);  // 长度3，索引1..3
    jac.Vector(idxMin, v);
    gp_Dir n(v(1), v(2), v(3));

    outPln = gp_Pln(center, n);

    // 5) 计算最大点到平面距离
    outMaxDist = 0.0;
    for (const auto& p : pts)
    {
        const double dist = std::abs(outPln.Distance(p));
        outMaxDist = std::max(outMaxDist, dist);
    }
    return true;
}

static bool CircleError(const std::vector<gp_Pnt>& p2, const gp_Pnt& C, double R, double tol)
{
    const double rate = 5.0 / 100;
    const double tol2 = tol * tol;
    const double R2 = R * R;
    for (auto& p : p2)
    {
        if (std::abs(p.SquareDistance(C) - R2) > tol2)
        {
            const double dis = p.Distance(C);
            if (fabs(dis - R) > rate * R)
            {
#ifdef _DEBUG
                printf("%f, %f\n", p.SquareDistance(C), R2);
#endif
                return false;
            }
        }
    }
    return true;
}

static gp_Vec TangentAt(const Handle(Geom_Curve) & c, double u)
{
    GeomLProp_CLProps props(c, u, 1, 1e-12);
    gp_Vec d1;
    return props.D1();
}

static double ParamOnCircle(const gp_Circ& C, const gp_Pnt& P)
{
    const gp_Pnt O = C.Location();
    const gp_Dir X = C.XAxis().Direction();
    const gp_Dir Y = C.YAxis().Direction();

    gp_Vec OP(O, P);

    double x = OP.Dot(gp_Vec(X));
    double y = OP.Dot(gp_Vec(Y));

    return std::atan2(y, x);  // 返回 [-pi, pi]
}

static bool BSplineCurveIsCircle(const Handle(Geom_BSplineCurve) & bc,
                                 Handle(Geom_Curve) & outCirc, double& u1, double& u2,
                                 const double tol)
{
    if (bc.IsNull())
    {
        return false;
    }
    double startParam = bc->FirstParameter();
    double endParam = bc->LastParameter();
    int nSamples = 25;

    // 1) 采样 3D 点
    std::vector<gp_Pnt> pts3;
    FeatureCommon::SampleCurvePoints(bc, startParam, endParam, nSamples, pts3);

    if (!pts3.empty())
    {
        if (pts3.back().IsEqual(pts3.front(), FeatureCommon::Confusion()))
        {
            pts3.pop_back();
        }
    }

    // 2) 拟合平面 + 共面误差
    gp_Pln pln;
    double maxPlanarDist = 0.0;
    if (!FitPlaneBySVD(pts3, pln, maxPlanarDist))
    {
        return false;
    }

    if (maxPlanarDist > tol)
    {
        return false;
    }

    // 3) 平面坐标系（Ax2）：原点取平面位置，X/Y 取平面基
    gp_Ax2 ax2(pln.Location(), pln.Axis().Direction());

    // 4) 先拟合圆
    gp_Pnt c3;
    double radius = 0.0;
    if (GetCircle(pts3, c3, radius))
    {
        // 圆弧验证（位置误差）
        if (CircleError(pts3, c3, radius, tol))
        {
            gp_Ax2 circAx2(c3, ax2.Direction(), ax2.XDirection());  // X 方向沿平面X
            gp_Circ circ(circAx2, radius);
            outCirc = new Geom_Circle(circ);

            double midParam = (startParam + endParam) * 0.5;
            gp_Vec tangent1 = TangentAt(bc, midParam);
            gp_Pnt refPt = bc->Value(midParam);

            double param3 = ParamOnCircle(circ, refPt);
            gp_Vec tangent2 = TangentAt(outCirc, param3);

            if (tangent1.Dot(tangent2) < 0.0)
            {
                // 翻转圆参数方向：Y轴取反（X轴不变）
                circAx2.SetYDirection(-circAx2.YDirection());
                circ = gp_Circ(circAx2, radius);
                outCirc = new Geom_Circle(circ);
            }

            double param1 = ParamOnCircle(circ, pts3[0]);
            double param2 = ParamOnCircle(circ, pts3[pts3.size() - 1]);
            if (param1 > param2)
            {
                param2 += M_PI * 2;
            }
            return true;
        }
    }
    return false;
}

bool FeatureCommon::IsCircle(const TopoDS_Edge& edge, gp_Ax1& outAxis, double& outRadius,
                             double& outConfusion)
{
    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);

    return IsCircle(curve, f, l, outAxis, outRadius, outConfusion);
}

/*
 * @brief judge if curve is circle, or circle like
 * @param curve
 * @param low param
 * @param high param
 * @param[out] axis
 * @param[out] radius
 * @param[out] confusion
 * @return true if curve is circle or circle like and calculate the direction, radius; false
 * otherwise
 */
bool FeatureCommon::IsCircle(Handle(Geom_Curve) curve, double low, double high, gp_Ax1& outAxis,
                     double& outRadius, double& outConfusion)
{
    if (curve.IsNull())
    {
        return false;
    }

    if (curve->DynamicType() == STANDARD_TYPE(Geom_Circle))
    {
        Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
        if (!circle.IsNull())
        {
            outAxis = circle->Axis();
            outRadius = circle->Radius();
            outConfusion = FeatureCommon::Confusion();
            return true;
        }
    }
    else if (curve->DynamicType() == STANDARD_TYPE(Geom_BSplineCurve))
    {
        // for some circle like curve, sampling point to check
        Handle(Geom_BSplineCurve) bc = Handle(Geom_BSplineCurve)::DownCast(curve);
        Handle(Geom_Circle) circle;
        double u1 = 0, u2 = 1;
        if (BSplineCurveIsCircle(bc, circle, u1, u2, 1e-1))
        {
            outConfusion = 1e-1;
            outAxis = circle->Axis();
            outRadius = circle->Radius();
            return true;
        }
    }
    else if (curve->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve))
    {
        Handle(Geom_TrimmedCurve) trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve);
        if (!trimmedCurve.IsNull())
        {
            Handle(Geom_Curve) basisCurve = trimmedCurve->BasisCurve();
            return IsCircle(basisCurve, low, high, outAxis, outRadius, outConfusion);
        }
    }
    return false;
}

// 角度转弧度
double FeatureCommon::DegreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// 弧度转角度
double FeatureCommon::RadiansToDegrees(double radians)
{
    return radians * 180.0 / M_PI;
}

double FeatureCommon::Angular() {
    //default tolerance as 5 degree;
    static double tolerance = DegreesToRadians(5.0);

    return tolerance;
}

double FeatureCommon::Confusion()
{
    return 1e-6;
}

bool FeatureCommon::IsCylindricalFace(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    return IsCylindricalSurface(surface);
}

bool FeatureCommon::IsCylindricalSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsCylindricalSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsOffsetSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_OffsetSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsOffsetSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsConicFace(const TopoDS_Face& face)
{
    if (face.IsNull())
    {
        return false;
    }

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull())
    {
        return false;
    }

    return IsConicalSurface(surface);
}

bool FeatureCommon::IsConicalSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_ConicalSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsConicalSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsPlane(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull())
    {
        return false;
    }

    return IsPlane(surface);
}

bool FeatureCommon::IsPlane(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_Plane)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsPlane(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsToroidalSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_ToroidalSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsToroidalSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsSurfaceOfRevolution(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_SurfaceOfRevolution)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsSurfaceOfRevolution(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsSphericalSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_SphericalSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsSphericalSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsBSplineSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsBSplineSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsBezierSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_BezierSurface)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsBezierSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}

bool FeatureCommon::IsExtrusionFace(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    if (surface.IsNull())
    {
        return false;
    }

    return IsExtrusionSurface(surface);
}

bool FeatureCommon::IsExtrusionSurface(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        return false;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_SurfaceOfLinearExtrusion)))
    {
        return true;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FeatureCommon::IsExtrusionSurface(trimmedSurface->BasisSurface());
        }
    }

    return false;
}


Handle(Geom_ToroidalSurface) FeatureCommon::FetchToroidalSurface(Handle(Geom_Surface) surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_ToroidalSurface)))
    {
        Handle(Geom_ToroidalSurface) torus = Handle(Geom_ToroidalSurface)::DownCast(surface);

        if (!torus)
        {
            assert(false);
        }
        return torus;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchToroidalSurface(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_BSplineSurface) FeatureCommon::FetchBsplineSurface(Handle(Geom_Surface) surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface)))
    {
        Handle(Geom_BSplineSurface) bspline = Handle(Geom_BSplineSurface)::DownCast(surface);

        if (!bspline)
        {
            assert(false);
        }
        return bspline;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchBsplineSurface(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_SphericalSurface) FeatureCommon::FetchSphericalSurface(Handle(Geom_Surface)
                                                                          surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_SphericalSurface)))
    {
        Handle(Geom_SphericalSurface) sphere =
            Handle(Geom_SphericalSurface)::DownCast(surface);

        if (!sphere)
        {
            assert(false);
        }
        return sphere;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchSphericalSurface(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_CylindricalSurface)
    FeatureCommon::FetchCylindricalSurface(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    if (surface.IsNull())
    {
        return nullptr;
    }

    return FetchCylindricalSurface(surface);
}

Handle(Geom_CylindricalSurface) FeatureCommon::FetchCylindricalSurface(Handle(Geom_Surface) surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
    {
        Handle(Geom_CylindricalSurface) cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface);

        if (!cylinder)
        {
            assert(false);
        }
        return cylinder;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchCylindricalSurface(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_Plane)
    FeatureCommon::FetchPlane(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    if (surface.IsNull())
    {
        return nullptr;
    }

    return FetchPlane(surface);
}

Handle(Geom_Plane) FeatureCommon::FetchPlane(Handle(Geom_Surface)
                                                                              surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_Plane)))
    {
        Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface);

        if (!plane)
        {
            assert(false);
        }
        return plane;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchPlane(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_ConicalSurface) FeatureCommon::FetchConicalSurface(const TopoDS_Face& face)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull())
    {
        return nullptr;
    }

    return FetchConicalSurface(surface);
}

Handle(Geom_ConicalSurface) FeatureCommon::FetchConicalSurface(Handle(Geom_Surface)
                                                                              surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_ConicalSurface)))
    {
        Handle(Geom_ConicalSurface) conic = Handle(Geom_ConicalSurface)::DownCast(surface);

        if (!conic)
        {
            assert(false);
        }
        return conic;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchConicalSurface(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

Handle(Geom_SurfaceOfRevolution)
    FeatureCommon::FetchSurfaceOfRevolution(Handle(Geom_Surface)
                                                                          surface)
{
    if (surface->IsKind(STANDARD_TYPE(Geom_SurfaceOfRevolution)))
    {
        Handle(Geom_SurfaceOfRevolution) revolution =
            Handle(Geom_SurfaceOfRevolution)::DownCast(surface);

        if (!revolution)
        {
            assert(false);
        }
        return revolution;
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return FetchSurfaceOfRevolution(trimmedSurface->BasisSurface());
        }
    }

    assert(false);

    return nullptr;
}

bool FeatureCommon::IsStraight(const TopoDS_Edge& edge, gp_Ax1& outAxis) {
    double f = 0;
    double l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);

    if (curve.IsNull())
    {
        return false;
    }

    gp_Dir dir;
    if (IsStraight(curve, f, l, dir))
    {
        outAxis = gp_Ax1(curve->Value(f), dir);
        return true;
    }

    return false;
}

/*
 * @brief judge if curve is straight, or straight like
 * @param curve
 * @param parameter range of low
 * @param parameter range of high
 * @param outDir the direction of straight
 * @return true if curve is straight or straight like and calculate the direction; false otherwise
 */
bool FeatureCommon::IsStraight(Handle(Geom_Curve) curve, double low, double high, gp_Dir& outDir)
{
    if (curve.IsNull())
    {
        return false;
    }

    if (curve->DynamicType() == STANDARD_TYPE(Geom_Line))
    {
        Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve);
        if (!line.IsNull())
        {
            outDir = line->Position().Direction();
            return true;
        }
    }
    // for some straight like curve, sampling point to check
    else if (curve->DynamicType() ==STANDARD_TYPE(Geom_BSplineCurve))
    {
        Handle(Geom_BSplineCurve) bc = Handle(Geom_BSplineCurve)::DownCast(curve);
        if (bc->Degree() == 1)
        {
            //linear
            outDir = gp_Dir(gp_Vec(bc->Value(low), bc->Value(high)));
            return true;
        }
        // #TODO for some straight like curve, sampling point to check
    }
    else if (curve->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve))
    {
        Handle(Geom_TrimmedCurve) trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve);
        if (!trimmedCurve.IsNull())
        {
            Handle(Geom_Curve) basisCurve = trimmedCurve->BasisCurve();
            return IsStraight(basisCurve, low, high, outDir);
        }
    }

    return false;
}

/**
 * @brief 判断两个轴是否共面
 * @param axis1 第一个轴
 * @param axis2 第二个轴
 * @param angularTol 角度容差，默认使用Precision::Angular()
 * @param linearTol 线性容差，默认使用Precision::Confusion()
 * @return true 如果两个轴共面
 */
bool FeatureCommon::IsAxesCoplanar(const gp_Ax1& axis1, const gp_Ax1& axis2,
                     double angularTol,
                     double linearTol)
{
    // 获取轴的位置和方向
    gp_Pnt p1 = axis1.Location();
    gp_Pnt p2 = axis2.Location();
    gp_Dir d1 = axis1.Direction();
    gp_Dir d2 = axis2.Direction();

    // 转换为向量
    gp_Vec v1(d1);
    gp_Vec v2(d2);

    // 计算方向向量的叉积
    gp_Vec cross = v1.Crossed(v2);
    double crossMagnitude = cross.Magnitude();

    // 检查是否平行（叉积模长为0表示平行或反平行）
    if (crossMagnitude <= angularTol)
    {
        return true;
    }
    else
    {
        //(v1 x v2) * (p2-p1)
        // 不平行：计算混合积
        gp_Vec connectVec(p1, p2);
        double mixedProduct = cross.Dot(connectVec);
        return (fabs(mixedProduct) <= linearTol);
    }

    return false;
}

bool FeatureCommon::IsBiCoaxial(const gp_Ax1& currentAxis, const gp_Ax1& axis,
                                double angularTol, double confusion)
{
    if (currentAxis.IsCoaxial(axis, angularTol, confusion))
    {
        return true;
    }

    gp_Ax1 anti(currentAxis.Location(), -currentAxis.Direction());

    if (anti.IsCoaxial(axis, angularTol, confusion))
    {
        return true;
    }

    return false;
}

// Calculate curvature using the coefficients of both fundamental forms
static double GetNormalCurvature(const gp_Vec2d& T, const double L, const double M,
                                 const double N, const double E, const double F, const double G)
{
    if (Abs(T.X()) < 1.0e-5)
    {
        return N / G;
    }
    else
    {
        const double lambda = T.Y() / T.X();
        return 
            (L + 2 * M * lambda + N * lambda * lambda) / (E + 2 * F * lambda + G * lambda * lambda);
    }

    return 0;
}

bool FeatureCommon::EvaluateCurvature(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                           const double t, gp_Pnt2d& UV, double& alongCurvature, double& perpendCurvature)
{
    // Get host geometries
    double f = 0, l = 1;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, f, l);
    BRepAdaptor_Surface surf(face);

    // Evaluate curve
    gp_Vec2d T;
    c2d->D1(t, UV, T);

    // Calculate differential properties
    BRepLProp_SLProps Props(surf, UV.X(), UV.Y(), 2, 1e-7);
    //
    if (!Props.IsCurvatureDefined())
    {
#if defined COUT_DEBUG
        std::cout << "Error: curvature is not defined" << std::endl;
#endif
        return false;
    }

    // Get differential properties
    const gp_Vec Xu = Props.D1U();
    const gp_Vec Xv = Props.D1V();
    const gp_Vec Xuu = Props.D2U();
    const gp_Vec Xuv = Props.DUV();
    const gp_Vec Xvv = Props.D2V();
    const gp_Vec n = Props.Normal();

    // Coefficients of the FFF
    const double E = Xu.Dot(Xu);
    const double F = Xu.Dot(Xv);
    const double G = Xv.Dot(Xv);

    // Coefficients of the SFF
    const double L = n.Dot(Xuu);
    const double M = n.Dot(Xuv);
    const double N = n.Dot(Xvv);

    alongCurvature = GetNormalCurvature(T, L, M, N, E, F, G);

    gp_Vec2d norm(T.Y(), T.X());

    perpendCurvature = GetNormalCurvature(norm, L, M, N, E, F, G);

    return true;
}


static bool MapEdgeToFace(TopoDS_Edge& edge, const TopoDS_Face& face)
{
    // 1. 检查是否已有pcurve
    double first, last;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);

    if (!pcurve.IsNull())
    {
        return true;  // 已存在pcurve
    }

    // 2. 自动构建pcurve
    ShapeFix_Edge sfe;
    sfe.FixAddPCurve(edge, face, false, Precision::Confusion());

    // 3. 再次检查
    pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
    if (!pcurve.IsNull())
    {
        return true;
    }

    // 4. 如果自动构建失败，手动构建
    // return BuildPCurveManually(edge, face);

    return false;
}

/**
 * @brief 计算面与平面的交线
 * @param face 面
 * @param sectionPlane 切割平面
 * @return Edge if calc success; null otherwise
 */
static TopoDS_Edge ComputeSection(const TopoDS_Face& face, const gp_Pln& sectionPlane)
{
    // 4. 计算面与平面的交线
    BRepAlgoAPI_Section sectionMaker(face, sectionPlane, false);
    sectionMaker.Build();

    if (!sectionMaker.IsDone())
    {
        //std::cerr << "错误：截面计算失败" << std::endl;
        return {};
    }

    TopoDS_Shape sectionShape = sectionMaker.Shape();

    // 5. 遍历截面结果，寻找圆弧边
    TopExp_Explorer edgeExplorer(sectionShape, TopAbs_EDGE);
    double totalAngle = 0.0;
    int arcCount = 0;

    while (edgeExplorer.More())
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

        // 获取边的几何曲线
        double firstParam = 0, lastParam = 1;
        Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, firstParam, lastParam);

        if (curve3d.IsNull())
        {
            edgeExplorer.Next();
            continue;
        }

        if (MapEdgeToFace(edge, face))
        {
            return edge;
        }

        edgeExplorer.Next();
    }

    // return null
    return {};
}

bool FeatureCommon::GetMidUVByCurve2d(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                      gp_Pnt2d& outUV)
{
    double f = 0, l = 1;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, f, l);

    if (c2d.IsNull())
    {
        return false;
    }

    const double t = (f + l) / 2;
    
    c2d->D0(t, outUV);
    
    return true;
    
}

/**
 * 将点投影到曲面，获取最近的UV坐标
 * @param point 要投影的3D点
 * @param surface 目标曲面
 * @param uv 输出的UV坐标
 * @param tolerance 容差
 * @return 投影是否成功
 */
bool FeatureCommon::ProjectPointToSurface(const gp_Pnt& point, const TopoDS_Face& face, gp_Pnt2d& outUV,
                           double tolerance)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull())
    {
        return false;
    }

    // 创建投影计算器
    GeomAPI_ProjectPointOnSurf projector(point, surface, tolerance);

    // 执行投影
    projector.Perform(point);

    if (projector.NbPoints() == 0)
    {
        return false;  // 没有找到投影点
    }

    // 获取最近投影点的UV坐标
    double u=0, v=0;
    projector.LowerDistanceParameters(u, v);
    outUV.SetCoord(u, v);

    return true;
}

bool FeatureCommon::EvaluateCurvatureAnalysis(const TopoDS_Face& face,
                                              const TopoDS_Edge& edge, double& alongCurvature,
                                              double& perpendCurvature)
{
    double f = 0, l = 0;
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    if (c.IsNull())
    {
        return false;
    }

    const double t = (f + l) / 2;

    gp_Pnt2d UVfeiwu;
    return EvaluateCurvatureAnalysis(face, edge, t, UVfeiwu, alongCurvature, perpendCurvature);
}

bool FeatureCommon::EvaluateCurvatureAnalysis(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                      const double t, gp_Pnt2d& UV, double& alongCurvature,
                                      double& perpendCurvature)
{
    // Get host geometries
    double f = 0, l = 1;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, f, l);
    Handle(Geom_Curve) c3d = BRep_Tool::Curve(edge, f, l);

    // Evaluate curve
    gp_Vec2d T;
    c2d->D1(t, UV, T);

    gp_Pnt P3d;
    gp_Vec T3d;
    c3d->D1(t, P3d, T3d);

    BRepAdaptor_Surface surf(face);
    // Calculate differential properties
    BRepLProp_SLProps Props(surf, UV.X(), UV.Y(), 2, 1e-7);

    #ifdef _DEBUG
    gp_Pnt CP = BRep_Tool::Surface(face)->Value(UV.X(), UV.Y());
    const double cmp = CP.Distance(P3d);
    printf("cmp = %.f\n", cmp);
    if (cmp > 1e-2)
    {
        std::cerr << "tol too big" << std::endl;
    }
    #endif

    //
    if (!Props.IsCurvatureDefined())
    {
#if defined COUT_DEBUG
        std::cout << "Error: curvature is not defined" << std::endl;
#endif
        return false;
    }

    // Get differential properties
    const gp_Vec Xu = Props.D1U();
    const gp_Vec Xv = Props.D1V();
    const gp_Vec Xuu = Props.D2U();
    const gp_Vec Xuv = Props.DUV();
    const gp_Vec Xvv = Props.D2V();
    const gp_Vec n = Props.Normal();

    const double angular = FeatureCommon::DegreesToRadians(1);

#ifdef _DEBUG

    assert(T3d.IsNormal(n, angular));
#endif  // _DEBUG

    // Coefficients of the FFF
    const double E = Xu.Dot(Xu);
    const double F = Xu.Dot(Xv);
    const double G = Xv.Dot(Xv);

    // Coefficients of the SFF
    const double L = n.Dot(Xuu);
    const double M = n.Dot(Xuv);
    const double N = n.Dot(Xvv);

    alongCurvature = GetNormalCurvature(T, L, M, N, E, F, G);

    // calculate norm curvature
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    gp_Pnt P;
    gp_Vec V1;
    c->D1(t, P, V1);
    // 创建垂直于法线的平面
    gp_Dir planeNormal(V1.X(), V1.Y(), V1.Z());  // 法线方向

    // 创建平面（需要一个点和一个法线方向）
    gp_Pln plane(P, planeNormal);

    TopoDS_Edge normEdge = ComputeSection(face, plane);

    if (normEdge.IsNull())
    {
        return false;
    }

    // Get host geometries
    Handle(Geom2d_Curve) c2dnorm = BRep_Tool::CurveOnSurface(normEdge, face, f, l);

    // 创建投影计算对象
    Geom2dAPI_ProjectPointOnCurve projector2d(UV, c2dnorm);

    Handle(Geom_Curve) c3dnorm = BRep_Tool::Curve(normEdge, f, l);

    if (projector2d.NbPoints() > 0)
    {
        std::cout << "2d nb = " << projector2d.NbPoints() << std::endl;
        // 获取投影点对应的参数
        double param = projector2d.LowerDistanceParameter();  // 最短距离对应的参数

        // 或者通过索引获取
        // double param = projector.Parameter(1);  // 第一个投影点参数

        // 验证投影点
        projector2d.NearestPoint();

        // 计算距离，判断点是否在曲线上（在容差范围内）
        double distance = projector2d.LowerDistance();
        if (distance < Precision::Confusion())
        {
            gp_Pnt2d UV1;
            gp_Vec2d norm;
            c2dnorm->D1(param, UV1, norm);

            c2dnorm->IsPeriodic();

#ifdef _DEBUG
            const double uvDis = UV1.Distance(UV);

            if (uvDis > Precision::Confusion())
            {
                int stop = 0;
            }

            assert(uvDis < Precision::Confusion());
#endif

            perpendCurvature = GetNormalCurvature(norm, L, M, N, E, F, G);

#ifdef _DEBUG

            gp_Pnt P3dnorm;
            gp_Vec T3dnorm;
            c3dnorm->D1(param, P3dnorm, T3dnorm);

            const double pdis = P3dnorm.Distance(P3d);
            if (pdis > Precision::Confusion())
            {
                int stop = 0;
            }

            if (pdis > 1e-3)
            {
                std::cerr << "big err = " << pdis << std::endl;
            }
            //assert(pdis < 1e-3);

            assert(T3dnorm.IsNormal(T3d, angular));

            assert(T3dnorm.IsNormal(n, angular));

#endif
        }
        else
        {
            int stop = 0;
        }
    }
    else
    {
        int stop = 0;
        GeomAPI_ProjectPointOnCurve projector3d(P3d, c3dnorm);
        if (projector3d.NbPoints() > 0)
        {
            std::cout << "3d nb = " << projector3d.NbPoints() << std::endl;
            // 获取投影点对应的参数
            double param = projector3d.LowerDistanceParameter();  // 最短距离对应的参数

            // 或者通过索引获取
            // double param = projector.Parameter(1);  // 第一个投影点参数

            // 验证投影点
            projector3d.NearestPoint();

            // 计算距离，判断点是否在曲线上（在容差范围内）
            double distance = projector3d.LowerDistance();
            if (distance > 1e-3)
            {
                std::cerr << "big error = " << distance << std::endl;
            }

            {
                gp_Pnt2d UV1;
                gp_Vec2d norm;
                c2dnorm->D1(param, UV1, norm);

#ifdef _DEBUG
                const double uvDis = UV1.Distance(UV);
                if (uvDis > 1e-3)
                {
                    std::cerr << "big error = " << uvDis << std::endl;
                }
#endif

                perpendCurvature = GetNormalCurvature(norm, L, M, N, E, F, G);

#ifdef _DEBUG

                gp_Pnt P3dnorm;
                gp_Vec T3dnorm;
                c3dnorm->D1(param, P3dnorm, T3dnorm);

                const double pdis = P3dnorm.Distance(P3d);
                if (pdis > Precision::Confusion())
                {
                    std::cerr << "big error = " << pdis << std::endl;
                }

                assert(T3dnorm.IsNormal(T3d, angular));

                assert(T3dnorm.IsNormal(n, angular));

#endif
            }
        }
    }

    return true;
}


TopoDS_Edge FeatureCommon::CreateOthorgnalCurveOnMidPoint(const TopoDS_Face& face,
                                              const TopoDS_Edge& edge)
{
    double f = 0, l = 0;
    // calculate norm curvature
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    const double t = (f + l) / 2;
    gp_Pnt P;
    gp_Vec V1;
    c->D1(t, P, V1);
    // 创建垂直于法线的平面
    gp_Dir planeNormal(V1.X(), V1.Y(), V1.Z());  // 法线方向

    // 创建平面（需要一个点和一个法线方向）
    gp_Pln plane(P, planeNormal);

    TopoDS_Edge normEdge = ComputeSection(face, plane);

    return normEdge;
}

void FeatureCommon::Print(const TColStd_PackedMapOfInteger& nums, const std::string& setName)
{
    printf("%s = {", setName.c_str());
    for (TColStd_PackedMapOfInteger::Iterator it(nums); it.More(); it.Next())
    {
        printf("%d, ", it.Key());
    }

    puts("}");
}

// 条件1：检查是否为单侧面
bool FeatureCommon::IsSingleSidedFace(const TopoDS_Face& face)
{
    // 在实体模型中，所有面都是单侧的
    // 双侧面通常出现在曲面模型中
    // 简化实现：检查面的朝向是否一致
    BRepAdaptor_Surface surface(face);

    // 获取面的边界框
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);

    if (bbox.IsVoid())
    {
        return false;
    }

    // 检查面是否有面积（排除退化面）
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    double area = props.Mass();

    return (area > FeatureCommon::Confusion());
}

bool FeatureCommon::HaveCommonVertex(const TopoDS_Edge& edge0, const TopoDS_Edge& edge1)
{
    ShapeAnalysis_Edge se;

    TopoDS_Vertex v00 = se.FirstVertex(edge0);
    TopoDS_Vertex v01 = se.LastVertex(edge0);

    TopoDS_Vertex v10 = se.FirstVertex(edge1);
    TopoDS_Vertex v11 = se.LastVertex(edge1);

    if (v00.IsSame(v10) || v00.IsSame(v11))
    {
        return true;
    }

    if (v01.IsSame(v10) || v01.IsSame(v11))
    {
        return true;
    }

    return false;

}


/**
 * 判断两个圆锥面是否几何上相同
 *
 * 圆锥面的几何定义由以下参数确定：
 * 1. 顶点（Apex）：轴上的参考点
 * 2. 轴方向（Axis Direction）
 * 3. 半角（Semi-Angle）
 * 4. 参考半径（RefRadius）：在顶点处的半径
 *
 * 注意：同一个几何圆锥面可以有不同的参数化表示
 * 例如，轴的原点（顶点）可以沿着轴移动，但必须保持几何形状不变
 */
static bool AreConicalSurfacesGeometricallyEqual(const Handle(Geom_ConicalSurface) & surface1,
                                                 const Handle(Geom_ConicalSurface) & surface2)
{
    if (surface1.IsNull() || surface2.IsNull())
    {
        return surface1.IsNull() && surface2.IsNull();
    }

    // 如果两个句柄指向同一个对象，它们当然是相同的
    if (surface1 == surface2)
    {
        return true;
    }

    // 获取两个圆锥面的几何定义
    const gp_Cone cone1 = surface1->Cone();
    const gp_Cone cone2 = surface2->Cone();

    // 获取圆锥面的轴
    const gp_Ax3 axis1 = cone1.Position();
    const gp_Ax3 axis2 = cone2.Position();

    // 1. 比较半角 - 必须精确相等
    const double semiAngle1 = cone1.SemiAngle();
    const double semiAngle2 = cone2.SemiAngle();

    if (Abs(semiAngle1 - semiAngle2) > Precision::Angular())
    {
        return false;
    }

    // 2. 比较轴的方向
    const gp_Dir dir1 = axis1.Direction();
    const gp_Dir dir2 = axis2.Direction();

    // 检查方向是否相同（考虑容差）
    const double dot = dir1.Dot(dir2);
    if (Abs(dot - 1.0) > Precision::Angular())
    {
        // 如果方向相反，但半角为0（退化情况），则仍可能是相同的
        if (semiAngle1 > Precision::Angular())
        {
            return false;
        }
        // 对于半角为0的情况（退化为圆柱），方向相反可能是相同的
        if (Abs(dot + 1.0) > Precision::Angular())
        {
            return false;
        }
    }

    // 3. 比较半径
    const double radius1 = cone1.RefRadius();
    const double radius2 = cone2.RefRadius();

    if (Abs(radius1 - radius2) > Precision::Confusion())
    {
        return false;
    }

    // 4. 比较轴的顶点位置
    // 对于圆锥面，顶点位置非常重要，因为半径是定义在顶点处的
    const gp_Pnt apex1 = axis1.Location();
    const gp_Pnt apex2 = axis2.Location();

    // 计算顶点之间的距离
    const double apexDistance = apex1.Distance(apex2);

    // 如果顶点距离很小，那么可以直接认为是同一个圆锥
    if (apexDistance <= Precision::Confusion())
    {
        return true;
    }

    // 5. 特殊情况：如果顶点不同，但圆锥是相同的几何形状
    // 这意味着两个圆锥的轴是平行的，且顶点沿着轴移动
    // 我们需要检查：从顶点1到顶点2的向量是否与轴方向平行
    gp_Vec vecApex1ToApex2(apex1, apex2);

    // 计算向量长度
    const double vecLength = vecApex1ToApex2.Magnitude();
    if (vecLength < Precision::Confusion())
    {
        return true;  // 实际上这应该被上面的顶点距离检查捕获
    }

    // 归一化向量
    gp_Dir dirApex1ToApex2 = vecApex1ToApex2;

    // 检查向量是否与轴方向平行
    const double parallelTest = dirApex1ToApex2.Dot(dir1);
    if (Abs(Abs(parallelTest) - 1.0) > Precision::Angular())
    {
        return false;  // 顶点不在同一条轴线上
    }

    // 6. 如果顶点在同一条轴线上，检查两个圆锥是否几何相同
    // 对于圆锥面，如果顶点沿着轴移动，半径会按比例缩放
    // 半径变化 = 距离 * tan(半角)
    const double expectedRadiusChange = vecLength * tan(semiAngle1);

    // 如果方向相同，半径应该增加
    if (parallelTest > 0)
    {
        const double expectedRadius2 = radius1 + expectedRadiusChange;
        if (Abs(radius2 - expectedRadius2) > Precision::Confusion())
        {
            return false;
        }
    }
    else
    {
        // 方向相反，半径应该减小
        const double expectedRadius2 = radius1 - expectedRadiusChange;
        if (Abs(radius2 - expectedRadius2) > Precision::Confusion())
        {
            return false;
        }
    }

    return true;
}

bool FeatureCommon::IsSameConic(const TopoDS_Face& face, const TopoDS_Face& partner)
{
    if (FeatureCommon::IsConicFace(face) && FeatureCommon::IsConicFace(partner))
    {
        Handle(Geom_ConicalSurface) c0 = FeatureCommon::FetchConicalSurface(face);
        Handle(Geom_ConicalSurface) c1 = FeatureCommon::FetchConicalSurface(partner);

        return AreConicalSurfacesGeometricallyEqual(c0, c1);
    }

    return false;
}

// 获取面上某点的法向量
static gp_Dir GetFaceNormalAtMidPoint(const TopoDS_Face& face, const TopoDS_Edge& edge)
{
    // Get host geometries
    double f = 0, l = 1;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, f, l);
    BRepAdaptor_Surface surf(face);

    const double t = (f + l) / 2;
    gp_Pnt2d UV;
    // Evaluate curve
    gp_Vec2d T;
    c2d->D1(t, UV, T);

    // Calculate differential properties
    BRepLProp_SLProps Props(surf, UV.X(), UV.Y(), 2, 1e-7);

    if (!Props.IsCurvatureDefined())
    {
        return gp_Dir(0, 0, 1);  // 返回默认法向量
    }

    gp_Dir normal = Props.Normal();
    if (face.Orientation() == TopAbs_FORWARD)
    {
        return normal;
    }
    else
    {
        return -normal;
    }
}

std::vector<TopoDS_Edge> FeatureCommon::FindCommonEdges(const TopoDS_Face& face1,
                                            const TopoDS_Face& face2)
{  // 找到两个面的公共边
    std::vector<TopoDS_Edge> commonEdges;

    // 获取face1的所有边
    TopTools_IndexedMapOfShape edges1;
    TopExp::MapShapes(face1, TopAbs_EDGE, edges1);

    // 获取face2的所有边
    TopTools_IndexedMapOfShape edges2;
    TopExp::MapShapes(face2, TopAbs_EDGE, edges2);

    // 找到公共边
    for (int i = 1; i <= edges1.Extent(); i++)
    {
        TopoDS_Edge edge1 = TopoDS::Edge(edges1(i));

        for (int j = 1; j <= edges2.Extent(); j++)
        {
            TopoDS_Edge edge2 = TopoDS::Edge(edges2(j));

            if (edge1.IsSame(edge2))
            {
                commonEdges.push_back(edge1);
                break;
            }
        }
    }

    return commonEdges;
}

// 计算两个面之间的夹角
double FeatureCommon::CalculateAngleBetweenFaces(const TopoDS_Face& face1,
                                                          const TopoDS_Face& face2,
                                                          gp_Dir& outNormal1,
                                                          gp_Dir& outNormal2)
{
    const std::vector<TopoDS_Edge> commonEdges = FindCommonEdges(face1, face2);
    if (commonEdges.empty())
    {
        return 0.0;  // 没有公共边
    }

    // 使用第一条公共边的中点计算法向量
    TopoDS_Edge commonEdge = commonEdges[0];

    // 计算两个面在该点的法向量
    outNormal1 = GetFaceNormalAtMidPoint(face1, commonEdge);
    outNormal2 = GetFaceNormalAtMidPoint(face2, commonEdge);

    #ifdef _DEBUG
    //printf("norm1 = (%.3f, %.3f, %.3f)\n", outNormal1.X(), outNormal1.Y(), outNormal1.Z());
    //printf("norm2 = (%.3f, %.3f, %.3f)\n", outNormal2.X(), outNormal2.Y(), outNormal2.Z());
    #endif

    // 计算夹角
    double dotProduct = outNormal1.Dot(outNormal2);
    dotProduct = std::max(-1.0, std::min(1.0, dotProduct));  // 限制在[-1, 1]范围内

    return acos(dotProduct);
}
