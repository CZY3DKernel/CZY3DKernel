#include "./ConicBlendRecognizer.h"

/*
曲面类型   类型名称                     是否为解析曲面
平面       Geom_Plane                     是
圆柱面     Geom_CylindricalSurface        是
圆锥面     Geom_ConicalSurface            是
球面       Geom_SphericalSurface          是
圆环面     Geom_ToroidalSurface           是
B样条曲面  Geom_BSplineSurface            否
贝塞尔曲面 Geom_BezierSurface             否
旋转曲面   Geom_SurfaceOfRevolution       可能
拉伸曲面   Geom_SurfaceOfLinearExtrusion  可能
裁剪曲面   Geom_RectangularTrimmedSurface 否
偏移曲面   Geom_OffsetSurface             否
*/

#include <assert.h>

#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Handle.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_ConicalSurface.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <gp_Cylinder.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pln.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <gp_Cone.hxx>

#include "./BlendPropertyFEVariousRadius.h"
#include "./BlendPropertyFFVariousRadius.h"
#include "./BlendProperty.h"
#include "./AttributeAdjacentGraph.h"
#include "./SmoothEdgesFinder.h"
#include "./SupportFacesFinder.h"
#include "./FeatureCommon.h"


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

static bool IsSameConic(const TopoDS_Face& face, const TopoDS_Face& partner)
{
    if (FeatureCommon::IsConicFace(face) && FeatureCommon::IsConicFace(partner))
    {
        Handle(Geom_ConicalSurface) c0 = FeatureCommon::FetchConicalSurface(face);
        Handle(Geom_ConicalSurface) c1 = FeatureCommon::FetchConicalSurface(partner);

        return AreConicalSurfacesGeometricallyEqual(c0, c1);
    }

    return false;
}

TColStd_PackedMapOfInteger ConicBlendRecognizer::FindSpringEdges(
    const TColStd_PackedMapOfInteger& smoothEdgeIndices, Handle(Geom_Surface) surface, const TopoDS_Face& face)
{
    Handle(Geom_ConicalSurface) conicSurface = FeatureCommon::FetchConicalSurface(surface);
    
    if (!conicSurface)
    {
        return {};
    }

    gp_Ax1 axis = conicSurface->Axis();
    
    TColStd_PackedMapOfInteger result;

    //iterator all edges and find straight and parallel with cylinder axis
    for (TColStd_PackedMapOfInteger::Iterator iter(smoothEdgeIndices); iter.More(); iter.Next())
    {
        const int edgeId = iter.Key();
        TopoDS_Edge edge = m_aag->GetEdge(edgeId);
        const TopoDS_Face& partner = m_aag->GetNeibourFace(face, edge);
        if (IsSameConic(face, partner))
        {
            continue;
        }

        double l = 0, h = 1;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, l, h);

        if (curve.IsNull())
        {
            continue;
        }
        
        gp_Dir dir;
        if (FeatureCommon::IsStraight(curve, l, h, dir))
        {
            gp_Ax1 axis1(curve->Value(l), dir);
            if (FeatureCommon::IsAxesCoplanar(axis, axis1))
            {
                if (FeatureCommon::IsConicFace(partner))
                {
                    double t = (l + h) / 2;
                    gp_Pnt2d UV;
                    double alongCurvature0 = 0;
                    double perpendCurvature0 = 0;
                    FeatureCommon::EvaluateCurvatureAnalysis(face, edge, t, UV, alongCurvature0,
                                                             perpendCurvature0);

                    double alongCurvature1 = 0;
                    double perpendCurvature1 = 0;
                    FeatureCommon::EvaluateCurvatureAnalysis(partner, edge, t, UV, alongCurvature1,
                                                             perpendCurvature1);

                    if (FeatureCommon::DoubleCompare(fabs(perpendCurvature0), fabs(perpendCurvature1)) > 0)
                    {
                        result.Add(edgeId);
                    }
                }
                else
                {
                    result.Add(edgeId);
                }
            }
        }
    }
    
    return result;
}

TColStd_PackedMapOfInteger ConicBlendRecognizer::FindCliffEdges(
    const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    Handle(Geom_Surface) surface)
{
    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(surface);

    if (!conicalSurface)
    {
        return {};
    }

    gp_Ax1 axis = conicalSurface->Axis();

    TColStd_PackedMapOfInteger result;

    // iterator all edges of face and find straight and parallel with cylinder axis
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        const int edgeId = m_aag->GetEdgeId(edge);

        if (smoothEdgeIndices.Contains(edgeId))
        {
            continue;
        }

        double l = 0, h = 1;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, l, h);

        if (curve.IsNull())
        {
            continue;
        }

        gp_Dir dir;
        if (FeatureCommon::IsStraight(curve, l, h, dir))
        {
            gp_Ax1 axis1(curve->Value(l), dir);
            if (FeatureCommon::IsAxesCoplanar(axis, axis1))
            {
                result.Add(edgeId);
            }
        }
    }

    return result;
}

TColStd_PackedMapOfInteger ConicBlendRecognizer::FindCrossEdges(
    const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface)
{
    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(surface);

    if (conicalSurface.IsNull())
    {
        return {};
    }

    gp_Ax1 axis = conicalSurface->Axis();

    TColStd_PackedMapOfInteger result;

    // iterator all edges and find straight and parallel with cylinder axis
    for (TColStd_PackedMapOfInteger::Iterator iter(lestSmoothEdges); iter.More(); iter.Next())
    {
        const int edgeId = iter.Key();
        TopoDS_Edge edge = m_aag->GetEdge(edgeId);

        double l = 0, h = 1;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, l, h);

        if (curve.IsNull())
        {
            continue;
        }

        gp_Ax1 currentAxis;
        double currentRadius = 0;
        double confusion = FeatureCommon::Confusion();
        if (FeatureCommon::IsCircle(curve, l, h, currentAxis, currentRadius, confusion))
        {
            // 共轴
            if (FeatureCommon::IsBiCoaxial(currentAxis, axis, FeatureCommon::Angular(), confusion))
            {
                result.Add(edgeId);   
            }
        }
    }

    return result;
}

/**
 * @brief 计算圆锥面被垂直于轴线的平面切割后得到的圆弧圆心角
 * @param conicFace 圆锥面
 * @param sectionPlane 切割平面（必须垂直于圆柱轴线）
 * @param 对应的radius
 * @return 圆弧的圆心角，如果失败返回-1
 */
static double ComputeArcAngleFromConicSection(const TopoDS_Face& cylinderFace,
                                          const gp_Pln& sectionPlane, const double radius/*hai mei use*/)
{
    // 1. 验证输入面是否为圆柱面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(cylinderFace);
    if (surface.IsNull())
    {
        std::cerr << "错误:无法获取面的几何表面" << std::endl;
        return -1.0;
    }

    // 尝试转换为圆柱面
    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(surface);

    if (conicalSurface.IsNull())
    {
        assert(false);
        // 如果不是圆柱面，尝试其他类型的柱面
        std::cerr << "错误:输入面不是圆柱面" << std::endl;
        return -1.0;
    }

    // 2. 获取圆柱的几何参数
    gp_Ax1 axis = conicalSurface->Axis();
    gp_Dir axisDir = axis.Direction();  // 圆柱轴线方向

    // 3. 验证切割平面是否垂直于圆柱轴线
    gp_Dir planeNormal = sectionPlane.Axis().Direction();
    double dotProduct = axisDir.Dot(planeNormal);

    //垂直的话，应该是叉积==0
    // 检查垂直性（点积应为0，考虑容差）
    if (fabs(dotProduct) > Precision::Angular())
    {
        std::cerr << "警告:切割平面不垂直于圆柱轴线,点积=" << dotProduct << std::endl;
        //std::cerr << "角度偏差:" << acos(fabs(dotProduct)) * 180.0 / M_PI << "度" << std::endl;
        // 可以继续计算，但结果可能不是圆弧而是椭圆弧
    }

    // 4. 计算面与平面的交线
    BRepAlgoAPI_Section sectionMaker(cylinderFace, sectionPlane, false);
    sectionMaker.Build();

    if (!sectionMaker.IsDone())
    {
        std::cerr << "错误:截面计算失败" << std::endl;
        return -1.0;
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

        // 检查曲线类型
        Handle(Geom_Circle) circle;
        Handle(Geom_TrimmedCurve) trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve3d);

        if (!trimmedCurve.IsNull())
        {
            // 如果是裁剪曲线，获取基础曲线
            Handle(Geom_Curve) baseCurve = trimmedCurve->BasisCurve();
            circle = Handle(Geom_Circle)::DownCast(baseCurve);

            // 如果是圆弧，获取参数范围
            if (!circle.IsNull())
            {
                firstParam = trimmedCurve->FirstParameter();
                lastParam = trimmedCurve->LastParameter();
            }
        }
        else
        {
            // 直接检查是否为圆
            circle = Handle(Geom_Circle)::DownCast(curve3d);
        }

        // 6. 如果是圆或圆弧，计算圆心角
        if (!circle.IsNull())
        {
            // 获取圆的参数
            double circleRadius = circle->Radius();

            // 验证半径是否与圆柱半径一致（考虑容差）
            if (fabs(circleRadius - radius) > Precision::Confusion())
            {
                //std::cerr << "警告:截面圆半径(" << circleRadius << ")与圆柱半径(" << radius << ")不一致" << std::endl;
            }

            // 计算圆心角
            double angleSpan = lastParam - firstParam;

            // 确保角度在合理范围内
            if (angleSpan < 0)
            {
                angleSpan += 2 * M_PI;
            }

            // 对于闭合圆，参数范围可能是[0, 2π]
            if (angleSpan > 2 * M_PI - Precision::PConfusion() &&
                angleSpan < 2 * M_PI + Precision::PConfusion())
            {
                angleSpan = 2 * M_PI;
            }

            std::cout << "找到圆弧段：" << std::endl;
            std::cout << "  起始参数: " << firstParam << " 弧度" << std::endl;
            std::cout << "  结束参数: " << lastParam << " 弧度" << std::endl;
            std::cout << "  圆心角: " << angleSpan << " 弧度 (" << angleSpan * 180.0 / M_PI
                      << " 度)" << std::endl;
            std::cout << "  半径: " << circleRadius << std::endl;

            totalAngle += angleSpan;
            arcCount++;
        }
        else
        {
            // 如果不是圆弧，可能是直线段或其他曲线
            std::cerr << "警告：截面包含非圆弧曲线" << std::endl;
        }

        edgeExplorer.Next();
    }

    if (arcCount == 0)
    {
        std::cerr << "错误：截面中没有找到圆弧" << std::endl;
        return -1.0;
    }

    // 7. 返回圆心角
    return totalAngle;
}

/**
 * @brief 创建垂直于圆柱轴线的切割平面
 * @param cylinderFace 圆柱面
 * @param planeLocation 平面上的一点
 * @return 切割平面
 */
static gp_Pln CreatePerpendicularSectionPlane(const TopoDS_Face& cylinderFace,
                                          const gp_Pnt& planeLocation)
{
    // 获取圆柱面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(cylinderFace);
    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(surface);

    if (conicalSurface.IsNull())
    {
        throw Standard_TypeMismatch("input is not cylinder");
    }

    // 获取圆柱参数
    gp_Ax3 axis = conicalSurface->Position();
    gp_Dir axisDir = axis.Direction();  // 圆柱轴线方向

    // 创建垂直于轴线的平面
    // 平面法线方向与轴线方向相同
    gp_Dir planeNormal = axisDir;

    // 创建平面（需要一个点和一个法线方向）
    return gp_Pln(planeLocation, planeNormal);
}

static double GetAngle(const TopoDS_Face& face, const TopoDS_Edge& springEdge) {
    double firstParam = 0, lastParam = 1;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(springEdge, firstParam, lastParam);

    if (curve.IsNull())
    {
        return 0;
    }

    const double midParam = (firstParam + lastParam) / 2;

    gp_Pnt planeLocation = curve->Value(midParam);

    gp_Pln plane = CreatePerpendicularSectionPlane(face, planeLocation);

    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(BRep_Tool::Surface(face));

    if (conicalSurface.IsNull())
    {
        return -1;
    }

    gp_Lin line(conicalSurface->Axis());

    const double radius = line.Distance(planeLocation);

    return ComputeArcAngleFromConicSection(face, plane, radius);
}

/*
* @brief 计算最小半径， 最大半径，平均半径
* @param face
* @param spring edge
* @param[out] outMinRadius
* @param[out] outMaxRadius
* @param[out] outAvgRadius
*/
void CalcRadii(const TopoDS_Face& face, const TopoDS_Edge& springEdge, double& outMinRadius,
               double& outMaxRadius, double& outAvgRadius)
{
    outMinRadius = DBL_MAX;
    outMaxRadius = 0;
    outAvgRadius = 0;
    Handle(Geom_ConicalSurface) conicalSurface =
        FeatureCommon::FetchConicalSurface(BRep_Tool::Surface(face));

    if (conicalSurface.IsNull())
    {
        return;
    }

    gp_Lin line(conicalSurface->Axis());

    TopoDS_Vertex v0, v1;
    TopExp::Vertices(springEdge, v0, v1);

    gp_Pnt p0 = BRep_Tool::Pnt(v0);

    const double r0 = line.Distance(p0);

    gp_Pnt p1 = BRep_Tool::Pnt(v1);
    const double r1 = line.Distance(p1);

    outAvgRadius = (r0 + r1) / 2;

    outMinRadius = std::min(r0, r1);
    outMaxRadius = std::max(r0, r1);
}

// 辅助结构，表示圆的信息
struct CircleInfo
{
    double confusion = 0;
    gp_Pnt center;
    double radius = 0;
    gp_Dir normal;  // 圆所在平面的法向
    bool valid = false;

    // 标准化法向，确保同一几何圆有唯一的表示
    void normalizeNormal()
    {
        // 将法向的X分量作为主要判断依据，确保方向一致性
        if (normal.X() < 0)
        {
            normal = gp_Dir(-normal.X(), -normal.Y(), -normal.Z());
        }
        else if (std::abs(normal.X()) < FeatureCommon::Confusion() && normal.Y() < 0)
        {
            normal = gp_Dir(-normal.X(), -normal.Y(), -normal.Z());
        }
        else if (std::abs(normal.X()) < FeatureCommon::Confusion() &&
                 std::abs(normal.Y()) < FeatureCommon::Confusion() && normal.Z() < 0)
        {
            normal = gp_Dir(-normal.X(), -normal.Y(), -normal.Z());
        }
    }

    // 重载小于运算符，用于set排序
    bool operator<(const CircleInfo& other) const
    {
        // 先比较圆心
        double dx = center.X() - other.center.X();
        if (std::abs(dx) > FeatureCommon::Confusion())
        {
            return dx < 0;
        }

        double dy = center.Y() - other.center.Y();
        if (std::abs(dy) > FeatureCommon::Confusion())
        {
            return dy < 0;
        }

        double dz = center.Z() - other.center.Z();
        if (std::abs(dz) > FeatureCommon::Confusion())
        {
            return dz < 0;
        }

        // 圆心相同，比较半径
        double dr = radius - other.radius;
        if (std::abs(dr) > FeatureCommon::Confusion())
        {
            return dr < 0;
        }

        // 圆心和半径相同，比较法向
        double dot = normal.Dot(other.normal);
        if (std::abs(dot - 1.0) > FeatureCommon::Angular())
        {
            // 法向不同，需要比较方向
            // 比较X分量
            double nxDiff = normal.X() - other.normal.X();
            if (std::abs(nxDiff) > FeatureCommon::Confusion())
            {
                return nxDiff < 0;
            }

            // 比较Y分量
            double nyDiff = normal.Y() - other.normal.Y();
            if (std::abs(nyDiff) > FeatureCommon::Confusion())
            {
                return nyDiff < 0;
            }

            // 比较Z分量
            double nzDiff = normal.Z() - other.normal.Z();
            if (std::abs(nzDiff) > FeatureCommon::Confusion())
            {
                return nzDiff < 0;
            }
        }

        return false;  // 完全相同
    }

    // 判断两个圆是否相同（考虑容差）
    bool isSameCircle(const CircleInfo& other) const
    {
        const double maxC = std::max(other.confusion, confusion);
        // 检查圆心距离
        if (center.Distance(other.center) > maxC)
        {
            return false;
        }

        // 检查半径
        if (std::abs(radius - other.radius) > maxC)
        {
            return false;
        }

        // 检查法向：要么相同，要么相反（同一个几何圆）
        double dot = normal.Dot(other.normal);
        return std::abs(std::abs(dot) - 1.0) <= FeatureCommon::Angular();
    }
};

// 从边中提取圆信息
static CircleInfo ExtractCircleInfo(const TopoDS_Edge& edge)
{
    CircleInfo info;
    info.valid = false;

    gp_Ax1 outAxis;
    double outRadius = 0;
    double outConfusion = 0;

    if (!FeatureCommon::IsCircle(edge, outAxis, outRadius, outConfusion))
    {
        return info;
    }
    

    // 提取圆的信息
    info.center = outAxis.Location();          // 圆心
    info.radius = outRadius;            // 半径
    info.normal = outAxis.Direction();  // 法向
    info.confusion = outConfusion;
    info.normalizeNormal();                    // 标准化法向
    info.valid = true;

    return info;
}

// 判断是否存在至少两个不同的圆
static bool HasTwoCircles(const std::vector<TopoDS_Edge>& edges)
{
    // 存储已识别的不同圆
    std::vector<CircleInfo> uniqueCircles;

    for (const auto& edge : edges)
    {
        // 提取圆信息
        CircleInfo info = ExtractCircleInfo(edge);

        if (!info.valid)
        {
            continue;  // 不是圆弧，跳过
        }

        // 检查这个圆是否与已有圆相同
        bool isNewCircle = true;
        for (const auto& existingCircle : uniqueCircles)
        {
            if (info.isSameCircle(existingCircle))
            {
                isNewCircle = false;
                break;
            }
        }

        if (isNewCircle)
        {
            uniqueCircles.push_back(info);

            // 如果已找到至少两个不同的圆，返回true
            if (uniqueCircles.size() >= 2)
            {
                return true;
            }
        }
    }

    return false;
}

static bool HasTwoCircles(std::shared_ptr<AttributeAdjacentGraph> aag,
                          const TColStd_PackedMapOfInteger& otherEdges)
{
    std::vector<TopoDS_Edge> edges;
    for (TColStd_PackedMapOfInteger::Iterator it(otherEdges); it.More(); it.Next())
    {
        edges.push_back(aag->GetEdge(it.Key()));
    }

    return HasTwoCircles(edges);
}

/*
* @brief judge if face is Cylindrical Blend
* @note it will judge if face is cylindrical by assertting
* @param face
* @param[out] outInfo, extract blend info into outInfo
* @param return true if face is Cylindrical Blend
*/
bool ConicBlendRecognizer::IsBlend(const TopoDS_Face& face,
                                   std::shared_ptr<BlendProperty>& outInfo)
{

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
#ifdef _DEBUG
    assert(FeatureCommon::IsConicalSurface(surface));
#endif
    
    //1. find smooth edges
    SmoothEdgesFinder smoothEdgesFinder(m_aag);
    TColStd_PackedMapOfInteger smoothEdgeIndices = smoothEdgesFinder.PerformForFace(face);

    if (smoothEdgeIndices.IsEmpty())
    {
        return false;
    }

    //2. find spring edges based on smooth edges
    TColStd_PackedMapOfInteger springEdgeIndices = FindSpringEdges(smoothEdgeIndices, surface, face);

    const BlendFType blendType = DetermineBlendTypeBySpringNums(springEdgeIndices.Extent());
    if (BlendFType::BlendType_Uncertain == blendType)
    {
        return false;
    }

    //3. find support face by spring edges
    SupportFacesFinder supportFacesFinder(m_aag);
    TColStd_PackedMapOfInteger supportFaceIndices =
        supportFacesFinder.PerformForFace(face, springEdgeIndices);

    //4. find cross edges, must be smooth edge and radius is equal to cylinder radius
    TColStd_PackedMapOfInteger crossEdgeIndices = FindCrossEdges(smoothEdgeIndices, surface);
    
    //5. find cliff edge, parallel with axis, but not smooth edge
    TColStd_PackedMapOfInteger cliffEdgeIndices = FindCliffEdges(face, smoothEdgeIndices, surface);

    //6. find terminating edges, all edges except smooth edges, spring edges, cross edges and cliff edges
    TColStd_PackedMapOfInteger terminatingEdgeIndices = m_aag->GetEdgeIds(face);

    //except (spring edge, cross edge), cliff edge, spring edge and cross edge are smooth
    
    terminatingEdgeIndices.Subtract(springEdgeIndices);
    terminatingEdgeIndices.Subtract(crossEdgeIndices);

    if (!cliffEdgeIndices.IsEmpty())
    {
        terminatingEdgeIndices.Subtract(cliffEdgeIndices);
    }

    TColStd_PackedMapOfInteger otherEdges;
    otherEdges.Union(crossEdgeIndices, terminatingEdgeIndices);

    if (!HasTwoCircles(m_aag, otherEdges))
    {
        return false;
    }

    TColStd_PackedMapOfInteger::Iterator it(springEdgeIndices);

    const TopoDS_Edge& springEdge = m_aag->GetEdge(it.Key());

    const double angle = GetAngle(face, springEdge);

    if (angle > FeatureCommon::LargeAngle())
    {
        return false;
    }

    const int faceId = m_aag->GetFaceId(face);

    double maxRadius = 0;
    double minRadius = 0;
    double avgRadius = 0;

    CalcRadii(face, springEdge, minRadius, maxRadius, avgRadius);

    if (cliffEdgeIndices.IsEmpty())
    {
        //ordinary
        outInfo = std::make_shared<BlendPropertyFFVariousRadius>(
            faceId, maxRadius, minRadius, avgRadius, angle, smoothEdgeIndices, springEdgeIndices,
            crossEdgeIndices, terminatingEdgeIndices, supportFaceIndices);
    }
    else
    {
        //cliff edge blend
        outInfo = std::make_shared<BlendPropertyFEVariousRadius>(
            faceId, maxRadius, minRadius, avgRadius, angle, cliffEdgeIndices, smoothEdgeIndices,
            springEdgeIndices, crossEdgeIndices, terminatingEdgeIndices, supportFaceIndices);
    }

    return nullptr != outInfo;
}
