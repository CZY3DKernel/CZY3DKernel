#include "./CylinderBlendRecognizer.h"

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
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Handle.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <gp_Cylinder.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pln.hxx>
#include <BRepAlgoAPI_Section.hxx>

#include "./BlendPropertyFEConstRadius.h"
#include "./BlendPropertyFFConstRadius.h"
#include "./BlendProperty.h"
#include "./AttributeAdjacentGraph.h"
#include "./SmoothEdgesFinder.h"
#include "./SupportFacesFinder.h"
#include "./FeatureCommon.h"

static double GetCylinderRadius(Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinder =
        FeatureCommon::FetchCylindricalSurface(surface);

    if (!cylinder)
    {
        assert(false);
        return 0;
    }

    return cylinder->Radius();
}

static bool SameCylinder(const TopoDS_Face& face, const TopoDS_Face& partner) {
    if (FeatureCommon::IsCylindricalFace(face) && FeatureCommon::IsCylindricalFace(partner))
    {
        Handle(Geom_CylindricalSurface) c0 = FeatureCommon::FetchCylindricalSurface(face);
        Handle(Geom_CylindricalSurface) c1 = FeatureCommon::FetchCylindricalSurface(partner);

        if (c0.IsNull() || c1.IsNull())
        {
            return false;
        }

        if (FeatureCommon::DoubleCompare(c0->Radius(), c1->Radius()) != 0)
        {
            return false;
        }

        if (!FeatureCommon::IsBiCoaxial(c0->Axis(), c1->Axis(), FeatureCommon::Angular(),
                                       FeatureCommon::Confusion()))
        {
            return false;
        }

        return true;

    }

    return false;
}

TColStd_PackedMapOfInteger CylinderBlendRecognizer::FindSpringEdges(
    const TColStd_PackedMapOfInteger& smoothEdgeIndices, const TopoDS_Face& face, Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        FeatureCommon::FetchCylindricalSurface(surface);
    
    if (!cylinderSurface)
    {
        return {};
    }

    ShapeAnalysis_Edge sa;

    gp_Ax1 axis = cylinderSurface->Axis();
    const double radius = cylinderSurface->Radius();

    TColStd_PackedMapOfInteger result;

    //iterator all edges and find straight and parallel with cylinder axis
    for (TColStd_PackedMapOfInteger::Iterator iter(smoothEdgeIndices); iter.More(); iter.Next())
    {
        const int edgeId = iter.Key();
        TopoDS_Edge edge = m_aag->GetEdge(edgeId);

        TopoDS_Face partner = m_aag->GetNeibourFace(face, edge);
        if (!partner.IsNull() && SameCylinder(face, partner))
        {
            continue;
        }

        if (sa.IsSeam(edge, face))
        {
            //self seam self
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
            if (dir.IsParallel(axis.Direction(), FeatureCommon::Angular()))
            {
                if (FeatureCommon::IsCylindricalFace(partner))
                {
                    bool ok = true;
                    //partner radius should larger than current
                    double f = 0, l = 0;
                    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);

                    if (!curve.IsNull())
                    {
                        const double t = (f + l) / 2;
                        gp_Pnt2d uv;
                        double alongCurvature = 0;
                        double perpendCurvature = 0;
                        if (FeatureCommon::EvaluateCurvatureAnalysis(
                                partner, edge, t, uv, alongCurvature, perpendCurvature))
                        {
                            if (fabs(perpendCurvature) > Precision::Confusion())
                            {
                                const double r = 1 / fabs(perpendCurvature);

                                if (FeatureCommon::DoubleCompare(r, radius) < 0)
                                {
                                    ok = false;
                                }
                                // #TODO r==radius
                            }
                        }
                    }

                    if (ok)
                    {
                        result.Add(edgeId);
                    }
                }
                else if (FeatureCommon::IsExtrusionFace(partner))
                {
                    bool ok = true;
                    double along = 0;
                    double perpend = 0;
                    if (FeatureCommon::EvaluateCurvatureAnalysis(partner, edge, along, perpend))
                    {
                        if (fabs(perpend) > FeatureCommon::Confusion())
                        {
                            const double r = 1 / fabs(perpend);

                            if (FeatureCommon::DoubleCompare(radius, r) >= 0)
                            {
                                ok = false;
                            }
                        }
                    }

                    if (ok)
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

TColStd_PackedMapOfInteger CylinderBlendRecognizer::FindCliffEdges(
    const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        FeatureCommon::FetchCylindricalSurface(surface);

    if (!cylinderSurface)
    {
        return {};
    }

    gp_Ax1 axis = cylinderSurface->Axis();

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
            if (dir.IsParallel(axis.Direction(), Precision::Angular()))
            {
                result.Add(edgeId);
            }
        }
    }

    return result;
}

TColStd_PackedMapOfInteger CylinderBlendRecognizer::FindCrossEdges(
    const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        FeatureCommon::FetchCylindricalSurface(surface);

    if (cylinderSurface.IsNull())
    {
        return {};
    }

    gp_Ax1 axis = cylinderSurface->Axis();

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
            // 共轴且半径相等
            
            if (FeatureCommon::IsBiCoaxial(currentAxis, axis, FeatureCommon::Angular(),
                                           confusion))
            {
                if (std::fabs(currentRadius - cylinderSurface->Radius()) < confusion)
                {
                    result.Add(edgeId);
                }
            }
        }
    }

    return result;
}

/**
 * @brief 计算圆柱面被垂直于轴线的平面切割后得到的圆弧圆心角
 * @param cylinderFace 圆柱面
 * @param sectionPlane 切割平面（必须垂直于圆柱轴线）
 * @param angleInDegrees 是否返回角度值（true返回度，false返回弧度）
 * @return 圆弧的圆心角，如果失败返回-1
 */
static double ComputeArcAngleFromCylinderSection(const TopoDS_Face& cylinderFace,
                                          const gp_Pln& sectionPlane)
{
    // 1. 验证输入面是否为圆柱面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(cylinderFace);
    if (surface.IsNull())
    {
        //std::cerr << "错误：无法获取面的几何表面" << std::endl;
        return -1.0;
    }

    // 尝试转换为圆柱面
    Handle(Geom_CylindricalSurface) cylinderSurface =
        FeatureCommon::FetchCylindricalSurface(surface);

    if (cylinderSurface.IsNull())
    {
        assert(false);
        // 如果不是圆柱面，尝试其他类型的柱面
        //std::cerr << "错误：输入面不是圆柱面" << std::endl;
        return -1.0;
    }

    // 2. 获取圆柱的几何参数
    gp_Cylinder cylinder = cylinderSurface->Cylinder();
    gp_Ax3 axis = cylinder.Position();
    gp_Dir axisDir = axis.Direction();  // 圆柱轴线方向
    double radius = cylinder.Radius();  // 圆柱半径

    // 3. 验证切割平面是否垂直于圆柱轴线
    gp_Dir planeNormal = sectionPlane.Axis().Direction();
    double dotProduct = axisDir.Dot(planeNormal);

    //垂直的话，应该是叉积==0
    // 检查垂直性（点积应为0，考虑容差）
    if (fabs(dotProduct) > Precision::Angular())
    {
        //std::cerr << "警告：切割平面不垂直于圆柱轴线，点积=" << dotProduct << std::endl;
        //std::cerr << "角度偏差：" << acos(fabs(dotProduct)) * 180.0 / M_PI << "度" << std::endl;
        // 可以继续计算，但结果可能不是圆弧而是椭圆弧
    }

    // 4. 计算面与平面的交线
    BRepAlgoAPI_Section sectionMaker(cylinderFace, sectionPlane, false);
    sectionMaker.Build();

    if (!sectionMaker.IsDone())
    {
        //std::cerr << "错误：截面计算失败" << std::endl;
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
                //std::cerr << "警告：截面圆半径(" << circleRadius << ")与圆柱半径(" << radius
                       //   << ")不一致" << std::endl;
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

bool CylinderBlendRecognizer::HasSeamEdge(const TColStd_PackedMapOfInteger& smoothEdgeIndices, const TopoDS_Face& face)
{
    ShapeAnalysis_Edge ae;
    for (TColStd_PackedMapOfInteger::Iterator it(smoothEdgeIndices); it.More(); it.Next())
    {
        const TopoDS_Edge edge = m_aag->GetEdge(it.Key());
        if (ae.IsSeam(edge, face))
        {
            return true;
        }
    }

    return false;
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
    Handle(Geom_CylindricalSurface) cylinderSurface =
        FeatureCommon::FetchCylindricalSurface(surface);

    if (cylinderSurface.IsNull())
    {
        throw Standard_TypeMismatch("input is not cylinder");
    }

    // 获取圆柱参数
    gp_Cylinder cylinder = cylinderSurface->Cylinder();
    gp_Ax3 axis = cylinder.Position();
    //gp_Pnt origin = axis.Location();  // 圆柱坐标系原点
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

    return ComputeArcAngleFromCylinderSection(face, plane);
}

bool CylinderBlendRecognizer::CheckRadiusRegular(
    const double radius, const TopoDS_Face& face,
                        const TColStd_PackedMapOfInteger& springEdgeIndices)
{
    for (TColStd_PackedMapOfInteger::Iterator it(springEdgeIndices); it.More(); it.Next())
    {
        const int edgeId = it.Key();
        const TopoDS_Edge& edge = m_aag->GetEdge(edgeId);
        const TopoDS_Face& partner = m_aag->GetNeibourFace(face, edge);

        if (FeatureCommon::IsCylindricalFace(partner))
        {
            double f = 0, l = 0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);

            if (!curve.IsNull())
            {
                const double t = (f + l) / 2;
                gp_Pnt2d uv;
                double alongCurvature = 0;
                double perpendCurvature = 0;
                if (FeatureCommon::EvaluateCurvatureAnalysis(partner, edge, t, uv, alongCurvature,
                                                             perpendCurvature))
                {
                    if (fabs(perpendCurvature) > Precision::Confusion())
                    {
                        const double r = 1 / fabs(perpendCurvature);

                        if (FeatureCommon::DoubleCompare(r, radius) < 0)
                        {
                            return false;
                        }
                        //#TODO r==radius
                    }
                }
            }
        }
    }

    return true;
}

/**
 * 判断两个平面是否几何共面
 * @param plane1 第一个平面
 * @param plane2 第二个平面
 * @param angularTol 角度容差（弧度），默认使用Precision::Angular()
 * @param linearTol 距离容差，默认使用Precision::Confusion()
 * @return 如果共面返回true
 */
bool ArePlanesCoincident(const Handle(Geom_Plane) & plane1, const Handle(Geom_Plane) & plane2,
                         double angularTol, double linearTol)
{
    if (plane1.IsNull() || plane2.IsNull())
    {
        return false;
    }

    // 如果是同一个对象，肯定共面
    if (plane1 == plane2)
    {
        return true;
    }

    // 获取gp_Pln对象
    const gp_Pln& pln1 = plane1->Pln();
    const gp_Pln& pln2 = plane2->Pln();

    // 1. 检查法向量是否平行
    const gp_Dir& normal1 = pln1.Axis().Direction();
    const gp_Dir& normal2 = pln2.Axis().Direction();

    // 法向量平行：点积绝对值接近1
    if (!normal1.IsParallel(normal2, angularTol))
    {
        return false;  // 法向量不平行
    }

    // 2. 检查一个平面上的点到另一个平面的距离
    // 获取平面1的位置（平面上的一个点）
    const gp_Pnt& loc1 = pln1.Location();

    // 计算点loc1到平面2的距离
    double distance = pln2.Distance(loc1);

    // 距离在容差范围内则为共面
    return distance <= linearTol;
}

static bool IsSameGeometry(const TopoDS_Face& a, const TopoDS_Face& b) {
    if (FeatureCommon::IsPlane(a) && FeatureCommon::IsPlane(b))
    {
        Handle(Geom_Plane) p0 = FeatureCommon::FetchPlane(a);
        Handle(Geom_Plane) p1 = FeatureCommon::FetchPlane(b);

        return ArePlanesCoincident(p0, p1, FeatureCommon::Angular(), FeatureCommon::Confusion());
    }

    return false;
}

static std::vector<TopoDS_Face> MergeFaces(
    const TColStd_PackedMapOfInteger& supportFaceIndices,
    std::shared_ptr<AttributeAdjacentGraph> aag)
{
    std::vector<TopoDS_Face> faces;

    for (TColStd_PackedMapOfInteger::Iterator it(supportFaceIndices); it.More(); it.Next())
    {
        const TopoDS_Face face = aag->GetFace(it.Key());

        bool found = false;
        for (const TopoDS_Face& f : faces)
        {
            if (IsSameGeometry(f, face))
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            faces.push_back(face);
        }
    }

    return faces;
}

/*
* @brief judge if face is Cylindrical Blend
* @note it will judge if face is cylindrical by assertting
* @param face
* @param[out] outInfo, extract blend info into outInfo
* @param return true if face is Cylindrical Blend
*/
bool CylinderBlendRecognizer::IsBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo) {

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
#ifdef _DEBUG
    assert(FeatureCommon::IsCylindricalSurface(surface));
#endif

    const double radius = GetCylinderRadius(surface);
    
    //1. find smooth edges
    SmoothEdgesFinder smoothEdgesFinder(m_aag);
    TColStd_PackedMapOfInteger smoothEdgeIndices = smoothEdgesFinder.PerformForFace(face);

    if (smoothEdgeIndices.IsEmpty())
    {
        return false;
    }

    if (HasSeamEdge(smoothEdgeIndices, face))
    {
        return false;
    }

    //2. find spring edges based on smooth edges
    TColStd_PackedMapOfInteger springEdgeIndices =
        FindSpringEdges(smoothEdgeIndices, face, surface);

    //3. find support face by spring edges
    SupportFacesFinder supportFacesFinder(m_aag);
    TColStd_PackedMapOfInteger supportFaceIndices =
        supportFacesFinder.PerformForFace(face, springEdgeIndices);

    std::vector<TopoDS_Face> mergedFaces = MergeFaces(supportFaceIndices, m_aag);

    const BlendFType blendType = DetermineBlendTypeBySpringNums(mergedFaces.size());
    if (BlendFType::BlendType_Uncertain == blendType)
    {
        return false;
    }

    //4. find cross edges, must be smooth edge and radius is equal to cylinder radius
    TColStd_PackedMapOfInteger crossEdgeIndices = smoothEdgeIndices;
    crossEdgeIndices.Subtract(springEdgeIndices);
    //crossEdgeIndices = FindCrossEdges(crossEdgeIndices, surface);

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

    TColStd_PackedMapOfInteger::Iterator it(springEdgeIndices);

    const TopoDS_Edge& springEdge = m_aag->GetEdge(it.Key());

    const double angle = GetAngle(face, springEdge);
    if (angle > FeatureCommon::LargeAngle())
    {
        return false;
    }

    const int faceId = m_aag->GetFaceId(face);

    if (cliffEdgeIndices.IsEmpty())
    {
        //ordinary
        outInfo = std::make_shared<BlendPropertyFFConstRadius>(
            faceId, radius, angle, smoothEdgeIndices, springEdgeIndices, crossEdgeIndices,
            terminatingEdgeIndices, supportFaceIndices);
    }
    else
    {
        //cliff edge blend
        outInfo = std::make_shared<BlendPropertyFEConstRadius>(
            faceId, radius, angle, cliffEdgeIndices, smoothEdgeIndices, springEdgeIndices,
            crossEdgeIndices, terminatingEdgeIndices, supportFaceIndices);
    }

    return nullptr != outInfo;
}
