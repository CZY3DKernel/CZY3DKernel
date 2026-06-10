#include "./ToroidalBlendRecognizer.h"

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
#include <Geom_ToroidalSurface.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <gp_Torus.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pln.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Standard_Type.hxx>

#include "./BlendPropertyFEConstRadius.h"
#include "./BlendPropertyFFConstRadius.h"
#include "./BlendProperty.h"
#include "./AttributeAdjacentGraph.h"
#include "./SmoothEdgesFinder.h"
#include "./SupportFacesFinder.h"
#include "./FeatureCommon.h"

static double GetToroidalMinorRadius(Handle(Geom_Surface) surface)
{
    if (surface.IsNull())
    {
        assert(false);
        return 0;
    }

    if (surface->IsKind(STANDARD_TYPE(Geom_ToroidalSurface)))
    {
        Handle(Geom_ToroidalSurface) torus = Handle(Geom_ToroidalSurface)::DownCast(surface);

        if (!torus)
        {
            assert(false);
            return 0;
        }
        return torus->MinorRadius();
    }
    else if (surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
    {
        Handle(Geom_RectangularTrimmedSurface) trimmedSurface =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);

        if (!trimmedSurface.IsNull())
        {
            return GetToroidalMinorRadius(trimmedSurface->BasisSurface());
        }
    }

    assert(false);
    return 0;

}

TColStd_PackedMapOfInteger ToroidalBlendRecognizer::FindSpringEdges(
    const TColStd_PackedMapOfInteger& smoothEdgeIndices, Handle(Geom_Surface) surface, const TopoDS_Face& face)
{
    Handle(Geom_ToroidalSurface) toroidalSurface = FeatureCommon::FetchToroidalSurface(surface);
    
    if (!toroidalSurface)
    {
        return {};
    }

    gp_Ax1 axis = toroidalSurface->Axis();
    const double radius = toroidalSurface->MinorRadius();

    TColStd_PackedMapOfInteger result;

    //iterator all edges and find straight and parallel with cylinder axis
    for (TColStd_PackedMapOfInteger::Iterator iter(smoothEdgeIndices); iter.More(); iter.Next())
    {
        const int edgeId = iter.Key();
        TopoDS_Edge edge = m_aag->GetEdge(edgeId);

        double l = 0, h = 1;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, l, h);

        if (curve.IsNull())
        {
            continue;
        }

        double tempRadius = 0;
        gp_Ax1 currentAxis;
        double confusion = FeatureCommon::Confusion();
        if (FeatureCommon::IsCircle(curve, l, h, currentAxis, tempRadius, confusion))
        {
            if (FeatureCommon::IsBiCoaxial(currentAxis, axis, FeatureCommon::Angular(), confusion))
            {
                result.Add(edgeId);
            }
            else if (currentAxis.IsParallel(axis, FeatureCommon::Angular()))
            {
                // for some circle like curve, sampling point to check
                double startParam = curve->FirstParameter();
                double endParam = curve->LastParameter();
                int nSamples = 25;

                // 1) 采样 3D 点
                std::vector<gp_Pnt> pts3;
                FeatureCommon::SampleCurvePoints(curve, startParam, endParam, nSamples, pts3);

                bool ok = true;

                double minDis = DBL_MAX;
                double maxDis = 0;
                gp_Lin line(axis);
                for (auto p : pts3)
                {
                    const double dis = line.Distance(p);
                    if (dis > maxDis)
                    {
                        maxDis = dis;
                    }
                    
                    if (dis < minDis)
                    {
                        minDis = dis;
                    }
                }

                const double rate = 5.0 / 100;
                if (maxDis - minDis > rate * maxDis)
                {
                    ok = false;
                }

                if (ok)
                {
                    const TopoDS_Face partner = m_aag->GetNeibourFace(face, edge);

                    double along = 0, perpend = 0;
                    FeatureCommon::EvaluateCurvatureAnalysis(partner, edge, along, perpend);

                    if (fabs(perpend) > FeatureCommon::Confusion())
                    {
                        const double r = 1 / fabs(perpend);
                        if (FeatureCommon::DoubleCompare(radius, r) >= 0)
                        {
                            ok = false;
                        }
                    }

                    if (ok)
                    {
                        result.Add(edgeId);
                    }
                }

            }
        }
    }
    
    return result;
}

TColStd_PackedMapOfInteger ToroidalBlendRecognizer::FindCliffEdges(
    const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    Handle(Geom_Surface) surface)
{
    Handle(Geom_ToroidalSurface) toroidalSurface = FeatureCommon::FetchToroidalSurface(surface);

    if (!toroidalSurface)
    {
        return {};
    }

    gp_Ax1 axis = toroidalSurface->Axis();

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

        double tempRadius = 0;
        gp_Ax1 currentAxis;
        double confusion = FeatureCommon::Confusion();
        if (FeatureCommon::IsCircle(curve, l, h, currentAxis, tempRadius, confusion))
        {
            if (FeatureCommon::IsBiCoaxial(currentAxis, axis, FeatureCommon::Angular(), confusion))
            {
                result.Add(edgeId);
            }
        }
    }

    return result;
}

TColStd_PackedMapOfInteger ToroidalBlendRecognizer::FindCrossEdges(
    const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface)
{
    Handle(Geom_ToroidalSurface) toroidalSurface = FeatureCommon::FetchToroidalSurface(surface);

    if (toroidalSurface.IsNull())
    {
        return {};
    }

    gp_Ax1 axis = toroidalSurface->Axis();

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
            // 轴与大圆的轴垂直，且半径相等
            if (currentAxis.IsNormal(axis, FeatureCommon::Angular()))
            {
                if (std::fabs(currentRadius - toroidalSurface->MinorRadius()) < confusion)
                {
                    result.Add(edgeId);
                }
            }
        }
    }

    return result;
}

/**
 * @brief 计算圆环面被经过轴线的平面切割后得到的圆弧圆心角
 * @param toroidalFace 圆柱面
 * @param sectionPlane 切割平面（必须垂直于圆柱轴线）
 * @param angleInDegrees 是否返回角度值（true返回度，false返回弧度）
 * @return 圆弧的圆心角，如果失败返回-1
 */
static double ComputeArcAngleFromTorusSection(const TopoDS_Face& torusFace,
                                          const gp_Pln& sectionPlane)
{
    // 1. 验证输入面是否为圆柱面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(torusFace);
    if (surface.IsNull())
    {
        //std::cerr << "错误：无法获取面的几何表面" << std::endl;
        return -1.0;
    }

    // 尝试转换为圆柱面
    Handle(Geom_ToroidalSurface) torusSurface = FeatureCommon::FetchToroidalSurface(surface);

    if (torusSurface.IsNull())
    {
        assert(false);
        // 如果不是圆柱面，尝试其他类型的柱面
        //std::cerr << "错误：输入面不是圆柱面" << std::endl;
        return -1.0;
    }

    // 2. 获取圆柱的几何参数
    gp_Torus torus = torusSurface->Torus();
    gp_Ax3 axis = torus.Position();
    gp_Dir axisDir = axis.Direction();  // 圆柱轴线方向
    double radius = torus.MinorRadius();  // 圆柱半径

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
    BRepAlgoAPI_Section sectionMaker(torusFace, sectionPlane, false);
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
                         // << ")不一致" << std::endl;
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

        //对于圆环面，切出来可能有多条，取其中一条即可。
        if (arcCount > 0)
        {
            break;
        }
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
 * @brief 创建经过大圆轴线的切割平面
 * @param torusFace 圆柱面
 * @param planeLocation 平面上的一点
 * @return 切割平面
 */
static gp_Pln CreatePerpendicularSectionPlane(const TopoDS_Face& torusFace,
                                          const gp_Pnt& planeLocation)
{
    // 获取圆柱面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(torusFace);
    Handle(Geom_ToroidalSurface) toroidalSurface = FeatureCommon::FetchToroidalSurface(surface);

    if (toroidalSurface.IsNull())
    {
        throw Standard_TypeMismatch("input is not cylinder");
    }

    // 获取圆柱参数
    gp_Torus torus = toroidalSurface->Torus();
    gp_Ax3 axis = torus.Position();
    
    gp_Dir axisDir = axis.Direction();  // 圆柱轴线方向

    gp_Dir planeNormal(gp_Vec(axis.Axis().Location(), planeLocation));

    planeNormal.Cross(axisDir);

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

    return ComputeArcAngleFromTorusSection(face, plane);
}

static BlendFType DetermineToroidalBlendTypeByNums(const int num)
{
    BlendFType blendType = BlendFType::BlendType_Uncertain;
    switch (num)
    {
        case 0:
        {
            return BlendFType::BlendType_Uncertain;
        }
        case 1:  // cliff blend
        {
            return BlendFType::BlendType_Cliff;
        }
        case 2:  // ordinary
        {
            return BlendFType::BlendType_Ordinary;
        }        

        //> 3
        default:
            return BlendFType::BlendType_MultiToMulti;
    }

    return BlendFType::BlendType_Uncertain;
}

/*
* @brief judge if face is toroidal Blend
* @note it will judge if face is toroidal by assertting
* @param face
* @param[out] outInfo, extract blend info into outInfo
* @param return true if face is toroidal Blend
*/
bool ToroidalBlendRecognizer::IsBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo) {

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
#ifdef _DEBUG
    assert(FeatureCommon::IsToroidalSurface(surface));
#endif

    const double radius = GetToroidalMinorRadius(surface);
    
    //1. find smooth edges
    SmoothEdgesFinder smoothEdgesFinder(m_aag);
    TColStd_PackedMapOfInteger smoothEdgeIndices = smoothEdgesFinder.PerformForFace(face);

    //FeatureCommon::Print(smoothEdgeIndices, "smoothEdgeIndices");

    if (smoothEdgeIndices.IsEmpty())
    {
        return false;
    }

    //2. find spring edges based on smooth edges
    TColStd_PackedMapOfInteger springEdgeIndices = FindSpringEdges(smoothEdgeIndices, surface, face);

    //3. find support face by spring edges
    SupportFacesFinder supportFacesFinder(m_aag);
    TColStd_PackedMapOfInteger supportFaceIndices =
        supportFacesFinder.PerformForFace(face, springEdgeIndices);

    //FeatureCommon::Print(supportFaceIndices, "supportFaceIndices");

    const BlendFType blendType = DetermineToroidalBlendTypeByNums(supportFaceIndices.Extent());
    if (BlendFType::BlendType_Uncertain == blendType)
    {
        return false;
    }

    //4. find cross edges, must be smooth edge and radius is equal to cylinder radius
    TColStd_PackedMapOfInteger crossEdgeIndices = smoothEdgeIndices;
    crossEdgeIndices.Subtract(springEdgeIndices);
    
    //TColStd_PackedMapOfInteger crossEdgeIndices = FindCrossEdges(lestSmoothEdges, surface);
    
    //5. find cliff edge, parallel with axis, but not smooth edge
    TColStd_PackedMapOfInteger cliffEdgeIndices = FindCliffEdges(face, smoothEdgeIndices, surface);
    
    //6. find terminating edges, all edges except smooth edges, spring edges, cross edges and cliff edges
    TColStd_PackedMapOfInteger terminatingEdgeIndices = m_aag->GetEdgeIds(face);

    //except (smooth edge, spring edge, cross edge), cliff edge, spring edge and cross edge are smooth
    
    terminatingEdgeIndices.Subtract(smoothEdgeIndices);

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
