#include "./VertexBlendRecognizer.h"

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
#include <ShapeAnalysis_Surface.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Geom_SphericalSurface.hxx>

#include "./BlendPropertyVertexConstRadius.h"
#include "./BlendPropertyVertexVariousRadius.h"
#include "./BlendProperty.h"
#include "./AttributeAdjacentGraph.h"
#include "./SmoothEdgesFinder.h"
#include "./SupportFacesFinder.h"
#include "./FeatureCommon.h"
#include "./SpringEdgesFinder.h"
#include "./CrossEdgesFinder.h"
#include "./CliffEdgesFinder.h"
#include "./CheckDihedralAngle.h"

static double GetSphereRadius(Handle(Geom_Surface) surface)
{
    Handle(Geom_SphericalSurface) sphere = FeatureCommon::FetchSphericalSurface(surface);

    if (!sphere)
    {
        assert(false);
        return 0;
    }

    return sphere->Radius();
}

/*
* @brief judge if curve is straight, or straight like
* @param curve
* @param parameter range of low
* @param parameter range of high
* @param outDir the direction of straight
* @return true if curve is straight or straight like and calculate the direction; false otherwise
*/
static bool IsStraight(Handle(Geom_Curve) curve, double low, double high, gp_Dir& outDir) {
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
    else if (curve->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve))
    {
        Handle(Geom_TrimmedCurve) trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve);
        if (!trimmedCurve.IsNull())
        {
            Handle(Geom_Curve) basisCurve = trimmedCurve->BasisCurve();
            return IsStraight(basisCurve, low, high, outDir);
        }
    }

    //#TODO for some straight like curve, sampling point to check

    return false;
}

TColStd_PackedMapOfInteger VertexBlendRecognizer::FindSpringEdges(
    const TColStd_PackedMapOfInteger& smoothEdgeIndices, Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        Handle(Geom_CylindricalSurface)::DownCast(surface);
    
    if (!cylinderSurface)
    {
        return {};
    }

    gp_Ax1 axis = cylinderSurface->Axis();

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

        gp_Dir dir;
        if (IsStraight(curve, l, h, dir))
        {
            if (dir.IsParallel(axis.Direction(), Precision::Angular()))
            {
                result.Add(edgeId);
            }
        }
    }
    
    return result;
}

TColStd_PackedMapOfInteger VertexBlendRecognizer::FindCliffEdges(
    const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        Handle(Geom_CylindricalSurface)::DownCast(surface);

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
        if (IsStraight(curve, l, h, dir))
        {
            if (dir.IsParallel(axis.Direction(), Precision::Angular()))
            {
                result.Add(edgeId);
            }
        }
    }

    return result;
}

TColStd_PackedMapOfInteger VertexBlendRecognizer::FindCrossEdges(
    const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface)
{
    Handle(Geom_CylindricalSurface) cylinderSurface =
        Handle(Geom_CylindricalSurface)::DownCast(surface);

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
            if (FeatureCommon::IsBiCoaxial(currentAxis, axis, FeatureCommon::Angular(), confusion))
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
        Handle(Geom_CylindricalSurface)::DownCast(surface);

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
                 //         << ")不一致" << std::endl;
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
    Handle(Geom_CylindricalSurface) cylinderSurface =
        Handle(Geom_CylindricalSurface)::DownCast(surface);

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

/*
* @brief calculate the minRadius, maxRadius, avgRadius of all spring edges
* @param aag the attribute adjacent graph
* @param spring edge indices
* @param[out] outMinRadius
* @param [out] outMaxRadius
* @param [out] outAvgRadius
*/
static void CalcRadii(const std::shared_ptr<AttributeAdjacentGraph>& aag,
    const TopoDS_Face& face,
    const TColStd_PackedMapOfInteger& springEdgeIndices, 
    double &outMinRadius, double &outMaxRadius, double &outAvgRadius) {

    outMinRadius = DBL_MAX;
    outMaxRadius = 0;
    outAvgRadius = 0;

    double totRadius = 0;
    int cnt = 0;

    BRepAdaptor_Surface AS(face, false);

    //iterate spring edges
    for (TColStd_PackedMapOfInteger::Iterator it(springEdgeIndices); it.More(); it.Next())
    {
        const int edgeId = it.Key();
        const TopoDS_Edge& edge = aag->GetEdge(edgeId);

        double low = 0, high = 1;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, low, high);

        if (curve.IsNull())
        {
            continue;
        }

        const int sampleCount = 10;
        const double delta = (high - low) / sampleCount;
        double param = low;
        for (int i = 0; i <= sampleCount; ++i, param += delta)
        {
            gp_Pnt P;
            gp_Vec T;
            curve->D1(param, P, T);

            if (T.Magnitude() < RealEpsilon())
            {
                continue;
            }

            gp_Pnt2d UV;
            {
                ShapeAnalysis_Surface SAS(AS.Surface().Surface());
                UV = SAS.ValueOfUV(P, 1.0e-2);
            }

            BRepLProp_SLProps AProps(AS, UV.X(), UV.Y(), 2, 1.0e-5);

            if (!AProps.IsCurvatureDefined())
            {
                continue;
            }

            gp_Dir A_minD, A_maxD;
            AProps.CurvatureDirections(A_maxD, A_minD);

            const double A_minK = AProps.MinCurvature();
            const double A_maxK = AProps.MaxCurvature();

            //T对应a1
            double a2 = 0;
            if (Abs(A_minD.Dot(T)) > Abs(A_maxD.Dot(T)))
            {
                a2 = A_maxK;
            }
            else
            {
                a2 = A_minK;
            }

            if (Abs(a2) > Precision::Confusion())
            {
                double tempRadius = 1.0 / Abs(a2);

                totRadius += tempRadius;
                ++cnt;

                outMaxRadius = std::max(outMaxRadius, tempRadius);
                outMinRadius = std::min(outMinRadius, tempRadius);
            }


        }
    }

    outAvgRadius = totRadius / cnt;
}

bool VertexBlendRecognizer::IsSupportFace(const TopoDS_Face& face) {
    //find neighbour face and check the support face is current face
    TopTools_IndexedMapOfShape neighbours = m_aag->GetNeibourFaces(face);
    const int faceId = m_aag->GetFaceId(face);
    for (TopTools_IndexedMapOfShape::Iterator it(neighbours); it.More(); it.Next())
    {
        const TopoDS_Face face = TopoDS::Face(it.Value());
        std::shared_ptr<BlendProperty> blendInfo = m_aag->GetBlendProperty(face);
        if (blendInfo)
        {
            if (blendInfo->GetSupportFaceIndices().Contains(faceId))
            {
                return true;
            }
        }
    }

    return false;
}

std::vector<std::shared_ptr<BlendProperty> > VertexBlendRecognizer::GetNeibourProperties(const TopoDS_Face& face)
{
    std::vector<std::shared_ptr<BlendProperty> > results;
    // find neighbour face and check the support face is current face
    TopTools_IndexedMapOfShape neighbours = m_aag->GetNeibourFaces(face);
    const int faceId = m_aag->GetFaceId(face);
    for (TopTools_IndexedMapOfShape::Iterator it(neighbours); it.More(); it.Next())
    {
        const TopoDS_Face face = TopoDS::Face(it.Value());
        std::shared_ptr<BlendProperty> blendInfo = m_aag->GetBlendProperty(face);
        if (blendInfo)
        {
            results.push_back(std::move(blendInfo));
        }
    }

    return results;
}

static int CountEbfs(const std::vector<std::shared_ptr<BlendProperty> >& blendInfos) {
    int result = 0;
    for (const auto& info : blendInfos)
    {
        switch (info->GetBlendType())
        {
            case BlendFType::BlendType_Cliff:
            case BlendFType::BlendType_Ordinary:
            {
                ++result;
            }

            case BlendFType::BlendType_Uncertain:
            case BlendFType::BlendType_Vertex:
            {
                break;
            }

            default:
            {
                assert(false);
                break;
            }
        }
    }

    return result;
}

/*
* @brief judge if there is vertex blend neibour
* @param infos, blendProperties
* @return true if there is vertex blend; false otherwise
*/
static bool HasVertexBlendNeibour(const std::vector<std::shared_ptr<BlendProperty> >& infos)
{
    for (const auto& info : infos)
    {
        if (info->GetBlendType() == BlendFType::BlendType_Vertex)
        {
            return true;
        }
    }

    return false;
}

bool VertexBlendRecognizer::HasTerminateEdgeAsCommon(
    const TopoDS_Face& face,
                              const std::vector<std::shared_ptr<BlendProperty>>& neibourProperties)
{
    for (const auto& info : neibourProperties)
    {
        const TopoDS_Face& partner = m_aag->GetFace(info->GetFaceId());
        const TColStd_PackedMapOfInteger& terminateEdges = info->GetTerminatingEdgeIndices();
        for (TColStd_PackedMapOfInteger::Iterator it(terminateEdges); it.More(); it.Next())
        {
            const TopoDS_Edge& commonEdge = m_aag->GetEdge(it.Key());
            if (m_aag->GetNeibourFace(partner, commonEdge).IsEqual(face))
            {
                return true;
            }
        }
    }

    return false;
}

bool VertexBlendRecognizer::AllSmoothEdge(const TopoDS_Face& face,
                                          TColStd_PackedMapOfInteger& crossEdgeIndices)
{
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());

        const TopoDS_Face& partner = m_aag->GetNeibourFace(face, edge);

        if (partner.IsNull())
        {
            continue;
        }

        auto blendInfo = m_aag->GetBlendProperty(partner);

        if (blendInfo)
        {
            DihedralAngleType type = CheckDihedralAngle().AngleBetweenFaces(
                face, edge, partner, true, FeatureCommon::Angular());

            if (type == DihedralAngleType::Smooth)
            {
                const int edgeId = m_aag->GetEdgeId(edge);
                crossEdgeIndices.Add(edgeId);
            }
            else
            {
                return false;
            }
        }
    }

    return true;
}

/*
* @brief judge if face is Cylindrical Blend
* @note it will judge if face is cylindrical by assertting
* @param face
* @param[out] outInfo, extract blend info into outInfo
* @param return true if face is Cylindrical Blend
*/
bool VertexBlendRecognizer::IsBlend(const TopoDS_Face& face,
                                  std::shared_ptr<BlendProperty>& outInfo)
{   

    /* ------------------------------------------------- */
    /* Heuristic 2: the face is not a blend support face */
    /* ------------------------------------------------- */
    if (IsSupportFace(face))
    {
        return false;
    }

    std::vector<std::shared_ptr<BlendProperty> > neibourProperties = GetNeibourProperties(face);

    /* -------------------------------------------------------- */
    /* Heuristic 3: all VBFs have not less than 3 adjacent EBFs */
    /* -------------------------------------------------------- */
    const int ebfCount = CountEbfs(neibourProperties);
    if (ebfCount < 3)
    {
        return false;
    }

    /* -------------------------------------------------------- */
    /* Heuristic： two vertex blend cannot be neibour */
    /* -------------------------------------------------------- */
    if (HasVertexBlendNeibour(neibourProperties))
    {
        return false;
    }

    /* -------------------------------------------------------- */
    /* Heuristic： cannot has terminate edge as common edge     */
    /* -------------------------------------------------------- */
    if (HasTerminateEdgeAsCommon(face, neibourProperties))
    {
        return false;
    }

    /* -------------------------------------------------------- */
    /*Heuristic: all common edge connect to blend should be smooth edge, turn to cross edge */
    /* -------------------------------------------------------- */
    TColStd_PackedMapOfInteger crossEdgeIndices;

    if (!AllSmoothEdge(face, crossEdgeIndices))
    {
        return false;
    }

    SmoothEdgesFinder smoothEdgeFinder(m_aag);
    TColStd_PackedMapOfInteger smoothEdgeIndices = smoothEdgeFinder.PerformForFace(face);

    const int faceId = m_aag->GetFaceId(face);
    
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    
    if (FeatureCommon::IsSphericalSurface(surface))
    {
        const double radius = GetSphereRadius(surface);
        //ordinary
        outInfo = std::make_shared<BlendPropertyVertexConstRadius>(
            faceId, radius, smoothEdgeIndices, crossEdgeIndices);
    }
    else
    {
        //#TODO calc radii
        double maxRadius = 0, minRadius = 0, avgRadius = 0;
        outInfo = std::make_shared<BlendPropertyVertexVariousRadius>(
            faceId, maxRadius, minRadius, avgRadius, smoothEdgeIndices, crossEdgeIndices);
    }


    
    return nullptr != outInfo;
}
