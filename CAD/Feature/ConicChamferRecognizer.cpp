
#include "./ConicChamferRecognizer.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_IntSS.hxx>
#include <TopoDS.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopExp.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Wire.hxx>
#include <BRepLProp_SLProps.hxx>
#include <ShapeAnalysis.hxx>

#include <unordered_set>

#include <math.h>
#include <assert.h>

#include "./FeatureCommon.h"
#include "./AttributeAdjacentGraph.h"
#include "./ChamferInfo.h"
#include "./SmoothEdgesFinder.h"
#include "./CompEdge.h"
#include "./ChamferRecognizerUtils.h"

// 获取边上某点的切向量
static gp_Vec GetEdgeTangentAtPoint(const TopoDS_Edge& edge, const gp_Pnt& point)
{
    double first = 0, last = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

    if (curve.IsNull())
    {
        return gp_Vec(0, 0, 0);
    }

    // 找到最近参数
    GeomAPI_ProjectPointOnCurve projector(point, curve);

    if (projector.NbPoints() == 0)
    {
        return gp_Vec(0, 0, 0);
    }

    double param = projector.LowerDistanceParameter();
    gp_Pnt p;
    gp_Vec tangent;

    curve->D1(param, p, tangent);

    return tangent;
}

std::shared_ptr<AttributeAdjacentGraph> s_aag;

class CircleCompEdgeExtrator
{
   public:
    struct CircleInfo
    {
        bool IsSame(const CircleInfo& other) const
        {
            if (fabs(radius - other.radius) > std::max(confusion, other.confusion))
            {
                return false;
            }

            if (!FeatureCommon::IsBiCoaxial(axis, other.axis, FeatureCommon::Angular(),
                                            FeatureCommon::Confusion()))
            {
                return false;
            }

            return true;
        }

        double Distance(const CircleInfo& other) const
        {
            const double dx = fabs(radius - other.radius);
            const double dy = axis.Location().Distance(other.axis.Location());

            return std::sqrt(dx * dx + dy * dy);
        }

        gp_Ax1 axis;
        double radius = 0;
        double confusion = 0;
    };
    /**
     * 获取面的外环
     */
    static TopoDS_Wire GetOuterWire(const TopoDS_Face& face)
    {
        if (face.IsNull())
        {
            return TopoDS_Wire();
        }

        // 方法1：使用BRepTools::OuterWire（推荐）
        TopoDS_Wire outerWire = BRepTools::OuterWire(face);

        if (!outerWire.IsNull())
        {
            return outerWire;
        }

        // 方法2：如果没有外环，返回第一个线框
        TopExp_Explorer wireExp(face, TopAbs_WIRE);
        if (wireExp.More())
        {
            return TopoDS::Wire(wireExp.Current());
        }

        return TopoDS_Wire();
    }

    /**
     * 获取面的所有线框（包括内环和外环）
     */
    static std::vector<TopoDS_Wire> GetAllWires(const TopoDS_Face& face)
    {
        std::vector<TopoDS_Wire> wires;

        if (face.IsNull())
        {
            return wires;
        }

        TopExp_Explorer wireExp(face, TopAbs_WIRE);
        for (; wireExp.More(); wireExp.Next())
        {
            wires.push_back(TopoDS::Wire(wireExp.Current()));
        }

        return wires;
    }

    /**
     * 判断边是否为直线
     */
    static bool IsCircleEdge(const TopoDS_Edge& edge, CircleInfo& outCircle)
    {
        return FeatureCommon::IsCircle(edge, outCircle.axis, outCircle.radius, outCircle.confusion);
    }

    /**
     * 获取线框中的所有圆边
     */
    static std::vector<TopoDS_Edge> GetCircleEdgesFromWire(
        const TopoDS_Wire& wire, double angleTolerance = FeatureCommon::Angular())
    {
        std::vector<TopoDS_Edge> linearEdges;

        if (wire.IsNull())
        {
            return linearEdges;
        }

        TopExp_Explorer edgeExp(wire, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next())
        {
            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            CircleInfo circleInfo;
            if (IsCircleEdge(edge, circleInfo))
            {
                linearEdges.push_back(edge);
            }
        }

        return linearEdges;
    }

    /**
     * 获取外环中的所有直线边
     */
    static std::vector<TopoDS_Edge> GetCircleEdgesFromOuterWire(
        const TopoDS_Face& face, double angleTolerance = FeatureCommon::Angular())
    {
        std::vector<TopoDS_Edge> result;

        if (face.IsNull())
        {
            return result;
        }

        // 获取外环
        TopoDS_Wire outerWire = GetOuterWire(face);

        if (outerWire.IsNull())
        {
            //std::cerr << "警告: 面没有外环" << std::endl;
            return result;
        }

        return GetCircleEdgesFromWire(outerWire, angleTolerance);
    }

    /**
     * 获取连续的直线边组成的复合边
     */
    static std::vector<std::vector<TopoDS_Edge>> GetConnectedCircleEdgeChains(
        const TopoDS_Wire& wire, double distanceTolerance = FeatureCommon::Confusion(),
        double angleTolerance = FeatureCommon::Angular())
    {
        std::vector<std::vector<TopoDS_Edge>> chains;

        if (wire.IsNull())
        {
            return chains;
        }

        // 获取所有边
        std::vector<TopoDS_Edge> allEdges;
        TopExp_Explorer edgeExp(wire, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next())
        {
            allEdges.push_back(TopoDS::Edge(edgeExp.Current()));
        }

        if (allEdges.empty())
        {
            return chains;
        }

        // 收集所有圆边
        std::vector<CircleInfo> infos;
        std::vector<TopoDS_Edge> circleEdges;

        /*
        printf("all edge ids = { ");
        for (const TopoDS_Edge& e : allEdges)
        {
            printf("%d, ", s_aag->GetEdgeId(e));
        }
        puts("}");
        */

        for (size_t i = 0; i < allEdges.size(); i++)
        {
         
            CircleInfo circleInfo;
            if (IsCircleEdge(allEdges[i], circleInfo))
            {
                circleEdges.push_back(allEdges[i]);
                infos.push_back(circleInfo);
            }
        }



        if (circleEdges.empty())
        {
            return chains;
        }

        std::vector<CircleInfo> chainCircles;

        // 构建邻接关系
        for (size_t i = 0; i < circleEdges.size(); i++)
        {
            std::vector<TopoDS_Edge> currentChain;
            currentChain.push_back(circleEdges[i]);
            
            int j = i + 1;
            for (; j < circleEdges.size(); j++)
            {
                bool found = false;
                if (FeatureCommon::HaveCommonVertex(currentChain.back(), circleEdges[j]))
                {
                    //parallel
                    if (infos[j].IsSame(infos[i]))
                    {
                        currentChain.push_back(allEdges[j]);
                        found = true;
                    }
                }

                if (!found)
                {
                    break;
                }
            }

            if (!currentChain.empty())
            {
                chains.push_back(currentChain);
                chainCircles.push_back(infos[i]);
            }

            i = j - 1;
        }

        if (chains.size() > 1)
        {
            std::vector<TopoDS_Edge> & chainA = chains.front();
            std::vector<TopoDS_Edge> & chainB = chains.back();
            if (CompEdge(chainA).GetStart().IsSame(CompEdge(chainB).GetEnd()))
            {
                if (chainCircles.front().IsSame(chainCircles.back()))
                {
                    //merge first and last chain
                    chainB.insert(chainB.end(), chainA.begin(), chainA.end());
                    std::swap(chainA, chainB);
                    chains.pop_back();
                    chainCircles.pop_back();
                }
            }
        }

        return chains;
    }

    /**
     * 获取外环中连续的直线边链
     */
    static std::vector<CompEdge> GetCircleEdgeChainsFromOuterWire(
        const TopoDS_Face& face, double distanceTolerance = FeatureCommon::Confusion(),
        double angleTolerance = FeatureCommon::Angular())
    {
        if (face.IsNull())
        {
            return std::vector<CompEdge>();
        }

        TopoDS_Wire outerWire = GetOuterWire(face);

        if (outerWire.IsNull())
        {
            return {};
        }

        std::vector<std::vector<TopoDS_Edge>> edgeGroups =
            GetConnectedCircleEdgeChains(outerWire, distanceTolerance, angleTolerance);

        std::vector<CompEdge> compEdges;
        for (const auto& group : edgeGroups)
        {
            compEdges.push_back(CompEdge(group));
        }

        return compEdges;
    }

   private:

    /**
     * 获取边的起点
     */
    static TopoDS_Vertex GetStartVertex(const TopoDS_Edge& edge)
    {
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2);
        return v1;
    }

    /**
     * 获取边的终点
     */
    static TopoDS_Vertex GetEndVertex(const TopoDS_Edge& edge)
    {
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2);
        return v2;
    }

    /**
     * 判断两个顶点是否重合
     */
    static bool AreVerticesCoincident(const TopoDS_Vertex& v1, const TopoDS_Vertex& v2,
                                      double tolerance)
    {
        if (v1.IsNull() || v2.IsNull())
        {
            return false;
        }

        gp_Pnt p1 = BRep_Tool::Pnt(v1);
        gp_Pnt p2 = BRep_Tool::Pnt(v2);

        return p1.Distance(p2) <= tolerance;
    }

};

static void CollectSameCircleCompEdges(const std::vector<CompEdge>& compEdges,
                                    std::vector<CircleCompEdgeExtrator::CircleInfo>& circles,
                                    std::vector<std::vector<CompEdge>>& edgeGroups)
{
    for (const CompEdge& compEdge : compEdges)
    {
        CircleCompEdgeExtrator::CircleInfo circleInfo;
        CircleCompEdgeExtrator::IsCircleEdge(compEdge.GetEdges().front(), circleInfo);

        int match = -1;
        for (int i = 0; i < circles.size(); ++i)
        {
            if (circles[i].IsSame(circleInfo))
            {
                match = i;
                break;
            }
        }

        if (match != -1)
        {
            // found
            edgeGroups[match].push_back(compEdge);
        }
        else
        {
            circles.push_back(circleInfo);
            // add new group
            edgeGroups.push_back({});
            edgeGroups.back().push_back(compEdge);
        }
    }
}

static double GetBeltLength(const std::vector<CompEdge>& compEdges)
{
    //now use sum of length, if there is discontigous segment,then fix bug
    double sum = 0;
    for (const CompEdge& compEdge : compEdges)
    {
        sum += compEdge.Length();
    }

    return sum;
}

static bool CalculateBeltLengthWidth(const std::vector<CompEdge>& compEdges,
                            std::vector<CompEdge>& springEdges0,
                            std::vector<CompEdge>& springEdges1,
    double& beltLength, double& beltWidth)
{
    //collect coaxis edges;
    std::vector<CircleCompEdgeExtrator::CircleInfo> circles;
    std::vector<std::vector<CompEdge>> edgeGroups;

    CollectSameCircleCompEdges(compEdges, circles, edgeGroups);

    //find parallel edgeGroup and distance min
    double minDis = DBL_MAX;
    int targetI = -1;
    int targetJ = -1;
    
    for (int i = 0;i < circles.size();++i)
    {
        for (int j = i + 1; j < circles.size(); ++j)
        {
            if (FeatureCommon::DoubleCompare(circles[i].radius, circles[j].radius) != 0 &&
                FeatureCommon::IsBiCoaxial(circles[i].axis, circles[j].axis,
                                           FeatureCommon::Angular(), FeatureCommon::Confusion()))
            {
                const double dis = circles[i].Distance(circles[j]);
                if (dis < minDis)
                {
                    minDis = dis;
                    targetI = i;
                    targetJ = j;
                }
            }
        }
    }

    if (targetI != -1)
    {
        springEdges0.swap(edgeGroups[targetI]);
        springEdges1.swap(edgeGroups[targetJ]);

        beltWidth = minDis;
        beltLength = std::max(GetBeltLength(springEdges0),
                              GetBeltLength(springEdges1));
    }

    return targetI != -1;
}

// 构造函数
ConicChamferRecognizer::ConicChamferRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag,
                                                 const TopoDS_Face& face)
    : m_aag(aag), m_face(face)
{
}

static void GetSupportFaces(const TopoDS_Face& current, const std::vector<CompEdge>& edges0,
                            const std::vector<CompEdge>& edges1,std::shared_ptr<AttributeAdjacentGraph> aag,
                            std::vector<TopoDS_Face>& supportFaces)
{
    TopTools_IndexedMapOfShape faces;
    
    auto addFaces = [&](const std::vector<CompEdge>& edges) -> void
    {
        for (const CompEdge& compEdge : edges)
        {
            for (const TopoDS_Edge& edge : compEdge.GetEdges())
            {
                faces.Add(aag->GetNeibourFace(current, edge));
            }
        }
    };

    addFaces(edges0);
    addFaces(edges1);

    supportFaces.clear();
    for (TopTools_IndexedMapOfShape::Iterator it(faces); it.More(); it.Next())
    {
        supportFaces.push_back(TopoDS::Face(it.Value()));
    }
}

static bool CheckAreaCondition(const TopoDS_Face& face,
                               const std::vector<TopoDS_Face>& supportFaces)
{
    const double faceArea = FeatureCommon::CalculateFaceArea(face);
    for (const TopoDS_Face& supportFace : supportFaces)
    {
        const double supportArea = FeatureCommon::CalculateFaceArea(supportFace);
        if (faceArea * 2 > supportArea)
        {
            return false;
        }
    }

    return true;
}

// 主识别函数
bool ConicChamferRecognizer::IsChamfer(ChamferFeature& feature)
{
    s_aag = m_aag;
    // 2. 检查条件1：是否为单侧面
    if (!FeatureCommon::IsSingleSidedFace(m_face))
    {
        return false;
    }

    feature.face = m_face;

    std::vector<CompEdge> compEdges =  CircleCompEdgeExtrator::GetCircleEdgeChainsFromOuterWire(m_face);

    // 4. 检查条件3：是否没有光滑边(spring edge 不能是光滑边)
    ChamferRecognizerUtils::FilterNonSmoothCompEdges(m_face, m_aag, compEdges);

    if (!CalculateBeltLengthWidth(compEdges, feature.springEdges0, feature.springEdges1,
                                  feature.beltLength, feature.beltWidth))
    {
        return false;
    }

    // 5. 检查条件4：带长带宽比大于5
    /*if (!ChamferRecognizerUtils::CheckBeltLengthWidthRatio(feature.beltLength, feature.beltWidth))
    {
        conic not need this condition
        return false;
    }*/

    GetSupportFaces(feature.face, feature.springEdges0, feature.springEdges1, m_aag,
                    feature.supportFaces);

    // 6. 检查条件5：角度条件
    if (!CheckAngleConditions(m_face, feature.supportFaces))
    {
        return false;
    }


    /*if (!CheckAreaCondition(m_face, feature.supportFaces))
    {
        return false;
    }*/

    feature.physicalSize = feature.beltWidth;
    feature.type = ChamferType::EDGE_CHAMFER;

    return true;
}

// ==================== 条件检查函数实现 ====================

// 条件3：检查是否没有光滑边
bool ConicChamferRecognizer::HasSmoothEdges(const TopoDS_Face& face) const
{
    // 论文条件：倒角面没有光滑边
    // 所以这里返回true表示"没有光滑边"，符合倒角特征
    // 实际实现需要检查所有边
    
    SmoothEdgesFinder smoothEdgeFinder(m_aag);

    TColStd_PackedMapOfInteger edges =  smoothEdgeFinder.PerformForFace(face);

    //remove comp conic edge
    TColStd_PackedMapOfInteger compConicEdge;

    for (TColStd_PackedMapOfInteger::Iterator it(edges); it.More(); it.Next())
    {
        const TopoDS_Edge midEdge = m_aag->GetEdge(it.Key());
        const TopoDS_Face partner = m_aag->GetNeibourFace(face, midEdge);

        if (FeatureCommon::IsSameConic(partner, face))
        {
            compConicEdge.Add(it.Key());
        }
    }

    edges.Subtract(compConicEdge);

    return !edges.IsEmpty();
}

// 条件4：检查带长带宽比是否大于5
bool ConicChamferRecognizer::CheckBeltLengthWidthRatio(const TopoDS_Face& face, 
                                                  const double beltLength, 
                                                  const double beltWidth) const {
    // 步骤3：检查比例是否大于5
    const double ratio = beltLength / beltWidth;
    return (ratio > 5.0);
}

// 条件5：检查角度条件
bool ConicChamferRecognizer::CheckAngleConditions(
    const TopoDS_Face& face, const std::vector<TopoDS_Face>& adjacentFaces) const
{
    std::vector<gp_Dir> dirs(adjacentFaces.size());

    static const double angle10 = FeatureCommon::DegreesToRadians(10);
    static const double angle70 = FeatureCommon::DegreesToRadians(70);
    static const double angle60 = FeatureCommon::DegreesToRadians(60);
    static const double angle120 = FeatureCommon::DegreesToRadians(120);

    // 检查倒角面与每个相邻面的夹角
    for (int i = 0; i < adjacentFaces.size(); ++i)
    {
        const auto& adjFace = adjacentFaces[i];
        gp_Dir current;
        double angle = CalculateAngleBetweenFaces(face, adjFace, current, dirs[i]);
        
        /*printf("angle between(%d, %d) = %.3f degrees\n", m_aag->GetFaceId(face),
               m_aag->GetFaceId(adjFace), FeatureCommon::RadiansToDegrees(angle));*/

        // 条件：倒角面与相邻面夹角在110°-170°之间
        //对应的是法向夹角10-70
        if (angle < angle10 || angle > angle70) {
            return false;
        }
    }
    
    // 检查相邻面之间的夹角（如果有两个相邻面）
    if (adjacentFaces.size() >= 2) {

        // 计算夹角
        double dotProduct = dirs[0].Dot(dirs[1]);
        dotProduct = std::max(-1.0, std::min(1.0, dotProduct));  // 限制在[-1, 1]范围内

        const double angleBetweenAdjacent = acos(dotProduct);
        
        // 条件：相邻面之间夹角在60°-120°之间
        //对应法向夹角在60-120
        if (angleBetweenAdjacent < angle60 || 
            angleBetweenAdjacent > angle120) {
            return false;
        }
    }
    
    return true;
}

// 计算边长度
double ConicChamferRecognizer::CalculateEdgeLength(const TopoDS_Edge& edge) const
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

// 计算两个面之间的夹角
double ConicChamferRecognizer::CalculateAngleBetweenFaces(const TopoDS_Face& face1,
                                                           const TopoDS_Face& face2,
                                                           gp_Dir& outNormal1,
                                                           gp_Dir& outNormal2) const
{
    return FeatureCommon::CalculateAngleBetweenFaces(face1, face2, outNormal1, outNormal2);
}
