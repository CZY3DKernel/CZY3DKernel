#include "./PlanarChamferRecognizer.h"

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

#include <unordered_set>

#include <math.h>
#include <assert.h>

#include "./FeatureCommon.h"
#include "./AttributeAdjacentGraph.h"
#include "./ChamferInfo.h"
#include "./SmoothEdgesFinder.h"
#include "./CompEdge.h"
#include "./ChamferRecognizerUtils.h"
#include "./CheckDihedralAngle.h"

// 获取面上某点的法向量
static gp_Dir GetFaceNormalAtPoint(const TopoDS_Face& face, const gp_Pnt& point)
{
    BRepAdaptor_Surface surface(face);

    // 将3D点投影到参数空间
    ShapeAnalysis_Surface surfaceAnalysis(surface.Surface().Surface());
    gp_Pnt2d uv = surfaceAnalysis.ValueOfUV(point, FeatureCommon::Confusion());
    
    // 计算法向量
    gp_Pnt p;
    gp_Vec d1u, d1v;
    surface.D1(uv.X(), uv.Y(), p, d1u, d1v);

    gp_Vec normal = d1u.Crossed(d1v);

    if (normal.Magnitude() < FeatureCommon::Confusion())
    {
        return gp_Dir(0, 0, 1);  // 返回默认法向量
    }

    if (face.Orientation() == TopAbs_FORWARD)
    {
        return gp_Dir(normal);
    }
    else
    {
        return gp_Dir(-normal);
    }
}


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

static std::shared_ptr<AttributeAdjacentGraph> s_aag;

class StraightCompEdgeExtrator
{
   public:

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
    static bool IsLinearEdge(const TopoDS_Edge& edge,
                             double angleTolerance = FeatureCommon::Angular())
    {
        gp_Ax1 axis;
        return FeatureCommon::IsStraight(edge, axis);
    }

    /**
     * 获取线框中的所有直线边
     */
    static std::vector<TopoDS_Edge> GetLinearEdgesFromWire(
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

            if (IsLinearEdge(edge, angleTolerance))
            {
                linearEdges.push_back(edge);
            }
        }

        return linearEdges;
    }

    /**
     * 获取外环中的所有直线边
     */
    static std::vector<TopoDS_Edge> GetLinearEdgesFromOuterWire(
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

        return GetLinearEdgesFromWire(outerWire, angleTolerance);
    }

    /**
     * 获取连续的直线边组成的复合边
     */
    static std::vector<std::vector<TopoDS_Edge>> GetConnectedLinearEdgeChains(
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

        // 收集所有直线边
        std::vector<bool> isLinear(allEdges.size(), false);
        std::vector<TopoDS_Edge> linearEdges;

        for (size_t i = 0; i < allEdges.size(); i++)
        {
            const bool res = IsLinearEdge(allEdges[i], angleTolerance);
            //printf("edge id = %d, isLinear = %d\n", s_aag->GetEdgeId(allEdges[i]), res);
            if (res)
            {
                isLinear[i] = true;
                linearEdges.push_back(allEdges[i]);
            }
        }

        if (linearEdges.empty())
        {
            return chains;
        }

        auto makeDir = [](const TopoDS_Edge& e) -> gp_Dir
        {
            TopoDS_Vertex v0, v1;
            TopExp::Vertices(e, v0, v1);

            gp_Pnt p0 = BRep_Tool::Pnt(v0);
            gp_Pnt p1 = BRep_Tool::Pnt(v1);

            return gp_Dir(gp_Vec(p0, p1));
        };

/*
        printf("linear ids = { ");
        for (int i=0;i<linearEdges.size();++i)
        {
            const TopoDS_Edge& e = linearEdges[i];
            printf("(%d -> %d), ", i, s_aag->GetEdgeId(e));
        }
        puts("}");
*/


        // 构建邻接关系
        for (size_t i = 0; i < linearEdges.size(); i++)
        {
            std::vector<TopoDS_Edge> currentChain;
            currentChain.push_back(linearEdges[i]);
            gp_Dir dir = makeDir(linearEdges[i]);
            int j = i + 1;
            for (; j < linearEdges.size(); j++)
            {
                bool found = false;
                if (FeatureCommon::HaveCommonVertex(currentChain.back(), linearEdges[j]))
                {
                    const gp_Dir currentDir = makeDir(linearEdges[j]);
                    //parallel
                    if (dir.IsParallel(currentDir, FeatureCommon::Angular()))
                    {
                        currentChain.push_back(linearEdges[j]);
                        found = true;
                    }
                }

                if (!found)
                {
                    break;
                }
            }

            i = j - 1;

            if (!currentChain.empty())
            {
                chains.push_back(currentChain);
            }
        }

        if (chains.size()>1)
        {
            const TopoDS_Edge firstEdge = chains.front().front();
            const TopoDS_Edge lastEdge = chains.back().back();
            if (FeatureCommon::HaveCommonVertex(firstEdge, lastEdge))
            {
                const gp_Dir firstDir = makeDir(firstEdge);
                const gp_Dir lastDir = makeDir(lastEdge);

                if (firstDir.IsParallel(lastDir, FeatureCommon::Angular()))
                {
                    chains.back().insert(chains.back().end(), chains.front().begin(),
                                         chains.front().end());
                    std::swap(chains.front(), chains.back());
                    chains.pop_back();
                    //assert(false);
                }
            }
            
        }

        return chains;
    }

    /**
     * 获取外环中连续的直线边链
     */
    static std::vector<CompEdge> GetLinearEdgeChainsFromOuterWire(
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
            GetConnectedLinearEdgeChains(outerWire, distanceTolerance, angleTolerance);

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

void PlanarChamferRecognizer::CollectCoaxialCompEdges(const std::vector<CompEdge>& compEdges,
                                    std::vector<gp_Ax1>& axiss,
                                    std::vector<std::vector<CompEdge>>& edgeGroups)
{
    for (const CompEdge& compEdge : compEdges)
    {
/*
        printf("edge ids = {");
        for (const TopoDS_Edge& e : compEdge.GetEdges())
        {
            printf("%d, ", m_aag->GetEdgeId(e));
        }
        puts("}");
*/

        const TopoDS_Vertex& start = compEdge.GetStart();
        const TopoDS_Vertex& end = compEdge.GetEnd();

        gp_Pnt startPos = BRep_Tool::Pnt(start);
        gp_Pnt endPos = BRep_Tool::Pnt(end);

        gp_Ax1 axis(startPos, gp_Dir(gp_Vec(startPos, endPos)));

        int match = -1;
        for (int i = 0; i < axiss.size(); ++i)
        {
            if (FeatureCommon::IsBiCoaxial(axis, axiss[i], FeatureCommon::Angular(), FeatureCommon::Confusion()))
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
            axiss.push_back(axis);
            // add new group
            edgeGroups.push_back({});
            edgeGroups.back().push_back(compEdge);
        }
    }
}

static double GetBeltLength(const std::vector<CompEdge>& compEdges, const gp_Ax1& axis)
{
    gp_Lin line(axis);

    double low = DBL_MAX;
    double high = -DBL_MAX;

    auto updateRange = [&](const gp_Pnt& pos) -> void
    {
        const double param = gp_Vec(line.Location(), pos).Dot(line.Direction());
        if (param > high)
        {
            high = param;
        }

        if (param < low)
        {
            low = param;
        }
    };

    for (const CompEdge& compEdge : compEdges)
    {
        TopoDS_Vertex start = compEdge.GetStart();
        TopoDS_Vertex end = compEdge.GetEnd();

        gp_Pnt startPos = BRep_Tool::Pnt(start);
        updateRange(startPos);

        gp_Pnt endPos = BRep_Tool::Pnt(end);
        updateRange(endPos);
    }

    return high - low;
}

static void CollectParallelEdgeGroups(const std::vector<gp_Ax1> axiss,
                                      std::vector<std::vector<int>>& outParallelIndices)
{
    std::vector<bool> visited(axiss.size(), false);

    for (int i = 0; i < axiss.size(); ++i)
    {
        if (visited[i])
        {
            continue;
        }

        visited[i] = true;

        std::vector<int> parallel;
        parallel.push_back(i);

        for (int j = i + 1; j < axiss.size(); ++j)
        {
            if (visited[j])
            {
                continue;
            }

            if (axiss[i].IsParallel(axiss[j], FeatureCommon::Angular()))
            {
                parallel.push_back(j);
                visited[j] = true;
            }
        }

        outParallelIndices.push_back(parallel);
    }
}

static bool GetMaxDistance(const std::vector<gp_Ax1>& axiss,
                           const std::vector<int>& parallelIndices, int& x, int& y, double& maxDis)
{
    if (parallelIndices.size() < 2)
    {
        return false;
    }

    maxDis = -1;

    for (int a = 0; a < parallelIndices.size(); ++a)
    {
        const int i = parallelIndices[a];
        gp_Lin line0(axiss[i]);

        for (int b = a + 1; b < parallelIndices.size(); ++b)
        {
            const int j = parallelIndices[b];
            const double dis = line0.Distance(axiss[j].Location());
            if (dis > maxDis)
            {
                maxDis = dis;
                x = i;
                y = j;
            }
        }
    }

    return true;
}

bool PlanarChamferRecognizer::CalculateBeltLengthWidth(const std::vector<CompEdge>& compEdges,
                            std::vector<CompEdge>& springEdges0,
                            std::vector<CompEdge>& springEdges1,
    double& beltLength, double& beltWidth)
{
    //collect coaxis edges;
    std::vector<gp_Ax1> axiss;
    std::vector<std::vector<CompEdge>> edgeGroups;

    CollectCoaxialCompEdges(compEdges, axiss, edgeGroups);

    std::vector<std::vector<int>> parallelIndices;
    CollectParallelEdgeGroups(axiss, parallelIndices);

    //find parallel edgeGroup and distance min
    double minDis = DBL_MAX;
    int targetI = -1;
    int targetJ = -1;
    
    for (const std::vector<int>& parallelGroup : parallelIndices)
    {
        double dis = 0;
        int x = 0;
        int y = 0;
        if (GetMaxDistance(axiss, parallelGroup, x, y, dis))
        {
            if (dis < minDis)
            {
                minDis = dis;
                targetI = x;
                targetJ = y;
            }
        }
    }

    if (targetI != -1)
    {
        springEdges0.swap(edgeGroups[targetI]);
        springEdges1.swap(edgeGroups[targetJ]);

        beltWidth = minDis;
        beltLength = std::max(GetBeltLength(springEdges0, axiss[targetI]),
                              GetBeltLength(springEdges1, axiss[targetJ]));
    }

    return targetI != -1;
}

bool PlanarChamferRecognizer::CalculateBeltLengthWidthWide(const std::vector<CompEdge>& compEdges,
                            std::vector<CompEdge>& springEdges0,
                            std::vector<CompEdge>& springEdges1,
    double& beltLength, double& beltWidth)
{
    //collect coaxis edges;
    std::vector<gp_Ax1> axiss;
    std::vector<std::vector<CompEdge>> edgeGroups;

    CollectCoaxialCompEdges(compEdges, axiss, edgeGroups);

    std::vector<std::vector<int>> parallelIndices;
    CollectParallelEdgeGroups(axiss, parallelIndices);

    //find parallel edgeGroup and distance min
    double maxDis = -1;
    int targetI = -1;
    int targetJ = -1;
    
    for (int i = 0;i < parallelIndices.size();++i)
    {
        int x = 0;
        double dis = DBL_MAX;
        int y = 0;
        if (GetMaxDistance(axiss, parallelIndices[i], x, y, dis))
        {
            if (dis > maxDis)
            {
                maxDis = dis;
                targetI = x;
                targetJ = y;
            }
        }
    }

    if (targetI != -1)
    {
        springEdges0.swap(edgeGroups[targetI]);
        springEdges1.swap(edgeGroups[targetJ]);

        beltWidth = maxDis;
        beltLength = std::max(GetBeltLength(springEdges0, axiss[targetI]),
                              GetBeltLength(springEdges1, axiss[targetJ]));
    }

    return targetI != -1;
}

// 构造函数
PlanarChamferRecognizer::PlanarChamferRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag,
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
        if (faceArea * 1.05 > supportArea)
        {
            return false;
        }
    }

    return true;
}

static double GetSectionLength(const TopoDS_Face& f, const gp_Pln& plane)
{
    // 4. 计算面与平面的交线
    BRepAlgoAPI_Section sectionMaker(f, plane, false);
    sectionMaker.Build();

    if (!sectionMaker.IsDone())
    {
        //std::cerr << "错误：截面计算失败" << std::endl;
        return -1.0;
    }

    TopoDS_Shape sectionShape = sectionMaker.Shape();

    // 5. 遍历截面结果，寻找圆弧边
    TopExp_Explorer edgeExplorer(sectionShape, TopAbs_EDGE);
    
    double maxLength = 0;

    const gp_Pnt& center = plane.Location();

    while (edgeExplorer.More())
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

        TopoDS_Vertex v0, v1;
        TopExp::Vertices(edge, v0, v1);

        if (BRep_Tool::Pnt(v0).Distance(center) < FeatureCommon::Confusion() ||
            BRep_Tool::Pnt(v1).Distance(center) < FeatureCommon::Confusion())
        {
            const double len = FeatureCommon::CalculateEdgeLength(edge);
            if (len > maxLength)
            {
                maxLength = len;
            }
        }
        
        edgeExplorer.Next();
    }

    return maxLength;
}

static bool CheckLengthCondition(const TopoDS_Face& face,
                                 const std::vector<CompEdge>& springEdges0,
                                 const std::vector<CompEdge>& springEdges1,
                                 std::shared_ptr<AttributeAdjacentGraph> aag)
{
    int edgeCount = 0;
    int wireCount = 0;
    for (TopExp_Explorer wireExp(face, TopAbs_WIRE); wireExp.More(); wireExp.Next())
    {
        ++wireCount;
        for (TopExp_Explorer edgeExp(TopoDS::Wire(wireExp.Current()), TopAbs_EDGE);
             edgeExp.More(); edgeExp.Next())
        {
            ++edgeCount;
        }
    }

    if (wireCount > 1)
    {
        return false;
    }

    if (edgeCount > 10)
    {
        return false;
    }

    auto findLarger = [&face, &aag](const std::vector<CompEdge>& compEdges) -> bool
    {
        for (const CompEdge& compEdge : compEdges)
        {
            for (const TopoDS_Edge& e : compEdge.GetEdges())
            {
                double low = 0, high = 0;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, low, high);

                const double t = (low + high) / 2;
                gp_Pnt point;
                gp_Vec T;
                c->D1(t, point, T);

                gp_Pln plane(point, gp_Dir(T));

                const TopoDS_Face partner = aag->GetNeibourFace(face, e);

                const double currentLength = GetSectionLength(face, plane);
                const double partnerLength = GetSectionLength(partner, plane);

                if (partnerLength > currentLength)
                {
                    return true;
                }
            }
        }

        return false;
    };
    if (findLarger(springEdges0) && findLarger(springEdges1))
    {
        return true;
    }

    return false;
}

static void OutputEdgeIds(const std::vector<CompEdge>& compEdges, std::shared_ptr<AttributeAdjacentGraph> aag) {
    puts("comp edge ids:");
    for (const CompEdge& compEdge : compEdges)
    {
        printf("{ ");
        for (const TopoDS_Edge& e : compEdge.GetEdges())
        {
            printf("%d, ", aag->GetEdgeId(e));
        }
        puts("}");
    }
}

bool PlanarChamferRecognizer::FindNarrowChamfer(const std::vector<CompEdge>& compEdges, ChamferFeature& feature)
{
    if (!CalculateBeltLengthWidth(compEdges, feature.springEdges0, feature.springEdges1,
                                  feature.beltLength, feature.beltWidth))
    {
        return false;
    }

    // 5. 检查条件4：带长带宽比大于5
    if (!ChamferRecognizerUtils::CheckBeltLengthWidthRatio(feature.beltLength, feature.beltWidth))
    {
        return false;
    }

    GetSupportFaces(feature.face, feature.springEdges0, feature.springEdges1, m_aag,
                    feature.supportFaces);

    // 6. 检查条件5：角度条件
    if (!CheckAngleConditions(m_face, feature.supportFaces))
    {
        return false;
    }

    // 检查大小条件，垂直于spring edge中点作平面，支撑面切割线长度需要大于当前面
    // 7. 检查面积条件
    if (!CheckAreaCondition(m_face, feature.supportFaces) &&
        !CheckLengthCondition(m_face, feature.springEdges0, feature.springEdges1, m_aag))
    {
        return false;
    }

    feature.physicalSize = feature.beltWidth;
    feature.type = ChamferType::EDGE_CHAMFER;

    return true;
}

bool PlanarChamferRecognizer::FindWideChamfer(const std::vector<CompEdge>& compEdges,
                                              ChamferFeature& feature)
{
    if (!CalculateBeltLengthWidthWide(compEdges, feature.springEdges0, feature.springEdges1,
                                  feature.beltLength, feature.beltWidth))
    {
        return false;
    }

    GetSupportFaces(feature.face, feature.springEdges0, feature.springEdges1, m_aag,
                    feature.supportFaces);

    // 6. 检查条件5：角度条件
    if (!CheckAngleConditions(m_face, feature.supportFaces))
    {
        return false;
    }

    // 检查大小条件，垂直于spring edge中点作平面，支撑面切割线长度需要大于当前面
    // 7. 检查面积条件
    if (!CheckAreaCondition(m_face, feature.supportFaces) &&
        !CheckLengthCondition(m_face, feature.springEdges0, feature.springEdges1, m_aag))
    {
        return false;
    }

    feature.physicalSize = feature.beltWidth;
    feature.type = ChamferType::EDGE_CHAMFER;

    return true;
}

static void OutputWireEdgeIds(const TopoDS_Wire& wire, std::shared_ptr<AttributeAdjacentGraph> aag) {
    // 获取所有边
    std::vector<TopoDS_Edge> allEdges;
    
    printf("edge id on wire = { ");
    for (TopExp_Explorer edgeExp(wire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
    {
        const TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
        printf("%d, ", aag->GetEdgeId(edge));
    }

    puts("}");
}

// 主识别函数
bool PlanarChamferRecognizer::IsChamfer(ChamferFeature& feature)
{
    // 2. 检查条件1：是否为单侧面
    if (!FeatureCommon::IsSingleSidedFace(m_face))
    {
        return false;
    }

    feature.face = m_face;

    s_aag = m_aag;
    std::vector<CompEdge> compEdges =  StraightCompEdgeExtrator::GetLinearEdgeChainsFromOuterWire(m_face);

    //OutputWireEdgeIds( StraightCompEdgeExtrator::GetOuterWire(m_face), m_aag);


    //OutputEdgeIds(compEdges, m_aag);

    // 4. 检查条件3：是否没有光滑边(spring edge不能是光滑边)
    ChamferRecognizerUtils::FilterNonSmoothCompEdges(m_face, m_aag, compEdges);

    if (FindNarrowChamfer(compEdges, feature))
    {
        return true;
    }
    else if (FindWideChamfer(compEdges, feature))
    {
        return true;
    }

    return false;
}

// ==================== 条件检查函数实现 ====================

// 条件3：检查是否没有光滑边
bool PlanarChamferRecognizer::HasSmoothEdges(const TopoDS_Face& face) const
{
    // 论文条件：倒角面没有光滑边
    // 所以这里返回true表示"没有光滑边"，符合倒角特征
    // 实际实现需要检查所有边
    
    SmoothEdgesFinder smoothEdgeFinder(m_aag);

    TColStd_PackedMapOfInteger edges =  smoothEdgeFinder.PerformForFace(face);

    return !edges.IsEmpty();
}

// 条件4：检查带长带宽比是否大于5
bool PlanarChamferRecognizer::CheckBeltLengthWidthRatio(const TopoDS_Face& face, 
                                                  const double beltLength, 
                                                  const double beltWidth) const {
    // 步骤3：检查比例是否大于5
    const double ratio = beltLength / beltWidth;
    return (ratio > 5.0);
}

// 条件5：检查角度条件
bool PlanarChamferRecognizer::CheckAngleConditions(
    const TopoDS_Face& face, const std::vector<TopoDS_Face>& adjacentFaces) const
{
    std::vector<gp_Dir> dirs(adjacentFaces.size());

    static const double angle10 = FeatureCommon::DegreesToRadians(10);
    static const double angle70 = FeatureCommon::DegreesToRadians(70);
    static const double angle60 = FeatureCommon::DegreesToRadians(60);
    static const double angle120 = FeatureCommon::DegreesToRadians(120);

    DihedralAngleType target = DihedralAngleType::Undefined;
    // 检查倒角面与每个相邻面的夹角
    for (int i = 0; i < adjacentFaces.size(); ++i)
    {
        const auto& adjFace = adjacentFaces[i];
        gp_Dir current;
        double angle = CalculateAngleBetweenFaces(face, adjFace, current, dirs[i]);

        const std::vector<TopoDS_Edge> commonEdges = FeatureCommon::FindCommonEdges(face, adjFace);
        const DihedralAngleType type = CheckDihedralAngle().AngleBetweenFaces(
            face, commonEdges.front(), adjFace, true, FeatureCommon::Confusion());

        if (DihedralAngleType::Undefined == target)
        {
            target = type;
        }
        else
        {
            if (type != target)
            {
                return false;
            }
        }

         //printf("angle between(%d, %d) = %.3f degrees\n", m_aag->GetFaceId(face),
           //   m_aag->GetFaceId(adjFace), FeatureCommon::RadiansToDegrees(angle));

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
double PlanarChamferRecognizer::CalculateEdgeLength(const TopoDS_Edge& edge) const
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

// 计算两个面之间的夹角
double PlanarChamferRecognizer::CalculateAngleBetweenFaces(const TopoDS_Face& face1,
                                                           const TopoDS_Face& face2,
                                                           gp_Dir& outNormal1,
                                                           gp_Dir& outNormal2) const
{
    return FeatureCommon::CalculateAngleBetweenFaces(face1, face2, outNormal1, outNormal2);
}
