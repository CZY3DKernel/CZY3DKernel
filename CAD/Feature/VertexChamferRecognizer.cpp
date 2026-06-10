
#include "./VertexChamferRecognizer.h"

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
            std::cerr << "warning: face has no outter wire" << std::endl;
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
            if (IsLinearEdge(allEdges[i], angleTolerance))
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
                        currentChain.push_back(allEdges[j]);
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

        //#TODO merge first and last connect compEdge

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

static void CollectCoaxialCompEdges(const std::vector<CompEdge>& compEdges,
                                    std::vector<gp_Ax1>& axiss,
                                    std::vector<std::vector<CompEdge>>& edgeGroups)
{
    for (const CompEdge& compEdge : compEdges)
    {
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

static bool CalculateBeltLengthWidth(const std::vector<CompEdge>& compEdges,
                            std::vector<CompEdge>& springEdges0,
                            std::vector<CompEdge>& springEdges1,
    double& beltLength, double& beltWidth)
{
    //collect coaxis edges;
    std::vector<gp_Ax1> axiss;
    std::vector<std::vector<CompEdge>> edgeGroups;

    CollectCoaxialCompEdges(compEdges, axiss, edgeGroups);

    //find parallel edgeGroup and distance min
    double minDis = DBL_MAX;
    int targetI = -1;
    int targetJ = -1;
    
    for (int i = 0;i < axiss.size();++i)
    {
        for (int j = i + 1; j < axiss.size(); ++j)
        {
            if (axiss[i].IsParallel(axiss[j], FeatureCommon::Angular()))
            {
                gp_Lin line0(axiss[i]);
                gp_Lin line1(axiss[j]);

                const double dis = line0.Distance(line1);
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
        beltLength = std::max(GetBeltLength(springEdges0, axiss[targetI]),
                              GetBeltLength(springEdges1, axiss[targetJ]));
    }

    return targetI != -1;
}

// 构造函数
VertexChamferRecognizer::VertexChamferRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag,
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

static bool CheckEdgeLength(const std::vector<CompEdge>& compEdges, double& beltWidth)
{
    beltWidth = 0;
    for (const CompEdge& e : compEdges)
    {
        if (e.Length() > beltWidth)
        {
            beltWidth = e.Length();
        }

    }

    if (beltWidth > 5)
    {
        return false;
    }

    return true;
}


static double GetBeltWidth(const std::vector<CompEdge>& compEdges)
{
    double beltWidth = 0;
    for (const CompEdge& e : compEdges)
    {
        if (e.Length() > beltWidth)
        {
            beltWidth = e.Length();
        }
    }

    return beltWidth;
}

static bool CheckEdgeNum(const std::vector<TopoDS_Face>& faces)
{
    for (const TopoDS_Face& face : faces)
    {
        TopoDS_Wire wire = StraightCompEdgeExtrator::GetOuterWire(face);
        TopExp_Explorer edgeExp(wire, TopAbs_EDGE);

        int edgeCount = 0;
        while (edgeExp.More())
        {
            ++edgeCount;
            edgeExp.Next();
        }

        if (edgeCount <= 4)
        {
            return false;
        }

    }
    return true;
}

static TopoDS_Edge GetOutEdge(const TopTools_IndexedMapOfShape& edges, const TopoDS_Face& face,
                                 std::shared_ptr<AttributeAdjacentGraph> aag)
{
    TopTools_IndexedMapOfShape faceEdge;
    TopExp::MapShapes(face, TopAbs_EDGE, faceEdge);

    TopoDS_Edge result;

    int outEdgeCount = 0;
    for (TopTools_IndexedMapOfShape::Iterator it(edges); it.More(); it.Next())
    {
        if (!faceEdge.Contains(it.Value()))
        {
            ++outEdgeCount;
            result = TopoDS::Edge(it.Value());
        }
    }

    if (1 != outEdgeCount)
    {
        return {};
    }

    return result;
}

static bool CheckOutEdge(const TopoDS_Face& face, std::shared_ptr<AttributeAdjacentGraph> aag)
{
    TopTools_IndexedMapOfShape vertices;
    TopoDS_Wire wire = StraightCompEdgeExtrator::GetOuterWire(face);
    TopExp::MapShapes(wire, TopAbs_VERTEX, vertices);

    if (vertices.Extent() != 3)
    {
        return false;
    }

    for (TopTools_IndexedMapOfShape::Iterator it1(vertices); it1.More(); it1.Next())
    {
        TopTools_ListOfShape edges = aag->GetEdgesByVertex(TopoDS::Vertex(it1.Value()));

        TopTools_IndexedMapOfShape uniqueEdges;
        for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next())
        {
            uniqueEdges.Add(it.Value());
        }

        if (uniqueEdges.Extent() != 3)
        {
            return false;
        }

        TopoDS_Edge outEdge = GetOutEdge(uniqueEdges, face, aag);

        if (outEdge.IsNull())
        {
            return false;
        }

        TopTools_ListOfShape fatherFaces = aag->GetFacesByEdge(outEdge);

        const DihedralAngleType type = CheckDihedralAngle().AngleBetweenFaces(
            TopoDS::Face(fatherFaces.First()), outEdge, TopoDS::Face(fatherFaces.Last()), true,
            FeatureCommon::Angular());

        if (DihedralAngleType::Convex != type)
        {
            return false;
        }
    }

    return true;
}

// 主识别函数
bool VertexChamferRecognizer::IsVertexSingleChamfer(ChamferFeature& feature)
{
    // 2. 检查条件1：是否为单侧面
    if (!FeatureCommon::IsSingleSidedFace(m_face))
    {
        return false;
    }

    feature.face = m_face;

    // 4. 检查条件3：是否没有光滑边
    if (HasSmoothEdges(m_face))
    {
        return false;  // 论文条件：倒角面没有光滑边
    }

    std::vector<CompEdge> compEdges =  StraightCompEdgeExtrator::GetLinearEdgeChainsFromOuterWire(m_face);

    //倒角面f0边数为3
    if (compEdges.size() != 3)
    {
        return false;
    }

    //每边的长度均小于5
    if (!CheckEdgeLength(compEdges, feature.beltWidth))
    {
        return false;
    }

     TopTools_IndexedMapOfShape neibours = m_aag->GetNeibourFaces(m_face);
      std::vector<TopoDS_Face> faces;
     for (TopTools_IndexedMapOfShape::Iterator it(neibours); it.More(); it.Next())
     {
         faces.push_back(TopoDS::Face(it.Value()));
     }

    // 6. 检查条件5：角度条件大于120度
    if (!CheckAngleConditions(m_face, faces))
    {
        return false;
    }

    //三个面的边数均大于4
    if (!CheckEdgeNum(faces))
    {
        return false;
    }

    //与三个顶点关联的边均只有一条，且都为凸边
    if (!CheckOutEdge(m_face, m_aag))
    {
        return false;
    }

    //7. 检查面积条件
    if (!CheckAreaCondition(m_face, faces))
    {
        return false;
    }

    feature.physicalSize = feature.beltWidth;
    feature.type = ChamferType::VERTEX_SINGLE_CHAMFER;

    return true;
}

// ==================== 条件检查函数实现 ====================

// 条件3：检查是否没有光滑边
bool VertexChamferRecognizer::HasSmoothEdges(const TopoDS_Face& face) const
{
    // 论文条件：倒角面没有光滑边
    // 所以这里返回true表示"没有光滑边"，符合倒角特征
    // 实际实现需要检查所有边
    
    SmoothEdgesFinder smoothEdgeFinder(m_aag);

    TColStd_PackedMapOfInteger edges =  smoothEdgeFinder.PerformForFace(face);

    return !edges.IsEmpty();
}

// 条件4：检查带长带宽比是否大于5
bool VertexChamferRecognizer::CheckBeltLengthWidthRatio(const TopoDS_Face& face, 
                                                  const double beltLength, 
                                                  const double beltWidth) const {
    // 步骤3：检查比例是否大于5
    const double ratio = beltLength / beltWidth;
    return (ratio > 5.0);
}

// 条件5：检查角度条件
bool VertexChamferRecognizer::CheckAngleConditions(
    const TopoDS_Face& face, const std::vector<TopoDS_Face>& adjacentFaces) const
{
    std::vector<gp_Dir> dirs(adjacentFaces.size());

    static const double angle60 = FeatureCommon::DegreesToRadians(60);

    // 检查倒角面与每个相邻面的夹角
    for (int i = 0; i < adjacentFaces.size(); ++i)
    {
        const auto& adjFace = adjacentFaces[i];
        gp_Dir current;
        double angle = CalculateAngleBetweenFaces(face, adjFace, current, dirs[i]);
        
        // 条件：倒角面与相邻面夹角在120°-180°之间
        
        if (angle > angle60) {
            return false;
        }
    }
    
    return true;
}

// 计算边长度
double VertexChamferRecognizer::CalculateEdgeLength(const TopoDS_Edge& edge) const
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

// 计算两个面之间的夹角
double VertexChamferRecognizer::CalculateAngleBetweenFaces(const TopoDS_Face& face1,
                                                           const TopoDS_Face& face2,
                                                           gp_Dir& outNormal1,
                                                           gp_Dir& outNormal2) const
{
    return FeatureCommon::CalculateAngleBetweenFaces(face1, face2, outNormal1, outNormal2);
}

static bool CheckAllAreChamfer(const std::vector<TopoDS_Face>& faces,
                               std::shared_ptr<AttributeAdjacentGraph> aag)
{
    for (const TopoDS_Face& f : faces)
    {
        if (!aag->HasChamferFeature(f))
        {
            return false;
        }
    }

    return true;
}

// 主识别函数
bool VertexChamferRecognizer::IsVertexJointChamfer(ChamferFeature& feature) const
{
    // 2. 检查条件1：是否为单侧面
    if (!FeatureCommon::IsSingleSidedFace(m_face))
    {
        return false;
    }

    feature.face = m_face;

    // 4. 检查条件3：是否没有光滑边
    if (HasSmoothEdges(m_face))
    {
        return false;  // 论文条件：倒角面没有光滑边
    }
    
    std::vector<CompEdge> compEdges =
        StraightCompEdgeExtrator::GetLinearEdgeChainsFromOuterWire(m_face);

    // 倒角面f0边数为3
    if (compEdges.size() != 3)
    {
        return false;
    }

    feature.beltWidth = GetBeltWidth(compEdges);

    TopTools_IndexedMapOfShape neibours = m_aag->GetNeibourFaces(m_face);

    //有3个邻面
    if (neibours.Extent() != 3)
    {
        return false;
    }


    std::vector<TopoDS_Face> faces;
    for (TopTools_IndexedMapOfShape::Iterator it(neibours); it.More(); it.Next())
    {
        faces.push_back(TopoDS::Face(it.Value()));
    }

    //邻面均为倒角
    if (!CheckAllAreChamfer(faces, m_aag))
    {
        return false;
    }

    // 6. 检查条件5：角度条件大于120度
    if (!CheckAngleConditions(m_face, faces))
    {
        return false;
    }

    // 7. 检查面积条件
    if (!CheckAreaCondition(m_face, faces))
    {
        return false;
    }

    feature.physicalSize = feature.beltWidth;
    feature.type = ChamferType::VERTEX_JIONT_CHAMFER;

    return true;
}
