#include "./ChamferRecognizer.h"

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

#include <unordered_set>
#include <queue>
#include <math.h>
#include <assert.h>

#include "./FeatureCommon.h"
#include "./AttributeAdjacentGraph.h"
#include "./PlanarChamferRecognizer.h"
#include "./ConicChamferRecognizer.h"
#include "./VertexChamferRecognizer.h"
#include "./ChamferNetwork.h"

// 构造函数
ChamferRecognizer::ChamferRecognizer(const TopoDS_Shape& shape, double sizeThreshold)
    :m_shape(shape), m_sizeThreshold(sizeThreshold) {
    m_aag = std::make_shared<AttributeAdjacentGraph>(shape);
}

// 设置阈值
void ChamferRecognizer::SetSizeThreshold(double threshold) {
    m_sizeThreshold = threshold;
}

bool ChamferRecognizer::IsChamfer(const int faceId, ChamferFeature& feature)
{
    const TopoDS_Face currentFace = m_aag->GetFace(faceId);
    return IsChamfer(currentFace, feature);
}

bool ChamferRecognizer::IsChamfer(const TopoDS_Face& currentFace, ChamferFeature& outFeature)
{
    // 3. 检查条件2：是否为平面或圆锥面
    SurfaceType surfaceType = SurfaceType::Unknow;
    if (!IsPlanarOrConicalFace(currentFace, surfaceType))
    {
        return false;
    }

    switch (surfaceType)
    {
        case SurfaceType::Conic:
        {
            // 8. 判别锥面倒角特征
            if (ConicChamferRecognizer(m_aag, currentFace).IsChamfer(outFeature))
            {
                outFeature.faceId = m_aag->GetFaceId(currentFace);
                outFeature.isSmall = (outFeature.physicalSize < m_sizeThreshold);
                
                //printf("chamfer id = %d\n", outFeature.faceId);

                return true;
            }
            break;
        }
        case SurfaceType::Planar:
        {
            // 8. 判别平面倒角特征
            if (PlanarChamferRecognizer(m_aag, currentFace).IsChamfer(outFeature))
            {
                outFeature.faceId = m_aag->GetFaceId(currentFace);
                outFeature.isSmall = (outFeature.physicalSize < m_sizeThreshold);

                //printf("chamfer id = %d\n", outFeature.faceId);

                return true;
            }
            else if (VertexChamferRecognizer(m_aag, currentFace).IsVertexSingleChamfer(outFeature))
            {
                outFeature.faceId = m_aag->GetFaceId(currentFace);
                outFeature.isSmall = true;

                //printf("chamfer id = %d\n", outFeature.faceId);

                return true;
            }
            break;
        }
        default:
        {
            assert(false);
        }
    }

    return false;
}

// 主识别函数
std::vector<ChamferFeature> 
ChamferRecognizer::RecognizeChamfers() {
    m_chamfers.clear();
    // 1. 遍历模型中的所有面
    
    const std::set<int> chamferIds = {25, 46};

    //Recognize Single Chamfer
    for (TopExp_Explorer faceExplorer(m_shape, TopAbs_FACE); faceExplorer.More();
         faceExplorer.Next())
    {
        TopoDS_Face currentFace = TopoDS::Face(faceExplorer.Current());

        const int faceId = m_aag->GetFaceId(currentFace);
        //printf("face id = %d\n", faceId);

        if (chamferIds.count(faceId) > 0)
        {
            int stop = 0;
        }

        std::shared_ptr<ChamferFeature> feature = std::make_shared<ChamferFeature>();
        if (IsChamfer(currentFace, *feature))
        {
            m_aag->AddChamferFeature(currentFace, feature);
            m_chamfers.push_back(*feature);
        }

    }

    //recognize vertex noint chamfer
    for (TopExp_Explorer faceExp(m_shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        TopoDS_Face currentFace = TopoDS::Face(faceExp.Current());

        if (m_aag->HasChamferFeature(currentFace))
        {
            continue;
        }

        const int faceId = m_aag->GetFaceId(currentFace);
        if (faceId == 5)
        {
            int stop = 0;
        }

        std::shared_ptr<ChamferFeature> feature = std::make_shared<ChamferFeature>();

        if (VertexChamferRecognizer(m_aag, currentFace).IsVertexJointChamfer(*feature))
        {
            feature->faceId = m_aag->GetFaceId(currentFace);
            feature->isSmall = true;
            m_aag->AddChamferFeature(currentFace, feature);
            m_chamfers.push_back(*feature);
        }
    }

    return m_chamfers;
}

std::shared_ptr<AttributeAdjacentGraph> ChamferRecognizer::GetAag() const
{
    return m_aag;
}

// 获取所有识别到的倒角
const std::vector<ChamferFeature>& 
ChamferRecognizer::GetChamfers() const {
    return m_chamfers;
}

// 获取小倒角
std::vector<ChamferFeature> 
ChamferRecognizer::GetSmallChamfers() const {
    std::vector<ChamferFeature> smallChamfers;
    for (const auto& chamfer : m_chamfers) {
        if (chamfer.isSmall) {
            smallChamfers.push_back(chamfer);
        }
    }
    return smallChamfers;
}

// 获取大倒角
std::vector<ChamferFeature> 
ChamferRecognizer::GetLargeChamfers() const {
    std::vector<ChamferFeature> largeChamfers;
    for (const auto& chamfer : m_chamfers) {
        if (!chamfer.isSmall) {
            largeChamfers.push_back(chamfer);
        }
    }
    return largeChamfers;
}

// ==================== 条件检查函数实现 ====================

// 条件1：检查是否为单侧面
bool ChamferRecognizer::IsSingleSidedFace(const TopoDS_Face& face) const
{
    // 在实体模型中，所有面都是单侧的
    // 双侧面通常出现在曲面模型中
    // 简化实现：检查面的朝向是否一致
    BRepAdaptor_Surface surface(face);
    
    // 获取面的边界框
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    
    if (bbox.IsVoid()) {
        return false;
    }
    
    // 检查面是否有面积（排除退化面）
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    double area = props.Mass();
    
    return (area > FeatureCommon::Confusion());
}

// 条件2：检查是否为平面或圆锥面
bool ChamferRecognizer::IsPlanarOrConicalFace(const TopoDS_Face& face, SurfaceType& type) const
{
    if (FeatureCommon::IsPlane(face))
    {
        type = SurfaceType::Planar;
        return true;
    }
    else if (FeatureCommon::IsConicFace(face))
    {
        type = SurfaceType::Conic;
        return true;
    }
    else
    {
        type = SurfaceType::Unknow;
        return false;
    }

    return false;
}

// 条件3：检查是否没有光滑边
bool ChamferRecognizer::HasSmoothEdges(const TopoDS_Face& face) const
{
    // 论文条件：倒角面没有光滑边
    // 所以这里返回true表示"没有光滑边"，符合倒角特征
    // 实际实现需要检查所有边
    
    TopExp_Explorer edgeExplorer(face, TopAbs_EDGE);
    
    for (; edgeExplorer.More(); edgeExplorer.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
        
        if (IsSmoothEdge(edge)) {
            return true; // 发现光滑边，不是倒角
        }
    }
    
    return false; // 没有光滑边，可能是倒角
}

// 检查边是否为光滑边
bool ChamferRecognizer::IsSmoothEdge(const TopoDS_Edge& edge) const
{
    // 获取共享该边的两个面
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(edge, TopAbs_FACE, faceMap);
    
    if (faceMap.Extent() < 2) {
        return false; // 边界边，不是光滑边
    }
    
    // 获取两个相邻面
    TopoDS_Face face1 = TopoDS::Face(faceMap(1));
    TopoDS_Face face2 = TopoDS::Face(faceMap(2));
    
    // 在边的中点处计算法向量
    double first=0, last=0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    
    if (curve.IsNull()) {
        return false;
    }
    
    double midParam = (first + last) / 2.0;
    gp_Pnt midPoint = curve->Value(midParam);
    
    // 计算两个面在该点的法向量
    gp_Dir normal1 = GetFaceNormalAtPoint(face1, midPoint);
    gp_Dir normal2 = GetFaceNormalAtPoint(face2, midPoint);
    
    // 检查法向量是否相同（光滑边要求法向量一致）
    double dotProduct = normal1.Dot(normal2);
    
    // 如果点积接近1，表示法向量方向相同，是光滑边
    return (fabs(dotProduct - 1.0) < FeatureCommon::Angular());
}

// 条件4：检查带长带宽比是否大于5
bool ChamferRecognizer::CheckBeltLengthWidthRatio(const TopoDS_Face& face, 
                                                  double& beltLength, 
                                                  double& beltWidth) const {
    return CalculateBeltLengthAndWidth(face, beltLength, beltWidth);
}

// 计算带长和带宽
bool ChamferRecognizer::CalculateBeltLengthAndWidth(const TopoDS_Face& face, 
                                                    double& beltLength, 
                                                    double& beltWidth) const {
    // 步骤1：找到带长边（最长的边）
    TopoDS_Edge beltLengthEdge = FindBeltLengthEdge(face);
    
    if (beltLengthEdge.IsNull()) {
        return false;
    }
    
    // 计算带长
    beltLength = CalculateEdgeLength(beltLengthEdge);
    
    if (beltLength < FeatureCommon::Confusion()) {
        return false;
    }
    
    // 步骤2：计算带宽（在带长边中点处）
    beltWidth = CalculateBeltWidthAtMidpoint(beltLengthEdge, face);
    
    if (beltWidth < FeatureCommon::Confusion()) {
        return false;
    }
    
    // 步骤3：检查比例是否大于5
    double ratio = beltLength / beltWidth;
    return (ratio > 5.0);
}

// 找到带长边（最长的边）
TopoDS_Edge ChamferRecognizer::FindBeltLengthEdge(const TopoDS_Face& face) const
{
    TopoDS_Edge longestEdge;
    double maxLength = 0.0;
    
    TopExp_Explorer edgeExplorer(face, TopAbs_EDGE);
    
    for (; edgeExplorer.More(); edgeExplorer.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
        double length = CalculateEdgeLength(edge);
        
        if (length > maxLength) {
            maxLength = length;
            longestEdge = edge;
        }
    }
    
    return longestEdge;
}
//
//TopoDS_Edge ChamferRecognizer::FindBeltLengthEdge(const TopoDS_Face& face) const
//{
//    TopoDS_Edge longestEdge;
//    double maxLength = 0.0;
//
//    // 1. 首先，尝试遍历所有“线”（Wire），计算连续边界的总长
//    TopExp_Explorer wireExplorer(face, TopAbs_WIRE);
//    for (; wireExplorer.More(); wireExplorer.Next())
//    {
//        TopoDS_Wire wire = TopoDS::Wire(wireExplorer.Current());
//        double wireLength = 0.0;
//        TopoDS_Edge currentEdge;
//
//        // 计算这个Wire中所有边的总长
//        TopExp_Explorer edgeInWireExplorer(wire, TopAbs_EDGE);
//        for (; edgeInWireExplorer.More(); edgeInWireExplorer.Next())
//        {
//            TopoDS_Edge edgeSegment = TopoDS::Edge(edgeInWireExplorer.Current());
//            wireLength += CalculateEdgeLength(edgeSegment);
//        }
//
//        // 如果这个Wire的总长超过当前记录的最大长度
//        if (wireLength > maxLength)
//        {
//            maxLength = wireLength;
//            // 注意：这里需要一种方式来表示“整个Wire”作为“带长”。
//            // 一种做法是：记录这个Wire，或者记录Wire上的第一条边并标记。
//            // 简化处理：可以记录这个Wire中的第一条边，并在后续计算中明确其代表一个复合边界。
//            BRepTools::SubShape(face, wire, currentEdge);  // 此函数仅为示意，实际需获取Wire的首边
//            if (!currentEdge.IsNull())
//            {
//                longestEdge = currentEdge;
//            }
//        }
//    }
//
//    // 2. 如果没有任何Wire（或作为备选逻辑），回退到原有的遍历单条边的逻辑
//    if (longestEdge.IsNull())
//    {
//        TopExp_Explorer edgeExplorer(face, TopAbs_EDGE);
//        for (; edgeExplorer.More(); edgeExplorer.Next())
//        {
//            TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
//            double length = CalculateEdgeLength(edge);
//            if (length > maxLength)
//            {
//                maxLength = length;
//                longestEdge = edge;
//            }
//        }
//    }
//
//    return longestEdge;
//}

// 在带长边中点处计算带宽
double ChamferRecognizer::CalculateBeltWidthAtMidpoint(const TopoDS_Edge& beltLengthEdge,
                                                       const TopoDS_Face& face) const
{
    // 步骤1：获取带长边的中点P1
    double first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(beltLengthEdge, first, last);

    if (curve.IsNull())
    {
        std::cerr << "错误: 无法获取带长边的几何曲线" << std::endl;
        return 0.0;
    }

    double midParam = (first + last) / 2.0;
    gp_Pnt midpoint = curve->Value(midParam);

    // 步骤2：计算中点处的切线单位向量v1
    gp_Vec tangent;
    curve->D1(midParam, midpoint, tangent);

    if (tangent.Magnitude() < FeatureCommon::Confusion())
    {
        std::cerr << "警告: 带长边切线长度为0" << std::endl;
        return 0.0;
    }

    gp_Dir tangentDir(tangent);

    // 步骤3：创建足够大的圆盘平面C
    // 计算模型的边界框以确定圆盘大小
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);

    if (bbox.IsVoid())
    {
        //std::cerr << "错误: 面的边界框为空" << std::endl;
        return 0.0;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    // 计算模型的最大尺寸
    double modelSize = sqrt(pow(xMax - xMin, 2) + pow(yMax - yMin, 2) + pow(zMax - zMin, 2));

    // 圆盘大小设置为模型尺寸的2倍，确保足够大
    double diskRadius = modelSize * 2.0;
    if (diskRadius < 10.0)
    {
        diskRadius = 100.0;  // 最小100单位
    }

    // 创建圆盘平面C（以中点P1为圆心，切线方向v1为法向量）
    gp_Ax3 diskAxis(midpoint, tangentDir);
    Handle(Geom_Plane) diskGeomPlane = new Geom_Plane(diskAxis);

    // 创建圆盘面（圆形的面，而不是无限平面）
    // 先创建一个大的方形面作为圆盘
    gp_Pln diskPln(diskAxis);

    // 在法平面上定义局部坐标系
    gp_Dir xDir, yDir;
    if (abs(tangentDir.X()) < 0.5)
    {
        xDir = gp_Dir(1.0, 0.0, 0.0);
    }
    else if (abs(tangentDir.Y()) < 0.5)
    {
        xDir = gp_Dir(0.0, 1.0, 0.0);
    }
    else
    {
        xDir = gp_Dir(0.0, 0.0, 1.0);
    }

    yDir = tangentDir.Crossed(xDir);
    xDir = yDir.Crossed(tangentDir);

    // 创建方形的四个角点
    gp_Pnt corners[4];
    corners[0] = midpoint.Translated(xDir.XYZ() * diskRadius + yDir.XYZ() * diskRadius);
    corners[1] = midpoint.Translated(-1 * xDir.XYZ() * diskRadius + yDir.XYZ() * diskRadius);
    corners[2] = midpoint.Translated(-1 * xDir.XYZ() * diskRadius - yDir.XYZ() * diskRadius);
    corners[3] = midpoint.Translated(xDir.XYZ() * diskRadius - yDir.XYZ() * diskRadius);

    // 创建圆盘的边界线框
    BRepBuilderAPI_MakeWire diskWire;
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        BRepBuilderAPI_MakeEdge edgeMaker(corners[i], corners[j]);
        if (edgeMaker.IsDone())
        {
            diskWire.Add(edgeMaker.Edge());
        }
    }

    if (!diskWire.IsDone())
    {
        std::cerr << "错误: 无法创建圆盘边界线框" << std::endl;
        return 0.0;
    }

    const TopoDS_Wire& diskWireShape = diskWire.Wire();

    // 创建圆盘面
    BRepBuilderAPI_MakeFace diskFaceMaker(diskGeomPlane, diskWireShape);
    if (!diskFaceMaker.IsDone())
    {
        //std::cerr << "错误: 无法创建圆盘面" << std::endl;
        return 0.0;
    }

    TopoDS_Face diskFace = diskFaceMaker.Face();

    // 步骤4：计算倒角面与圆盘面的交线
    BRepAlgoAPI_Section sectionMaker(face, diskFace);

    if (!sectionMaker.IsDone())
    {
        std::cerr << "错误: 无法计算面与圆盘面的交线" << std::endl;
        assert(false);

        // 尝试备用方法：使用IntCurvesFace_Intersector
        //return CalculateBeltWidthByIntersection(face, diskGeomPlane, diskRadius);
    }

    TopoDS_Shape intersectionShape = sectionMaker.Shape();

    // 步骤5：提取交线中的所有边，计算总长度
    double totalWidth = 0.0;
    int edgeCount = 0;

    TopExp_Explorer edgeExplorer(intersectionShape, TopAbs_EDGE);
    for (; edgeExplorer.More(); edgeExplorer.Next())
    {
        TopoDS_Edge intersectionEdge = TopoDS::Edge(edgeExplorer.Current());

        // 计算边的长度
        double edgeLength = CalculateEdgeLength(intersectionEdge);
        totalWidth += edgeLength;
        edgeCount++;

        // 调试信息
        if (edgeLength > FeatureCommon::Confusion())
        {
            std::cout << "  交线边 #" << edgeCount << " 长度: " << edgeLength << std::endl;
        }
    }

    if (edgeCount == 0)
    {
        std::cerr << "警告: 未找到倒角面与圆盘面的交线" << std::endl;
        assert(false);

        // 尝试另一种方法：通过参数空间求交
        //return CalculateBeltWidthByParameterSpace(face, beltLengthEdge, midpoint, tangentDir);
    }

    std::cout << "带宽计算: 找到 " << edgeCount << " 段交线，总长度: " << totalWidth << std::endl;

    return totalWidth;
}

// 条件5：检查角度条件
bool ChamferRecognizer::CheckAngleConditions(const TopoDS_Face& face) const
{
    // 获取相邻面
    std::vector<TopoDS_Face> adjacentFaces = GetAdjacentFaces(face);
    
    if (adjacentFaces.size() < 2) {
        return false; // 倒角至少连接两个面
    }
    
    // 检查倒角面与每个相邻面的夹角
    for (const auto& adjFace : adjacentFaces) {
        double angle = CalculateAngleBetweenFaces(face, adjFace);
        
        // 条件：倒角面与相邻面夹角在110°-180°之间
        if (angle < 110.0 * M_PI / 180.0 || angle > 180.0 * M_PI / 180.0) {
            return false;
        }
    }
    
    // 检查相邻面之间的夹角（如果有两个相邻面）
    if (adjacentFaces.size() >= 2) {
        double angleBetweenAdjacent = CalculateAngleBetweenFaces(adjacentFaces[0], adjacentFaces[1]);
        
        // 条件：相邻面之间夹角在60°-120°之间
        if (angleBetweenAdjacent < 60.0 * M_PI / 180.0 || 
            angleBetweenAdjacent > 120.0 * M_PI / 180.0) {
            return false;
        }
    }
    
    return true;
}

// 条件6：计算物理尺寸（带宽）
double ChamferRecognizer::CalculatePhysicalSize(const TopoDS_Face& face) const
{
    // 论文中使用带宽作为物理尺寸
    double beltLength, beltWidth;
    
    if (CalculateBeltLengthAndWidth(face, beltLength, beltWidth)) {
        return beltWidth;
    }
    
    // 备用方法：计算面的面积
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return sqrt(props.Mass()); // 返回面积的平方根作为尺寸估计
}

// ==================== 辅助函数实现 ====================

// 获取相邻面
std::vector<TopoDS_Face> ChamferRecognizer::GetAdjacentFaces(const TopoDS_Face& face) const
{
    TopTools_IndexedMapOfShape neibours = m_aag->GetNeibourFaces(face);
    
    std::vector<TopoDS_Face> adjacentFaces;
    // 转换为向量
    for (TopTools_IndexedMapOfShape::Iterator it(neibours);it.More(); it.Next())
    {
        adjacentFaces.push_back(TopoDS::Face(it.Value()));
    }
    
    return adjacentFaces;
}

// 计算边长度
double ChamferRecognizer::CalculateEdgeLength(const TopoDS_Edge& edge) const
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

// 计算两个面之间的夹角
double ChamferRecognizer::CalculateAngleBetweenFaces(const TopoDS_Face& face1, 
                                                     const TopoDS_Face& face2) const
{
    // 找到两个面的公共边
    std::vector<TopoDS_Edge> commonEdges;
    
    // 获取face1的所有边
    TopTools_IndexedMapOfShape edges1;
    TopExp::MapShapes(face1, TopAbs_EDGE, edges1);
    
    // 获取face2的所有边
    TopTools_IndexedMapOfShape edges2;
    TopExp::MapShapes(face2, TopAbs_EDGE, edges2);
    
    // 找到公共边
    for (int i = 1; i <= edges1.Extent(); i++) {
        TopoDS_Edge edge1 = TopoDS::Edge(edges1(i));
        
        for (int j = 1; j <= edges2.Extent(); j++) {
            TopoDS_Edge edge2 = TopoDS::Edge(edges2(j));
            
            if (edge1.IsSame(edge2)) {
                commonEdges.push_back(edge1);
                break;
            }
        }
    }
    
    if (commonEdges.empty()) {
        return 0.0; // 没有公共边
    }
    
    // 使用第一条公共边的中点计算法向量
    TopoDS_Edge commonEdge = commonEdges[0];
    
    double first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(commonEdge, first, last);
    
    if (curve.IsNull()) {
        return 0.0;
    }
    
    double midParam = (first + last) / 2.0;
    gp_Pnt midPoint = curve->Value(midParam);
    
    // 计算两个面在该点的法向量
    gp_Dir normal1 = GetFaceNormalAtPoint(face1, midPoint);
    gp_Dir normal2 = GetFaceNormalAtPoint(face2, midPoint);
    
    // 计算夹角
    double dotProduct = normal1.Dot(normal2);
    dotProduct = std::max(-1.0, std::min(1.0, dotProduct)); // 限制在[-1, 1]范围内
    
    return acos(dotProduct);
}

// 获取面上某点的法向量
gp_Dir ChamferRecognizer::GetFaceNormalAtPoint(const TopoDS_Face& face, 
                                               const gp_Pnt& point) const
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
    
    if (normal.Magnitude() < FeatureCommon::Confusion()) {
        return gp_Dir(0, 0, 1);  // 返回默认法向量
    }
    
    return gp_Dir(normal);
}

// 获取边上某点的切向量
gp_Vec ChamferRecognizer::GetEdgeTangentAtPoint(const TopoDS_Edge& edge, 
                                                const gp_Pnt& point) const
{
    double first=0, last=0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    
    if (curve.IsNull()) {
        return gp_Vec(0, 0, 0);
    }
    
    // 找到最近参数
    GeomAPI_ProjectPointOnCurve projector(point, curve);
    
    if (projector.NbPoints() == 0) {
        return gp_Vec(0, 0, 0);
    }
    
    double param = projector.LowerDistanceParameter();
    gp_Pnt p;
    gp_Vec tangent;
    
    curve->D1(param, p, tangent);
    
    return tangent;
}

void ChamferRecognizer::FetchChamferChain(const int seed, std::vector<TopoDS_Face>& faces,
                                      std::vector<int>& visited)
{
    auto isSupport = [](const std::vector<TopoDS_Face>& supportFaces,
                        const TopoDS_Face& target) -> bool
    {
        for (const TopoDS_Face& f : supportFaces)
        {
            if (f.IsEqual(target))
            {
                return true;
            }
        }

        return false;
    };

    const TopoDS_Face seedFace = m_aag->GetFace(seed);
    std::shared_ptr<ChamferFeature> seedFeature = m_aag->GetChamferFeature(seedFace);
    const double width = seedFeature->beltWidth;
    std::queue<int> faceQue;
    faceQue.push(seed);
    visited[seed] = true;

    while (!faceQue.empty())
    {
        const int faceId = faceQue.front();
        faceQue.pop();

        const TopoDS_Face& current = m_aag->GetFace(faceId);
        faces.push_back(current);

        //printf("current faceId = %d", faceId);

        std::shared_ptr<ChamferFeature> currentFeature = m_aag->GetChamferFeature(current);
        
        TopTools_IndexedMapOfShape neibours = m_aag->GetNeibourFaces(current);
        for (TopTools_IndexedMapOfShape::Iterator it(neibours); it.More(); it.Next())
        {
            const TopoDS_Face partner = TopoDS::Face(it.Value());
            const int partnerId = m_aag->GetFaceId(partner);
            //printf("partner id = %d\n", partnerId);
            if (partner.IsNull())
            {
                continue;
            }

            if (!m_aag->HasChamferFeature(partner))
            {
                continue;
            }

            std::shared_ptr<ChamferFeature> partnerFeature = m_aag->GetChamferFeature(partner);

            //same width
            if (partnerFeature->type == ChamferType::EDGE_CHAMFER && FeatureCommon::DoubleCompare(partnerFeature->beltWidth, width) != 0)
            {
                continue;
            }

            if (isSupport(partnerFeature->supportFaces, current) ||
                isSupport(currentFeature->supportFaces, partner))
            {
                continue;
            }
            
            if (!visited[partnerId])
            {
                visited[partnerId] = true;
                faceQue.push(partnerId);
            }
        }
    }
}

std::vector<std::vector<TopoDS_Face> > ChamferRecognizer::ExtractChamferChains()
{
    RecognizeChamfers();

    // connect faces by cross edges
    std::vector<int> visited(m_aag->GetFaces().Extent() + 1);

    const int faceCount = m_aag->GetFaces().Extent();

    std::vector<std::vector<TopoDS_Face> > result;

    for (int i = 1; i <= faceCount; ++i)
    {
        if (visited[i])
        {
            continue;
        }

        const TopoDS_Face& face = m_aag->GetFace(i);
        if (!m_aag->HasChamferFeature(face))
        {
            continue;
        }

        std::shared_ptr<ChamferFeature> feature = m_aag->GetChamferFeature(face);

        switch (feature->type)
        {
            case ChamferType::EDGE_CHAMFER:
            case ChamferType::VERTEX_SINGLE_CHAMFER:
            {
                std::vector<TopoDS_Face> faces;

                FetchChamferChain(i, faces, visited);

                result.push_back(faces);
                break;
            }

            case ChamferType::VERTEX_JIONT_CHAMFER:
            {
                break;
            }
            default:
                assert(false);
        }
    }

    return result;
}

std::vector<std::vector<TopoDS_Face>> ChamferRecognizer::ExtractOrderedBlendChains()
{
    std::vector<std::vector<TopoDS_Face>> faceChains = ExtractChamferChains();

    ChamferNetwork bn(m_aag, std::move(faceChains));

    return bn.GetOrderedChains();
}
