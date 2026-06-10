#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <Precision.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <vector>
#include <map>
#include <set>

#include "./ChamferInfo.h"

#include "../Common/Extension_Export.hxx"

class AttributeAdjacentGraph;

class SurfaceType;

EXTENSION_EXPORT class ChamferRecognizer {
public:
    
    // 构造函数
    ChamferRecognizer(const TopoDS_Shape& shape, double sizeThreshold = 1e100);

    std::vector<std::vector<TopoDS_Face> > ExtractChamferChains();
    std::vector<std::vector<TopoDS_Face>> ExtractOrderedBlendChains();
    
    // 设置阈值
    void SetSizeThreshold(double threshold);
    
    // 主识别函数
    std::vector<ChamferFeature> RecognizeChamfers();

    /*
    * @brief judge if face with faceId is chamfer
    * @param faceId
    * @param[out] ChamferInfo
    * @return true if face is chamfer; false otherwise
    */
    bool IsChamfer(const int faceId, ChamferFeature& feature);

    /*
     * @brief judge if face is chamfer
     * @param face
     * @param[out] ChamferInfo
     * @return true if face is chamfer; false otherwise
     */
    bool IsChamfer(const TopoDS_Face& face, ChamferFeature& feature);
    
    // 获取所有识别到的倒角
    const std::vector<ChamferFeature>& GetChamfers() const;
    
    // 获取小倒角（尺寸小于阈值）
    std::vector<ChamferFeature> GetSmallChamfers() const;
    
    // 获取大倒角（尺寸大于等于阈值）
    std::vector<ChamferFeature> GetLargeChamfers() const;

    std::shared_ptr<AttributeAdjacentGraph> GetAag() const;
    
private:

    void FetchChamferChain(const int seed, std::vector<TopoDS_Face>& faces,
                        std::vector<int>& visited);

 enum class SurfaceType
 {
     Conic = 0,
     Planar = 1,
     Unknow = -1
 };

    // 识别条件检查函数
 bool IsSingleSidedFace(const TopoDS_Face& face) const;
 bool IsPlanarOrConicalFace(const TopoDS_Face& face, SurfaceType& type) const;
 bool HasSmoothEdges(const TopoDS_Face& face) const;
 bool CheckBeltLengthWidthRatio(const TopoDS_Face& face, 
                                   double& beltLength, 
                                   double& beltWidth) const;
 bool CheckAngleConditions(const TopoDS_Face& face) const;
 double CalculatePhysicalSize(const TopoDS_Face& face) const;
    
    // 辅助函数
 std::vector<TopoDS_Face> GetAdjacentFaces(const TopoDS_Face& face) const;
 bool IsSmoothEdge(const TopoDS_Edge& edge) const;
 double CalculateEdgeLength(const TopoDS_Edge& edge) const;
 double CalculateAngleBetweenFaces(const TopoDS_Face& face1, 
                                      const TopoDS_Face& face2) const;
 gp_Dir GetFaceNormalAtPoint(const TopoDS_Face& face, const gp_Pnt& point) const;
 gp_Vec GetEdgeTangentAtPoint(const TopoDS_Edge& edge, const gp_Pnt& point) const;
    
    // 带长带宽计算函数
 bool CalculateBeltLengthAndWidth(const TopoDS_Face& face, 
                                     double& beltLength, 
                                     double& beltWidth) const;
 TopoDS_Edge FindBeltLengthEdge(const TopoDS_Face& face) const;
 double CalculateBeltWidthAtMidpoint(const TopoDS_Edge& beltLengthEdge, 
                                        const TopoDS_Face& face) const;
    
    // 成员变量
    double m_sizeThreshold = 1e100;                 // 尺寸阈值
    std::vector<ChamferFeature> m_chamfers; // 识别到的倒角

    std::shared_ptr<AttributeAdjacentGraph> m_aag;

    const TopoDS_Shape m_shape;
};
