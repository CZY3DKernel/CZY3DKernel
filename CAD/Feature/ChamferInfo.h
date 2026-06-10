#include <TopoDS_Face.hxx>

#include <vector>

#include "./CompEdge.h"

#include "../Common/Extension_Export.hxx"

#pragma once
// 倒角类型枚举
EXTENSION_EXPORT enum ChamferType
{
    EDGE_CHAMFER = 0,
    VERTEX_SINGLE_CHAMFER = 1,
    VERTEX_JIONT_CHAMFER = 2,
    UNKNOWN = -1
};

// 倒角特征结构体
EXTENSION_EXPORT struct ChamferFeature
{
    int faceId = 0;
    TopoDS_Face face;                       // 倒角面
    ChamferType type;                          // 倒角类型
    double beltLength = 0;                     // 带长
    double beltWidth = 0;                      // 带宽
    double physicalSize = 0;                   // 物理尺寸（带宽）
    std::vector<CompEdge> springEdges0;
    std::vector<CompEdge> springEdges1;
    std::vector<TopoDS_Face> supportFaces;  // 相邻面
    bool isSmall = false;                      // 是否是小倒角
};

using CskIndexedDataMapOfShapeChamferFeature = NCollection_IndexedDataMap<TopoDS_Shape, std::shared_ptr<ChamferFeature>, TopTools_ShapeMapHasher>;
