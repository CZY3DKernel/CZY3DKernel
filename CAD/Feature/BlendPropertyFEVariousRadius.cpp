#include "./BlendPropertyFEVariousRadius.h"

#include <assert.h>

BlendPropertyFEVariousRadius::BlendPropertyFEVariousRadius(
    int faceId,
    double maxRadius,  // 最大半径
    double minRadius,  // 最小半径
    double avgRadius,  // 平均半径
    double angle,      // 过渡面对应横截面圆心角
    TColStd_PackedMapOfInteger cliffEdgeIndices, TColStd_PackedMapOfInteger smoothEdgeIndices,
    TColStd_PackedMapOfInteger springEdgeIndices, TColStd_PackedMapOfInteger crossEdgeIndices,
    TColStd_PackedMapOfInteger terminatingEdgeIndices,
    TColStd_PackedMapOfInteger supportFaceIndices)
    : BlendProperty(faceId),
      m_maxRadius(maxRadius),
      m_minRadius(minRadius),
      m_avgRadius(avgRadius),
      m_angle(angle),
      m_cliffEdgeIndices(cliffEdgeIndices),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_springEdgeIndices(springEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices),
      m_terminatingEdgeIndices(terminatingEdgeIndices),
      m_supportFaceIndices(supportFaceIndices)
{
}

void BlendPropertyFEVariousRadius::UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyFEVariousRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_terminatingEdgeIndices.Subtract(indices);
}

bool BlendPropertyFEVariousRadius::IsConstRadius() const
{
    return false;
}

double BlendPropertyFEVariousRadius::GetMaxRadius() const {
    return m_maxRadius;
}

double BlendPropertyFEVariousRadius::GetMinRadius() const {
    return m_minRadius;
}

double BlendPropertyFEVariousRadius::GetRadius() const {
    //invalid 接口
    assert(false);
    return 0;
}

double BlendPropertyFEVariousRadius::GetAvgRadius() const {
    return m_avgRadius;
}

BlendFType BlendPropertyFEVariousRadius::GetBlendType() const {
    return BlendFType::BlendType_Cliff;
}

double BlendPropertyFEVariousRadius::GetAngle() const {
    return m_angle;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetCliffEdgeIndices() const {
    return m_cliffEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetSmoothEdgeIndices() const {
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetSpringEdgeIndices() const {
    return m_springEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetCrossEdgeIndices() const {
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetTerminatingEdgeIndices() const
{
    return m_terminatingEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEVariousRadius::GetSupportFaceIndices() const
{
    return m_supportFaceIndices;
}
