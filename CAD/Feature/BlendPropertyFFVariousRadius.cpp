#include "./BlendPropertyFFVariousRadius.h"

#include <assert.h>

BlendPropertyFFVariousRadius::BlendPropertyFFVariousRadius(
    int faceId,
    double maxRadius,  // 最大半径
    double minRadius,  // 最小半径
    double avgRadius,  // 平均半径
    double angle,      // 过渡面对应横截面圆心角
    TColStd_PackedMapOfInteger smoothEdgeIndices,
    TColStd_PackedMapOfInteger springEdgeIndices, TColStd_PackedMapOfInteger crossEdgeIndices,
    TColStd_PackedMapOfInteger terminatingEdgeIndices,
    TColStd_PackedMapOfInteger supportFaceIndices)
    : BlendProperty(faceId),
      m_maxRadius(maxRadius),
      m_minRadius(minRadius),
      m_avgRadius(avgRadius),
      m_angle(angle),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_springEdgeIndices(springEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices),
      m_terminatingEdgeIndices(terminatingEdgeIndices),
      m_supportFaceIndices(supportFaceIndices)
{
}

void BlendPropertyFFVariousRadius::UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyFFVariousRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_terminatingEdgeIndices.Subtract(indices);
}

bool BlendPropertyFFVariousRadius::IsConstRadius() const
{
    return false;
}

double BlendPropertyFFVariousRadius::GetMaxRadius() const
{
    return m_maxRadius;
}

double BlendPropertyFFVariousRadius::GetMinRadius() const
{
    return m_minRadius;
}

double BlendPropertyFFVariousRadius::GetRadius() const
{
    //invalid interface;
    assert(false);
    return 0;
}

double BlendPropertyFFVariousRadius::GetAvgRadius() const {
    return m_avgRadius;
}

BlendFType BlendPropertyFFVariousRadius::GetBlendType() const
{
    return BlendFType::BlendType_Ordinary;
}

double BlendPropertyFFVariousRadius::GetAngle() const
{
    return m_angle;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetCliffEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetSmoothEdgeIndices() const
{
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetSpringEdgeIndices() const
{
    return m_springEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetCrossEdgeIndices() const
{
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetTerminatingEdgeIndices() const
{
    return m_terminatingEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFVariousRadius::GetSupportFaceIndices() const
{
    return m_supportFaceIndices;
}
