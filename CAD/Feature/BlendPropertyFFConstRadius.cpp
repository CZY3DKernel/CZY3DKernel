#include "./BlendPropertyFFConstRadius.h"

BlendPropertyFFConstRadius::BlendPropertyFFConstRadius(
    int faceId, double radius,  // 半径
    double angle,               // 过渡面对应横截面圆心角
    const TColStd_PackedMapOfInteger &smoothEdgeIndices,
    const TColStd_PackedMapOfInteger &springEdgeIndices,
    const TColStd_PackedMapOfInteger &crossEdgeIndices,
    const TColStd_PackedMapOfInteger &terminatingEdgeIndices,
    const TColStd_PackedMapOfInteger &supportFaceIndices)
    : BlendProperty(faceId),
      m_radius(radius),
      m_angle(angle),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_springEdgeIndices(springEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices),
      m_terminatingEdgeIndices(terminatingEdgeIndices),
      m_supportFaceIndices(supportFaceIndices)
{
}

void BlendPropertyFFConstRadius::UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyFFConstRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_terminatingEdgeIndices.Subtract(indices);
}

bool BlendPropertyFFConstRadius::IsConstRadius() const {
    return true;
}

double BlendPropertyFFConstRadius::GetMaxRadius() const {
    return m_radius;
}

double BlendPropertyFFConstRadius::GetMinRadius() const {
    return m_radius;
}

double BlendPropertyFFConstRadius::GetRadius() const
{
    return m_radius;
}

double BlendPropertyFFConstRadius::GetAvgRadius() const
{
    return m_radius;
}

BlendFType BlendPropertyFFConstRadius::GetBlendType() const
{
    return BlendFType::BlendType_Ordinary;
}

double BlendPropertyFFConstRadius::GetAngle() const
{
    return m_angle;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetCliffEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetSmoothEdgeIndices() const
{
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetSpringEdgeIndices() const
{
    return m_springEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetCrossEdgeIndices() const
{
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetTerminatingEdgeIndices() const
{
    return m_terminatingEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFFConstRadius::GetSupportFaceIndices() const
{
    return m_supportFaceIndices;
}
