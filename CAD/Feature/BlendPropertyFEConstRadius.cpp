#include "./BlendPropertyFEConstRadius.h"


BlendPropertyFEConstRadius::BlendPropertyFEConstRadius(
    int faceId, double radius,  // 半径
    double angle,               // 过渡面对应横截面圆心角
    const TColStd_PackedMapOfInteger &cliffEdgeIndices,  // cliff edges
    const TColStd_PackedMapOfInteger &smoothEdgeIndices,
    const TColStd_PackedMapOfInteger &springEdgeIndices,
    const TColStd_PackedMapOfInteger &crossEdgeIndices,
    const TColStd_PackedMapOfInteger &terminatingEdgeIndices,
    const TColStd_PackedMapOfInteger &supportFaceIndices)
    : BlendProperty(faceId),
      m_radius(radius),
      m_angle(angle),
      m_cliffEdgeIndices(cliffEdgeIndices),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_springEdgeIndices(springEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices),
      m_terminatingEdgeIndices(terminatingEdgeIndices),
      m_supportFaceIndices(supportFaceIndices)
{
}

void BlendPropertyFEConstRadius::UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyFEConstRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_terminatingEdgeIndices.Subtract(indices);
}

bool BlendPropertyFEConstRadius::IsConstRadius() const {
    return true;
}

double BlendPropertyFEConstRadius::GetMaxRadius() const {
    return m_radius;
}

double BlendPropertyFEConstRadius::GetMinRadius() const {
    return m_radius;
}

double BlendPropertyFEConstRadius::GetRadius() const {
    return m_radius;
}

double BlendPropertyFEConstRadius::GetAvgRadius() const {
    return m_radius;
}

BlendFType BlendPropertyFEConstRadius::GetBlendType() const {
    return BlendFType::BlendType_Cliff;
}

double BlendPropertyFEConstRadius::GetAngle() const {
    return m_angle;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetCliffEdgeIndices() const {
    return m_cliffEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetSmoothEdgeIndices() const {
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetSpringEdgeIndices() const {
    return m_springEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetCrossEdgeIndices() const {
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetTerminatingEdgeIndices() const {
    return m_terminatingEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyFEConstRadius::GetSupportFaceIndices() const {
    return m_supportFaceIndices;
}
