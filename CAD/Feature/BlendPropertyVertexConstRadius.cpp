#include "./BlendPropertyVertexConstRadius.h"

#include <assert.h>

BlendPropertyVertexConstRadius::BlendPropertyVertexConstRadius(
    int faceId, double radius, const TColStd_PackedMapOfInteger &smoothEdgeIndices,
    const TColStd_PackedMapOfInteger &crossEdgeIndices)
    : BlendProperty(faceId),
      m_radius(radius),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices)
{
}

void BlendPropertyVertexConstRadius::UniteCrossEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyVertexConstRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    //no member
}

bool BlendPropertyVertexConstRadius::IsConstRadius() const
{
    return true;
}

double BlendPropertyVertexConstRadius::GetMaxRadius() const
{
    return m_radius;
}

double BlendPropertyVertexConstRadius::GetMinRadius() const
{
    return m_radius;
}

double BlendPropertyVertexConstRadius::GetRadius() const
{
    return m_radius;
}

double BlendPropertyVertexConstRadius::GetAvgRadius() const
{
    return m_radius;
}

BlendFType BlendPropertyVertexConstRadius::GetBlendType() const
{
    return BlendFType::BlendType_Vertex;
}

double BlendPropertyVertexConstRadius::GetAngle() const
{
    //invalid interface
    assert(false);
    return 0;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetCliffEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetSmoothEdgeIndices() const
{
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetSpringEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetCrossEdgeIndices() const
{
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetTerminatingEdgeIndices()
    const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexConstRadius::GetSupportFaceIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}
