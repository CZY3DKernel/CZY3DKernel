#include "./BlendPropertyVertexVariousRadius.h"

#include <assert.h>

BlendPropertyVertexVariousRadius::BlendPropertyVertexVariousRadius(
    int faceId, double minRadius, double maxRadius, double avgRadius,
    const TColStd_PackedMapOfInteger& smoothEdgeIndices,
    const TColStd_PackedMapOfInteger &crossEdgeIndices)
    : BlendProperty(faceId),
      m_minRadius(minRadius),
      m_maxRadius(maxRadius),
      m_avgRadius(avgRadius),
      m_smoothEdgeIndices(smoothEdgeIndices),
      m_crossEdgeIndices(crossEdgeIndices)
{
}

void BlendPropertyVertexVariousRadius::UniteCrossEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    m_crossEdgeIndices.Unite(indices);
}

void BlendPropertyVertexVariousRadius::SubtractTerminatingEdgeIndices(
    const TColStd_PackedMapOfInteger &indices)
{
    //no member
}

bool BlendPropertyVertexVariousRadius::IsConstRadius() const
{
    return false;
}

double BlendPropertyVertexVariousRadius::GetMaxRadius() const
{
    return m_maxRadius;
}

double BlendPropertyVertexVariousRadius::GetMinRadius() const
{
    return m_minRadius;
}

double BlendPropertyVertexVariousRadius::GetRadius() const
{
    //invalid interface;
    assert(false);
    return 0;
}

double BlendPropertyVertexVariousRadius::GetAvgRadius() const
{
    return m_avgRadius;
}

BlendFType BlendPropertyVertexVariousRadius::GetBlendType() const
{
    return BlendFType::BlendType_Vertex;
}

double BlendPropertyVertexVariousRadius::GetAngle() const
{
    //invalid interface
    assert(false);
    return 0;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetCliffEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetSmoothEdgeIndices() const
{
    return m_smoothEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetSpringEdgeIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetCrossEdgeIndices() const
{
    return m_crossEdgeIndices;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetTerminatingEdgeIndices()
    const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}

const TColStd_PackedMapOfInteger &BlendPropertyVertexVariousRadius::GetSupportFaceIndices() const
{
    static TColStd_PackedMapOfInteger s_empty;
    return s_empty;
}
