#pragma once

#include "./BlendProperty.h"

#include <TColStd_PackedMapOfInteger.hxx>

EXTENSION_EXPORT class BlendPropertyFFConstRadius : public BlendProperty
{
   public:
        BlendPropertyFFConstRadius(int faceId, double radius, // 半径
                               double angle,   // 过渡面对应横截面圆心角
                               const TColStd_PackedMapOfInteger& smoothEdgeIndices,
                               const TColStd_PackedMapOfInteger& springEdgeIndices,
                               const TColStd_PackedMapOfInteger& crossEdgeIndices,
                               const TColStd_PackedMapOfInteger& terminatingEdgeIndices,
                               const TColStd_PackedMapOfInteger& supportFaceIndices);

        virtual bool IsConstRadius() const override;
        virtual double GetMaxRadius() const override;
        virtual double GetMinRadius() const override;
        virtual double GetRadius() const override;
        virtual double GetAvgRadius() const override;
        virtual BlendFType GetBlendType() const override;
        virtual double GetAngle() const override;
        virtual const TColStd_PackedMapOfInteger &GetCliffEdgeIndices() const override;
        virtual const TColStd_PackedMapOfInteger &GetSmoothEdgeIndices() const override;
        virtual const TColStd_PackedMapOfInteger &GetSpringEdgeIndices() const override;
        virtual const TColStd_PackedMapOfInteger &GetCrossEdgeIndices() const override;
        virtual const TColStd_PackedMapOfInteger &GetTerminatingEdgeIndices() const override;
        virtual const TColStd_PackedMapOfInteger &GetSupportFaceIndices() const override;

        virtual void UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger &indices) override;
        virtual void SubtractTerminatingEdgeIndices(
        const TColStd_PackedMapOfInteger &indices) override;

   private:
    double m_radius = 0;  // 半径
    double m_angle = 0; //过渡面对应横截面圆心角
    TColStd_PackedMapOfInteger m_smoothEdgeIndices;
    TColStd_PackedMapOfInteger m_springEdgeIndices;
    TColStd_PackedMapOfInteger m_crossEdgeIndices;
    TColStd_PackedMapOfInteger m_terminatingEdgeIndices;
    TColStd_PackedMapOfInteger m_supportFaceIndices;
};
