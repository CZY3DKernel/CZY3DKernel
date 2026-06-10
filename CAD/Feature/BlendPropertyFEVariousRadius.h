#pragma once

#include "./BlendProperty.h"

#include <TColStd_PackedMapOfInteger.hxx>

EXTENSION_EXPORT class BlendPropertyFEVariousRadius : public BlendProperty
{
   public:
       BlendPropertyFEVariousRadius(int faceId,
                                 double maxRadius,  // 最大半径
                                 double minRadius,  // 最小半径
                                 double avgRadius,  // 平均半径
                                 double angle,      // 过渡面对应横截面圆心角
                                 TColStd_PackedMapOfInteger cliffEdgeIndices,
                                 TColStd_PackedMapOfInteger smoothEdgeIndices,
                                 TColStd_PackedMapOfInteger springEdgeIndices,
                                 TColStd_PackedMapOfInteger crossEdgeIndices,
                                 TColStd_PackedMapOfInteger terminatingEdgeIndices,
                                 TColStd_PackedMapOfInteger supportFaceIndices);

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
     double m_maxRadius = 0; //最大半径
     double m_minRadius = 0; //最小半径
     double m_avgRadius = 0; //平均半径
     double m_angle = 0; //过渡面对应横截面圆心角
     TColStd_PackedMapOfInteger m_cliffEdgeIndices;
     TColStd_PackedMapOfInteger m_smoothEdgeIndices;
     TColStd_PackedMapOfInteger m_springEdgeIndices;
     TColStd_PackedMapOfInteger m_crossEdgeIndices;
     TColStd_PackedMapOfInteger m_terminatingEdgeIndices;
     TColStd_PackedMapOfInteger m_supportFaceIndices;
};
