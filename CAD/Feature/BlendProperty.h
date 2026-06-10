#pragma once

#include <TColStd_PackedMapOfInteger.hxx>

#include <NCollection_IndexedDataMap.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include "../Common/Extension_Export.hxx"

EXTENSION_EXPORT enum class BlendFType  // F = Feature
{
    BlendType_Uncertain = 0,  //!< Uncertain blend type for recognizer. 
    BlendType_Ordinary = 1,   //!< Ordinary blend.  blend between face and face
    BlendType_Vertex = 2,     //!< Vertex blend. 
    BlendType_Cliff = 3,       //!< Cliff blend (a special case of ordinary blend).
    BlendType_MultiToMulti = 4
};

EXTENSION_EXPORT class BlendProperty
{
   public:
       
    BlendProperty(int faceId);

    /*Get Functions*/
       int GetFaceId() const;
       virtual bool IsConstRadius() const = 0;
       virtual double GetMaxRadius() const = 0;
       virtual double GetMinRadius() const = 0;
       virtual double GetRadius() const = 0;
       virtual double GetAvgRadius() const = 0;
       virtual BlendFType GetBlendType() const = 0;
       virtual double GetAngle() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetCliffEdgeIndices() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetSmoothEdgeIndices() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetSpringEdgeIndices() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetCrossEdgeIndices() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetTerminatingEdgeIndices() const = 0;
       virtual const TColStd_PackedMapOfInteger &GetSupportFaceIndices() const = 0;
      virtual void UniteCrossEdgeIndices(const TColStd_PackedMapOfInteger& indices) = 0;
       virtual void SubtractTerminatingEdgeIndices(const TColStd_PackedMapOfInteger &indices) = 0;

    /*Set Functions*/
        void SetFaceId(int id) { m_faceId = id; }

        void Report();

   protected:
    int m_faceId = 0;  // OCC面id,从1开始
};

BlendFType DetermineBlendTypeBySpringNums(const int num);

using CskIndexedDataMapOfShapeBlendProperty=NCollection_IndexedDataMap<TopoDS_Shape, std::shared_ptr<BlendProperty>, TopTools_ShapeMapHasher>;