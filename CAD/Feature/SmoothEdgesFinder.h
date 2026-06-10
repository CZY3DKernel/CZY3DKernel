#pragma once

#include <TColStd_PackedMapOfInteger.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

class AttributeAdjacentGraph;

class SmoothEdgesFinder
{
   public:
    SmoothEdgesFinder(std::shared_ptr<AttributeAdjacentGraph> aag) : m_aag(aag) {}
   
    //TColStd_PackedMapOfInteger PerformForFace(const int face_idx);
    TColStd_PackedMapOfInteger PerformForFace(const TopoDS_Shape& face);

   private:
    std::shared_ptr<AttributeAdjacentGraph> m_aag;
};