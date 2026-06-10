#pragma once

#include <TColStd_PackedMapOfInteger.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

class AttributeAdjacentGraph;

class SupportFacesFinder
{
   public:
    SupportFacesFinder(std::shared_ptr<AttributeAdjacentGraph> aag) : m_aag(aag) {}
   
    //TColStd_PackedMapOfInteger PerformForFace(const int face_idx);
    /*
    * @brief find support faces by current faces and spring edges
    * @param current face
    * @param spring edge ids
    * @return support face ids
    */
    TColStd_PackedMapOfInteger PerformForFace(const TopoDS_Face& face, const TColStd_PackedMapOfInteger& springEdgeIds);

   private:
    std::shared_ptr<AttributeAdjacentGraph> m_aag;
};
