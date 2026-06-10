#pragma once

#include <vector>
#include <set>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

class Geom_Surface;
class AttributeAdjacentGraph;
class BlendProperty;

class ToroidalBlendRecognizer
{
   public:
    ToroidalBlendRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag) : m_aag(aag) {}
    
    /*
     * @brief check if face is Toroidal blend
     * @param face
     * @param[out] if face is blend then collect the blend info
     * @return true if face is blend; false otherwise
     */
    bool IsBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo);
    
   private:
       /*
       * @brief find spring edges from smooth edges
       * @param smooth edges
       * @param toroidal surface
       * @param face
       * @return spring edges indices;
       */
    TColStd_PackedMapOfInteger FindSpringEdges(
        const TColStd_PackedMapOfInteger& smoothEdgeIndices, Handle(Geom_Surface) surface,
        const TopoDS_Face& face);
    
    /*
     * @brief find cross edges from smooth edges
     * @param smooth edges other than spring edges
     * @param toroidal surface
     * @return cross edges indices;
     */
    TColStd_PackedMapOfInteger FindCrossEdges(
        const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface);

    /*
     * @brief find cliff edges
     * @note cliff edge don't belongs to smooth edges
     * @param face
     * @param smooth edges
     * @param toroidal surface
     * @return cliff edges indices;
     */
    TColStd_PackedMapOfInteger FindCliffEdges(
        const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
        Handle(Geom_Surface) surface);

    std::shared_ptr<AttributeAdjacentGraph> m_aag;
};
