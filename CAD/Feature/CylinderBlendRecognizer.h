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

class CylinderBlendRecognizer
{
   public:
    CylinderBlendRecognizer(std::shared_ptr<AttributeAdjacentGraph> aag) : m_aag(aag) {}
    
    /*
     * @brief check if face is cylinder blend
     * @param face
     * @param[out] if face is blend then collect the blend info
     * @return true if face is blend; false otherwise
     */
    bool IsBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo);
    
   private:

       /*
       * @brief check support face radius big than current face
       * @param radius, of current face
       * @param current face
       * @param spring edge indices
       * @return true if radius regular passed; false otherwise
       */
       bool CheckRadiusRegular(const double radius, const TopoDS_Face& face, const TColStd_PackedMapOfInteger& springEdgeIndices);

       /*
       * @brief check if there is seam edge
       * @param edgeIndices
       * @param face
       * @return true if there is seam edge; false otherwise
       */
       bool HasSeamEdge(const TColStd_PackedMapOfInteger& smoothEdgeIndices,
                        const TopoDS_Face& face);

       /*
       * @brief find spring edges from smooth edges
       * @param smooth edges
       * @param cylinder surface
       * @return spring edges indices;
       */
    TColStd_PackedMapOfInteger FindSpringEdges(
        const TColStd_PackedMapOfInteger& smoothEdgeIndices,
        const TopoDS_Face& face, Handle(Geom_Surface) surface);
    
    /*
     * @brief find cross edges from smooth edges
     * @param smooth edges other than spring edges
     * @param cylinder surface
     * @return cross edges indices;
     */
    TColStd_PackedMapOfInteger FindCrossEdges(
        const TColStd_PackedMapOfInteger& lestSmoothEdges, Handle(Geom_Surface) surface);

    /*
     * @brief find cliff edges
     * @note cliff edge don't belongs to smooth edges
     * @param face
     * @param smooth edges
     * @param cylinder surface
     * @return cliff edges indices;
     */
    TColStd_PackedMapOfInteger FindCliffEdges(
        const TopoDS_Face& face, const TColStd_PackedMapOfInteger& smoothEdgeIndices,
        Handle(Geom_Surface) surface);

    std::shared_ptr<AttributeAdjacentGraph> m_aag;
};
