#pragma once

#include <vector>
#include <set>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <NCollection_IndexedDataMap.hxx>

#include "./BlendProperty.h"
#include "./AttributeAdjacentGraph.h"



EXTENSION_EXPORT class BlendRecognizer
{
   public:
        BlendRecognizer(const TopoDS_Shape& shape_);

        std::vector<std::vector<TopoDS_Face> > ExtractOrderedBlendChains();
        std::vector<std::vector<TopoDS_Face> > ExtractBlendChains();
        CskIndexedDataMapOfShapeBlendProperty RecognizeBlends();

    std::shared_ptr<AttributeAdjacentGraph> GetAag() const;

     bool RecognizeBlend(const int faceId);
     bool RecognizeBlend(const int faceId, std::shared_ptr<BlendProperty>& info);

   private:
    /*
     * @brief check if face is blend
     * @param face
     * @param[out] if face is blend then collect the blend info
     * @return true if face is blend; false otherwise
     */
    bool IsEdgeBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo);

    /*
     * @brief check if face is vertex blend
     * @param face
     * @param[out] if face is vertex blend then collect the blend info
     * @return true if face is vertex blend; false otherwise
     */
    bool IsVertexBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo);

    /*
     * @brief refine terminating edge as cross edge
     * @param faceid
     */
    void RefineTerminatingEdgeAsCrossEdge(const int faceId);

    /*
    * @brief fetch blend chain by seed face
    * @param seed faceid
    * @param[out] face chain
    * @param[out] visited faces
    */
    void FetchBlendChain(const int seed, std::vector<TopoDS_Face>& faces,
                         std::vector<int>& visited);

    std::shared_ptr<AttributeAdjacentGraph> m_aag;

};
