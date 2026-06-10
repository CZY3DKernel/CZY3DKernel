#pragma once

#include <vector>

#include <TColStd_PackedMapOfInteger.hxx>

#include <NCollection_IndexedDataMap.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include "./AttributeAdjacentGraph.h"

class ChamferNetwork
{
   public:
    ChamferNetwork(const std::shared_ptr<AttributeAdjacentGraph>& aag,
                 std::vector<std::vector<TopoDS_Face>>&& faceChains);

    bool Valid() const;

    std::vector<std::vector<TopoDS_Face>> GetOrderedChains() const;

   protected:

    void ChangeToChains(const std::vector<std::vector<TopoDS_Face>>& faceChains,
                                      std::vector<TColStd_PackedMapOfInteger>& chains);
    void TopoSort(const std::vector<TColStd_PackedMapOfInteger>& chains);
    std::shared_ptr<AttributeAdjacentGraph> m_aag;

    NCollection_IndexedDataMap<int, int> m_faceIdToChainId;

    //blend chain relative order, used to topo sort
    std::vector<TColStd_PackedMapOfInteger> m_chainToChains;

    std::vector<int> m_chainOrders;//store chain ids
    std::vector<std::vector<TopoDS_Face>> m_orderedChains;

    bool m_valid = false;
};
