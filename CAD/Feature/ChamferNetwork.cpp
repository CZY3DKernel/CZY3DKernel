#include "./ChamferNetwork.h"

#include <assert.h>
#include <queue>

void ChamferNetwork::ChangeToChains(const std::vector<std::vector<TopoDS_Face>>& faceChains,
                           std::vector<TColStd_PackedMapOfInteger>& chains)
{
    chains.clear();
    chains.reserve(faceChains.size());
    for (int i = 0; i < faceChains.size(); ++i)
    {
        TColStd_PackedMapOfInteger faceIds;
        for (const TopoDS_Face& face : faceChains[i])
        {
            const int faceId = m_aag->GetFaceId(face);
            faceIds.Add(faceId);
        }

        chains.push_back(std::move(faceIds));
    }
}

static void ShowGraph(const std::vector<TColStd_PackedMapOfInteger>& chainToChains)
{
#ifdef _DEBUG
    puts("Show graph");
    int i = 0;
    for (const auto& chains : chainToChains)
    {
        printf("%d: ", i);
        ++i;
        for (TColStd_PackedMapOfInteger::Iterator it(chains);it.More(); it.Next())
        {
            printf("%d, ", it.Key());
        }

        puts("");
    }
#endif
}

ChamferNetwork::ChamferNetwork(const std::shared_ptr<AttributeAdjacentGraph>& aag,
                           std::vector<std::vector<TopoDS_Face>>&& faceChains)
    : m_aag(aag)
{
    // index start from 0
    std::vector<TColStd_PackedMapOfInteger> chains;
    
    ChangeToChains(faceChains, chains);

    //NCollection_IndexedDataMap<int, int> m_faceIdToChainId;
    m_faceIdToChainId.Clear();
    for (int i = 0; i < chains.size(); ++i)
    {
#ifdef _DEBUG
        printf("chain[%d] = {",i);
        #endif
        for (TColStd_PackedMapOfInteger::Iterator it(chains[i]); it.More(); it.Next())
        {
            m_faceIdToChainId.Add(it.Key(), i);
#ifdef _DEBUG
            printf("%d, ", it.Key());
            #endif
        }
#ifdef _DEBUG
        printf("}\n");
        #endif
    }

#ifdef _DEBUG
    puts("face map to chain:");
    for (NCollection_IndexedDataMap<int, int>::Iterator it(m_faceIdToChainId);it.More(); it.Next())
    {
        printf("%d -> %d\n", it.Key(), it.Value());
    }
    #endif

    // chamfer chain relative order, used to topo sort
    //std::vector<TColStd_PackedMapOfInteger> m_chainToChains;

    m_chainToChains.resize(chains.size());
    for (int i = 0; i < chains.size(); ++i)
    {
        for (TColStd_PackedMapOfInteger::Iterator it(chains[i]); it.More(); it.Next())
        {
            const int faceId = it.Key();
            const int toChainId = m_faceIdToChainId.FindFromKey(faceId);
            const TopoDS_Face& face = m_aag->GetFace(faceId);
            if (m_aag->HasChamferFeature(face))
            {
                std::shared_ptr<ChamferFeature> info = m_aag->GetChamferFeature(face);
                const std::vector<TopoDS_Face>& supportFaces = info->supportFaces;
                for (const TopoDS_Face& supportFace : supportFaces)
                {
                    const int supportFaceId = m_aag->GetFaceId(supportFace);
                    if (m_aag->HasChamferFeature(supportFace))
                    {
                        // build arc support face -> face
                        const int fromChainId = m_faceIdToChainId.FindFromKey(supportFaceId);

                        if (fromChainId != toChainId)
                        {
#ifdef _DEBUG
                            printf("faceId(%d, %d) make graph chain(%d -> %d)\n", supportFaceId,
                                   faceId, fromChainId, toChainId);
                            #endif
                            m_chainToChains[fromChainId].Add(toChainId);
                        }
                    }
                }
            }
        }
    }
    
    #ifdef _DEBUG
    ShowGraph(m_chainToChains);
    #endif

    //std::vector<int> m_chainOrders;  // store chain ids
    //topo sort
    TopoSort(chains);

    m_orderedChains.clear();
    for (int i = 0; i < m_chainOrders.size(); ++i)
    {
        const int chainId = m_chainOrders[i];
        m_orderedChains.push_back(std::move(faceChains[chainId]));
    }

    m_valid = faceChains.size() == m_chainOrders.size();

    if (!m_valid)
    {
        int stop = 0;
    }
    assert(m_valid);
}

void ChamferNetwork::TopoSort(const std::vector<TColStd_PackedMapOfInteger>& chains)
{
    std::vector<int> degrees(chains.size(), 0);
    for (int i = 0; i < m_chainToChains.size(); ++i)
    {
        for (TColStd_PackedMapOfInteger::Iterator it(m_chainToChains[i]); it.More(); it.Next())
        {
            ++degrees[it.Key()];
        }
    }

    m_chainOrders.clear();
    m_chainOrders.reserve(chains.size());

    std::queue<int> que;
    //chain id start from 1
    for (int i = 0; i < degrees.size(); ++i)
    {
        if (0 == degrees[i])
        {
            que.push(i);
        }
    }

    while (!que.empty())
    {
        const int chainId = que.front();
        que.pop();
        m_chainOrders.push_back(chainId);

        for (TColStd_PackedMapOfInteger::Iterator it(m_chainToChains[chainId]); it.More();
             it.Next())
        {
            int& d = degrees[it.Key()];
            --d;
            if (0 == d)
            {
                que.push(it.Key());
            }
        }
    }

}

bool ChamferNetwork::Valid() const
{
    //there is circle
    return m_valid;
}


std::vector<std::vector<TopoDS_Face>> ChamferNetwork::GetOrderedChains() const
{
    return m_orderedChains;
}
