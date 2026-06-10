#include "./BlendRecognizer.h"

/*
曲面类型   类型名称                     是否为解析曲面
平面       Geom_Plane                     是
圆柱面     Geom_CylindricalSurface        是
圆锥面     Geom_ConicalSurface            是
球面       Geom_SphericalSurface          是
圆环面     Geom_ToroidalSurface           是
B样条曲面  Geom_BSplineSurface            否
贝塞尔曲面 Geom_BezierSurface             否
旋转曲面   Geom_SurfaceOfRevolution       可能
拉伸曲面   Geom_SurfaceOfLinearExtrusion  可能
裁剪曲面   Geom_RectangularTrimmedSurface 否
偏移曲面   Geom_OffsetSurface             否
*/

#include <assert.h>
#include <queue>

#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Handle.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <gp_Cylinder.hxx>

#include "./BlendPropertyFEConstRadius.h"
#include "./BlendPropertyFEVariousRadius.h"
#include "./BlendPropertyFFConstRadius.h"
#include "./BlendPropertyFFVariousRadius.h"
#include "./BlendPropertyVertexConstRadius.h"
#include "./BlendPropertyVertexVariousRadius.h"

#include "./VertexBlendRecognizer.h"
#include "./CylinderBlendRecognizer.h"
#include "./ConicBlendRecognizer.h"
#include "./ToroidalBlendRecognizer.h"
#include "./EdgeBlendRecognizer.h"
#include "./FeatureCommon.h"
#include "./BlendNetwork.h"

BlendRecognizer::BlendRecognizer(const TopoDS_Shape& shape_)
    : m_aag(std::make_shared<AttributeAdjacentGraph>(shape_))
{
}

/*
* @brief check if face is edge blend
* @param face
* @param[out] if face is edge blend then collect the blend info
* @return true if face is edge blend; false otherwise
*/
bool BlendRecognizer::IsEdgeBlend(const TopoDS_Face& face, std::shared_ptr<BlendProperty>& outInfo) {
    if (face.IsNull())
    {
        return false;
    }

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    std::cout << "surface type = " << surface->get_type_name() << std::endl;
    //judge blend according face type(cylinder, torus, sphere, bspline, bezier)
    if (FeatureCommon::IsPlane(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsCylindricalSurface(surface))
    {
        return CylinderBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsToroidalSurface(surface))
    {
        return ToroidalBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsConicalSurface(surface))
    {
        return ConicBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsSphericalSurface(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsBSplineSurface(surface))
    {
        //call general recognizer
        return EdgeBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsBezierSurface(surface))
    {
        // call general recognizer
        return EdgeBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsExtrusionSurface(surface))
    {
        return EdgeBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsSurfaceOfRevolution(surface))
    {
        return EdgeBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsOffsetSurface(surface))
    {
        return EdgeBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else
    {
        assert(false);
    }

    return false;
}

/*
 * @brief check if face is vertex blend
 * @param face
 * @param[out] if face is vertex blend then collect the blend info
 * @return true if face is vertex blend; false otherwise
 */
bool BlendRecognizer::IsVertexBlend(const TopoDS_Face& face,
                                  std::shared_ptr<BlendProperty>& outInfo)
{
    if (face.IsNull())
    {
        return false;
    }

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    std::cout << "surface type = " << surface->get_type_name() << std::endl;
    // judge blend according face type(cylinder, torus, sphere, bspline, bezier)
    if (FeatureCommon::IsPlane(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsConicalSurface(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsCylindricalSurface(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsToroidalSurface(surface))
    {
        return VertexBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsSphericalSurface(surface))
    {
        return VertexBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsBSplineSurface(surface))
    {
        // call general recognizer
        return VertexBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsBezierSurface(surface))
    {
        // call general recognizer
        return VertexBlendRecognizer(m_aag).IsBlend(face, outInfo);
    }
    else if (FeatureCommon::IsExtrusionSurface(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsSurfaceOfRevolution(surface))
    {
        return false;
    }
    else if (FeatureCommon::IsOffsetSurface(surface))
    {
        return false;
    }
    else
    {
        assert(false);
    }

    return false;
}

void BlendRecognizer::RefineTerminatingEdgeAsCrossEdge(const int faceId)
{
#ifdef _DEBUG
    if (faceId == 2)
    {
        int stop = 0;
    }
#endif
    const TopoDS_Face& face = m_aag->GetFace(faceId);
    if (!m_aag->HasBlendProperty(face))
    {
        return;
    }

    std::shared_ptr<BlendProperty> blendInfo = m_aag->GetBlendProperty(face);
    const TColStd_PackedMapOfInteger& terminatingEdgeIndices =
        blendInfo->GetTerminatingEdgeIndices();

    TColStd_PackedMapOfInteger suspectedTerminating;

    for (TColStd_PackedMapOfInteger::Iterator it(terminatingEdgeIndices); it.More(); it.Next())
    {
        const int edgeId = it.Key();
        const TopoDS_Edge& edge = m_aag->GetEdge(edgeId);
        const TopoDS_Face& partner = m_aag->GetNeibourFace(face, edge);
        if (partner.IsNull())
        {
            continue;
        }

#ifdef _DEBUG
        printf("partner face Id = %d\n", m_aag->GetFaceId(partner));
#endif

        if (m_aag->HasBlendProperty(partner))
        {
            suspectedTerminating.Add(edgeId);
        }
    }

    blendInfo->UniteCrossEdgeIndices(suspectedTerminating);

    blendInfo->SubtractTerminatingEdgeIndices(suspectedTerminating);
}

static void CheckBlendInfo(const int faceId, std::shared_ptr<AttributeAdjacentGraph> aag)
{
    if (faceId >= 1 && faceId <= aag->GetFaces().Extent())
    {
        const TopoDS_Face& face = aag->GetFace(faceId);
        if (aag->HasBlendProperty(face))
        {
            std::shared_ptr<BlendProperty> blendProperty = aag->GetBlendProperty(face);

            printf("faceId = %d\n", faceId);
            printf("cross count = %d\n", blendProperty->GetCrossEdgeIndices().Extent());
        }
    }
}

CskIndexedDataMapOfShapeBlendProperty BlendRecognizer::RecognizeBlends()
{
    const TopTools_IndexedMapOfShape& faces = m_aag->GetFaces();

    //recognize edge blend
    for (int id = 1; id <= faces.Extent(); ++id)
    {
#ifdef _DEBUG
        if (id == 286 || id == 287 || id == 288)
        {
            int stop = 0;
        }

#endif
        std::shared_ptr<BlendProperty> info;
        TopoDS_Face face = TopoDS::Face(faces.FindKey(id));
        if (IsEdgeBlend(face, info))
        {
            info->SetFaceId(id);
            m_aag->AddBlendInfo(face, info);
        }
    }

    #ifdef _DEBUG
    CheckBlendInfo(286, m_aag);
    CheckBlendInfo(287, m_aag);
    CheckBlendInfo(288, m_aag);
    #endif

    //recognize vertex blend
    for (int id = 1; id <= faces.Extent(); ++id)
    {
        TopoDS_Face face = TopoDS::Face(faces.FindKey(id));
        if (m_aag->HasBlendProperty(face))
        {
            //face is blend vertex
            continue;
        }

        std::shared_ptr<BlendProperty> info;
        if (IsVertexBlend(face, info))
        {
            info->SetFaceId(id);
            m_aag->AddBlendInfo(face, info);
        }
    }

    #ifdef _DEBUG
    CheckBlendInfo(286, m_aag);
    CheckBlendInfo(287, m_aag);
    CheckBlendInfo(288, m_aag);
    #endif

    /* ===========================================================
     *  Reconsider some terminating edges as cross edges
     * =========================================================== */
    for (int id = 1; id <= faces.Extent(); ++id)
    {
        RefineTerminatingEdgeAsCrossEdge(id);
    }
    
    #ifdef _DEBUG
    CheckBlendInfo(286, m_aag);
    CheckBlendInfo(287, m_aag);
    CheckBlendInfo(288, m_aag);
    
    #endif

    return m_aag->GetBlendProperties();
}

bool BlendRecognizer::RecognizeBlend(const int faceId, std::shared_ptr<BlendProperty>& info) {
    const TopoDS_Face& face = m_aag->GetFace(faceId);
    return IsEdgeBlend(face, info);
}

bool BlendRecognizer::RecognizeBlend(const int faceId)
{
    const TopoDS_Face& face = m_aag->GetFace(faceId);
    std::shared_ptr<BlendProperty> info;
    return IsEdgeBlend(face, info);
}

void BlendRecognizer::FetchBlendChain(const int seed, std::vector<TopoDS_Face>& faces,
                                      std::vector<int>& visited)
{
    std::queue<int> faceQue;
    faceQue.push(seed);
    visited[seed] = true;

    while (!faceQue.empty())
    {
        const int faceId = faceQue.front();
        faceQue.pop();

        const TopoDS_Face& current = m_aag->GetFace(faceId);
        faces.push_back(current);

        std::shared_ptr<BlendProperty> info = m_aag->GetBlendProperty(current);

        const TColStd_PackedMapOfInteger& crossEdgeIndices = info->GetCrossEdgeIndices();

        for (TColStd_PackedMapOfInteger::Iterator it(crossEdgeIndices); it.More(); it.Next())
        {
            const int edgeId = it.Key();
            const TopoDS_Edge& edge = m_aag->GetEdge(edgeId);

            const TopoDS_Face& partner = m_aag->GetNeibourFace(current, edge);
            if (partner.IsNull())
            {
                continue;
            }

            const int partnerId = m_aag->GetFaceId(partner);
            if (m_aag->HasBlendProperty(partner))
            {
                std::shared_ptr<BlendProperty> partnerProperty = m_aag->GetBlendProperty(partner);
                if (partnerProperty->GetCrossEdgeIndices().Contains(edgeId))
                {
                    if (!visited[partnerId])
                    {
                        visited[partnerId] = true;
                        faceQue.push(partnerId);
                    }
                }
            }
        }
    }
}

std::vector<std::vector<TopoDS_Face> > BlendRecognizer::ExtractBlendChains() {

    RecognizeBlends();

    //connect faces by cross edges
    std::vector<int> visited(m_aag->GetFaces().Extent() + 1);

    const int faceCount = m_aag->GetFaces().Extent();

    std::vector<std::vector<TopoDS_Face> > result;

    for (int i = 1; i <= faceCount; ++i)
    {
        if (visited[i])
        {
            continue;
        }

        const TopoDS_Face& face = m_aag->GetFace(i);
        if (!m_aag->HasBlendProperty(face))
        {
            continue;
        }

        std::vector<TopoDS_Face> faces;

        FetchBlendChain(i, faces, visited);

        result.push_back(faces);
    }

    return result;
}

std::vector<std::vector<TopoDS_Face> > BlendRecognizer::ExtractOrderedBlendChains()
{
    std::vector<std::vector<TopoDS_Face>> faceChains = ExtractBlendChains();

    BlendNetwork bn(m_aag, std::move(faceChains));

    return bn.GetOrderedChains();
}

std::shared_ptr<AttributeAdjacentGraph> BlendRecognizer::GetAag() const
{
    return m_aag;
}
