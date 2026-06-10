#include "./BlendProperty.h"

#include <assert.h>

#include <TColStd_PackedMapOfInteger.hxx>

BlendProperty::BlendProperty(int faceId) : m_faceId(faceId) {}

BlendFType DetermineBlendTypeBySpringNums(const int num)
{
    BlendFType blendType = BlendFType::BlendType_Uncertain;
    switch (num)
    {
        case 1:  // cliff blend
        {
            return BlendFType::BlendType_Cliff;
        }
        case 2:  // ordinary
        {
            return BlendFType::BlendType_Ordinary;
        }
        
        default:
            break;
    }

    return BlendFType::BlendType_Uncertain;
}

int BlendProperty::GetFaceId() const
{
    return m_faceId;
}

static std::string GetBlendTypeName(const BlendFType type)
{
    switch (type)
    {
        case BlendFType::BlendType_Cliff:
        {
            return "cliff blend";
        }
        case BlendFType::BlendType_Ordinary:
        {
            return "ordinary blend";
        }
        case BlendFType::BlendType_Vertex:
        {
            return "vertex blend";
        }
        case BlendFType::BlendType_Uncertain:
        {
            return "uncertain";
        }
        default:
        {
            assert(false);
            return "invalid";
        }
    }

    return "invalid";
}

void ReportEdges(const TColStd_PackedMapOfInteger& edges, const std::string& edgeName) {
    printf("%s = {", edgeName.c_str());
    for (TColStd_PackedMapOfInteger::Iterator it(edges); it.More(); it.Next())
    {
        printf("%d, ", it.Key());
    }

    puts("}");
}

void BlendProperty::Report() {
    printf("faceId = %d\n", GetFaceId());
    printf("blendType = %s\n", GetBlendTypeName(GetBlendType()).c_str());

    //virtual const TColStd_PackedMapOfInteger &GetSmoothEdgeIndices() const = 0;
    ReportEdges(GetSmoothEdgeIndices(), "smooth edges");

    //virtual const TColStd_PackedMapOfInteger &GetCliffEdgeIndices() const = 0;
    ReportEdges(GetCliffEdgeIndices(), "cliff edges");

    //virtual const TColStd_PackedMapOfInteger &GetSpringEdgeIndices() const = 0;
    ReportEdges(GetSpringEdgeIndices(), "spring edges");

    //virtual const TColStd_PackedMapOfInteger &GetCrossEdgeIndices() const = 0;
    ReportEdges(GetCrossEdgeIndices(), "cross edges");
    
    //virtual const TColStd_PackedMapOfInteger &GetTerminatingEdgeIndices() const = 0;
    ReportEdges(GetTerminatingEdgeIndices(), "terminating edges");
    
    //virtual const TColStd_PackedMapOfInteger &GetSupportFaceIndices() const = 0;
    ReportEdges(GetSupportFaceIndices(), "support faces");
}
