#include "./CompEdge.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>

#include <assert.h>

#include "./FeatureCommon.h"

TopoDS_Vertex CompEdge::GetStart() const 
{
    
    ShapeAnalysis_Edge se;
    if (m_edges.size() == 1)
    {
        return se.FirstVertex(m_edges.front());
    }
    else
    {
        TopoDS_Vertex v00 = se.FirstVertex(m_edges.front());

        //gp_Pnt p00 = BRep_Tool::Pnt(v00);

        TopoDS_Vertex v01 = se.LastVertex(m_edges.front());

        //gp_Pnt p01 = BRep_Tool::Pnt(v01);

        TopoDS_Vertex v10 = se.FirstVertex(m_edges[1]);

        //gp_Pnt p10 = BRep_Tool::Pnt(v10);

        TopoDS_Vertex v11 = se.LastVertex(m_edges[1]);

        //gp_Pnt p11 = BRep_Tool::Pnt(v11);

        if (v00.IsSame(v10) || v00.IsSame(v11))
        {
            return v01;
        }
        else if (v01.IsSame(v10)||v01.IsSame(v11))
        {
            return v00;
        }
        else
        {
            assert(false);
        }
    }

    assert(false);
    return {};
}

TopoDS_Vertex CompEdge::GetEnd() const 
{
    ShapeAnalysis_Edge se;
    if (m_edges.size() == 1)
    {
        return se.LastVertex(m_edges.back());
    }
    else
    {
        TopoDS_Vertex v00 = se.LastVertex(m_edges.back());
        TopoDS_Vertex v01 = se.FirstVertex(m_edges.back());

        TopoDS_Vertex v10 = se.LastVertex(m_edges.rbegin()[1]);
        TopoDS_Vertex v11 = se.FirstVertex(m_edges.rbegin()[1]);

        if (v00.IsSame(v10) || v00.IsSame(v11))
        {
            return v01;
        }
        else if (v01.IsSame(v10) || v01.IsSame(v11))
        {
            return v00;
        }
        else
        {
            assert(false);
        }
    }

    assert(false);
        return {};
}

double CompEdge::Length() const 
{
    double sum = 0;
    for (const TopoDS_Edge& edge : m_edges)
    {
        sum += FeatureCommon::CalculateEdgeLength(edge);
    }

    return sum;
}
