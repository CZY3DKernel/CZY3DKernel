
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>

#pragma once

class CompEdge
{
   public:
    CompEdge() = default;
    CompEdge(const std::vector<TopoDS_Edge>& edges) : m_edges(edges) {}
    void SetEdges(const std::vector<TopoDS_Edge>& edges) { m_edges = edges; }
    const std::vector<TopoDS_Edge> GetEdges() const { return m_edges; }
    double Length() const;

    TopoDS_Vertex GetStart() const;
    TopoDS_Vertex GetEnd() const;

   private:
    // 复合边，有可能 是一个环，有可能 是链
    std::vector<TopoDS_Edge> m_edges;
};
