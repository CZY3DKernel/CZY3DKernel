
#ifndef _SHAPE2TOPO_HeaderFile
#define _SHAPE2TOPO_HeaderFile
#include "Topo.hxx"
#include <map>
#include "../Common/Extension_Export.hxx"

class Standard_Transient;
class TopoDS_Vertex;
class TopoDS_Edge;
class TopoDS_Wire;
class TopoDS_Face;
class TopoDS_Shell;
class TopoDS_Solid;
class TopoDS_CompSolid;
class TopoDS_Compound;
class TopoDS_Shell;

class TopoDS_TVertex;
class TopoDS_TEdge;
class TopoDS_TWire;
class TopoDS_TFace;
class TopoDS_TShell;
class TopoDS_TSolid;
class TopoDS_TCompSolid;
class BRep_Builder;
class gp_Pnt;

class Geom2d_Curve;
class EdgeOnFaceBoolInfo;
class SectionEdgeBoolInfo;
class gp_Pnt2d;
class Hash2TopoMap {
	public: 
		std::map<size_t, VertexPtr> vertexMap;
		std::map<size_t, size_t> edgeMap;
		std::map<size_t, size_t> faceMap;
		std::vector<std::vector<std::pair<EdgePtr, TopoDS_Edge>>> edgeOCCVector;
		std::vector<std::vector<std::pair<FacePtr, TopoDS_Face>>> faceOCCVector;
};

class BrepTranslator
{
public:
	// from occt brep
	EXTENSION_EXPORT static VertexPtr fromOCC(const TopoDS_Vertex& occVertex, Hash2TopoMap& topoMap);

	EXTENSION_EXPORT static EdgePtr fromOCC(const TopoDS_Edge& occEdge, Hash2TopoMap&);

	EXTENSION_EXPORT static CoEdgePtr fromOCC(const TopoDS_Edge& occEdge, const TopoDS_Face& occFace, Hash2TopoMap&);

	EXTENSION_EXPORT static WirePtr fromOCC(const TopoDS_Wire& occWire, const TopoDS_Face& occtFace, Hash2TopoMap&);

	EXTENSION_EXPORT static CoFacePtr fromOCC(const TopoDS_Face& occFace, Hash2TopoMap&);

	EXTENSION_EXPORT static ShellPtr fromOCC(const TopoDS_Shell& occShell);

	EXTENSION_EXPORT static ShellPtr fromOCC(const TopoDS_Solid& occSolid);
	;
	EXTENSION_EXPORT static ShellPtr fromOCC(const TopoDS_CompSolid& occCompSolid);

	// to occt brep
	EXTENSION_EXPORT static TopoDS_Vertex toOCC(BRep_Builder BB, const VertexPtr& V, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Edge toOCC(BRep_Builder BB, const EdgePtr& E, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Edge toOCC(BRep_Builder BB, const CoEdgePtr& CE, const FacePtr& F, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Wire toOCC(BRep_Builder BB, const WirePtr& W, const FacePtr& F, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Face toOCC(BRep_Builder BB, const FacePtr& F, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Face toOCC(BRep_Builder BB, const CoFacePtr& F, std::map<void*, Handle(Standard_Transient)>&);

	EXTENSION_EXPORT static TopoDS_Shell toOCC(const ShellPtr&);

	// for debug
	EXTENSION_EXPORT static TopoDS_Vertex toOCC(const gp_Pnt& pt);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<gp_Pnt>& pts);
	EXTENSION_EXPORT static TopoDS_Vertex toOCC(const VertexPtr& v);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<VertexPtr>& vs);
	EXTENSION_EXPORT static TopoDS_Edge toOCC(const Handle(Geom_Curve)& curve, double s, double e);
	EXTENSION_EXPORT static TopoDS_Edge toOCC(const EdgePtr& edge);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<EdgePtr>& edges);
	EXTENSION_EXPORT static TopoDS_Edge toOCC(const CoEdgePtr& coedge);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<CoEdgePtr>& coedges);
	EXTENSION_EXPORT static TopoDS_Wire toOCC(const WirePtr& wire);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<WirePtr>& wires);
	EXTENSION_EXPORT static TopoDS_Face toOCC(const FacePtr& face);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<FacePtr>& faces);
	EXTENSION_EXPORT static TopoDS_Face toOCC(const CoFacePtr& coface);
	EXTENSION_EXPORT static TopoDS_Compound toOCC(const std::vector<CoFacePtr>& cofaces);
};
#endif