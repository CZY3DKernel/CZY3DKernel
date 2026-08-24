#include "BrepTranslator.hxx"
#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_CompSolid.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_TEdge.hxx>
#include <BRep_TEdge.hxx>
#include <TopLoc_Location.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepTools.hxx>
#include <Geom2d_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include "Boolean.hxx"

Standard_EXPORT VertexPtr BrepTranslator::fromOCC(const TopoDS_Vertex& occVertex, Hash2TopoMap& occ2TopoMap) {

	size_t hashValue = occVertex.HashCode(std::numeric_limits<Standard_Integer>::max());
	auto it = occ2TopoMap.vertexMap.find(hashValue);
	if (it != occ2TopoMap.vertexMap.end()) {
		return it->second;
	}
	else {
		VertexPtr V = std::make_shared<Vertex>();
		V->SetPnt(BRep_Tool::Pnt(occVertex));
		V->SetTol(BRep_Tool::Tolerance(occVertex));
		occ2TopoMap.vertexMap[hashValue] = V;
		return V;
	}
}

Standard_EXPORT EdgePtr BrepTranslator::fromOCC(const TopoDS_Edge& occEdge, Hash2TopoMap& occ2TopoMap) {
	EdgePtr E;
	double startParam, lastParam;

	size_t hashValue = occEdge.HashCode(std::numeric_limits<Standard_Integer>::max());
	auto it = occ2TopoMap.edgeMap.find(hashValue);
	if (it != occ2TopoMap.edgeMap.end()) {
		auto& vector = occ2TopoMap.edgeOCCVector[it->second];
		for (auto& pair : vector) {
			if (pair.second.IsSame(occEdge)) {
				E = pair.first;
				break;
			}
		}
	}

	if (E == nullptr)
	{
		E = std::make_shared<Edge>();
		Handle(Geom_Curve) curve = BRep_Tool::Curve(occEdge, startParam, lastParam);
		if (!curve.IsNull())
		{
			auto& trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve);
			if (!trimmedCurve.IsNull())
			{
				curve = trimmedCurve->BasisCurve();
			}
		}

		TopoDS_Vertex startVertex = TopExp::FirstVertex(occEdge);
		TopoDS_Vertex lastVertex = TopExp::LastVertex(occEdge);
		E->SetCurve(curve);
		E->SetFirstVertex(fromOCC(startVertex, occ2TopoMap));
		E->SetLastVertex(fromOCC(lastVertex, occ2TopoMap));
		E->SetFirstParam(startParam);
		E->SetLastParam(lastParam);
		E->SetTolerance(BRep_Tool::Tolerance(occEdge));
		std::pair<EdgePtr, TopoDS_Edge> pair = { E, occEdge };
		size_t vectorIndex;
		if (it != occ2TopoMap.edgeMap.end())
		{
			vectorIndex = it->second;
			occ2TopoMap.edgeOCCVector[it->second].push_back(pair);
		}
		else {
			occ2TopoMap.edgeMap[hashValue] = occ2TopoMap.edgeOCCVector.size();
			occ2TopoMap.edgeOCCVector.push_back({ pair });
		}
	}

	return E;
}

Standard_EXPORT CoEdgePtr BrepTranslator::fromOCC(const TopoDS_Edge& occEdge, const TopoDS_Face& occFace, Hash2TopoMap& occ2TopoMap) {
	const EdgePtr& E = fromOCC(occEdge, occ2TopoMap);

	double startParam, lastParam;
	Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(occEdge, occFace, startParam, lastParam);
	CoEdgePtr coedge = std::make_shared<CoEdge>();
	coedge->SetEdge(E);
	coedge->SetPcurve(pcurve);
	coedge->SetFirstParam(startParam);
	coedge->SetLastParam(lastParam);
	coedge->SetSameOrient(occEdge.Orientation() == TopAbs_FORWARD);

	return coedge;
}

Standard_EXPORT WirePtr BrepTranslator::fromOCC(const TopoDS_Wire& occWire, const TopoDS_Face& occtFace,
	Hash2TopoMap& occ2TopoMap) {
	WirePtr W = std::make_shared<Wire>();

	BRepTools_WireExplorer itEdge(occWire);
	for (; itEdge.More(); itEdge.Next()) {
		const TopoDS_Edge& occEdge = itEdge.Current();
		W->AddCoedge(fromOCC(occEdge, occtFace, occ2TopoMap));
	}
	return W;
}

Standard_EXPORT CoFacePtr BrepTranslator::fromOCC(const TopoDS_Face& occFace,
	Hash2TopoMap& occ2TopoMap) {
	FacePtr F;
	size_t hashValue = occFace.HashCode(std::numeric_limits<Standard_Integer>::max());
	auto it = occ2TopoMap.faceMap.find(hashValue);
	if (it != occ2TopoMap.faceMap.end()) {
		auto& vector = occ2TopoMap.faceOCCVector[it->second];
		for (auto& pair : vector) {
			if (pair.second.IsSame(occFace)) {
				F = pair.first;
				break;
			}
		}
	}

	if (F == nullptr) {
		F = std::make_shared<Face>();
		Handle(Geom_Surface) surface = BRep_Tool::Surface(occFace);
		if (!surface.IsNull())
		{
			auto& trimmedSurface = Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);
			if (!trimmedSurface.IsNull())
			{
				surface = trimmedSurface->BasisSurface();
			}
		}
		F->SetSurface(BRep_Tool::Surface(occFace));
		F->SetTolerance(BRep_Tool::Tolerance(occFace));
		//TODO:
		TopLoc_Location theLocation;
		//F->SetNaturalRestriction(BRep_Tool::NaturalRestriction(occFace));

		TopoDS_Wire outWire = BRepTools::OuterWire(occFace);
		outWire.Orientation(TopAbs::Compose(occFace.Orientation(), outWire.Orientation()));

		F->AddWire(fromOCC(outWire, occFace, occ2TopoMap));

		TopoDS_Iterator itWire(occFace, Standard_False);
		for (; itWire.More(); itWire.Next())
		{
			const TopoDS_Wire& wire = TopoDS::Wire(itWire.Value());
			if (wire.IsEqual(outWire)) {
				continue;
			}
			else {
				F->AddWire(fromOCC(wire, occFace, occ2TopoMap));
			}
		}

		std::pair<FacePtr, TopoDS_Face> pair = { F, occFace };
		if (it != occ2TopoMap.faceMap.end()) {
			occ2TopoMap.faceOCCVector[it->second].push_back(pair);
		}
		else {
			occ2TopoMap.faceMap[hashValue] = occ2TopoMap.faceOCCVector.size();
			occ2TopoMap.faceOCCVector.push_back({ pair });
		}
	}
	CoFacePtr coFace = std::make_shared<CoFace>(F, occFace.Orientation() == TopAbs_FORWARD);
	return coFace;
}

Standard_EXPORT ShellPtr BrepTranslator::fromOCC(const TopoDS_Shell& occShell) {
	Hash2TopoMap occ2TopoMap;
	auto S = std::make_shared<Shell>();
	TopoDS_Iterator itFace(occShell);
	for (; itFace.More(); itFace.Next())
	{
		const TopoDS_Face& occFace = TopoDS::Face(itFace.Value());
		S->AddCoface(fromOCC(occFace, occ2TopoMap));
	}
	return S;
}

Standard_EXPORT ShellPtr BrepTranslator::fromOCC(const TopoDS_Solid& occSolid) {
	Hash2TopoMap occ2TopoMap;
	auto S = std::make_shared<Shell>();
	TopExp_Explorer itFace(occSolid, TopAbs_ShapeEnum::TopAbs_FACE);
	for (; itFace.More(); itFace.Next()) {
		const TopoDS_Face& occFace = TopoDS::Face(itFace.Value());
		S->AddCoface(fromOCC(occFace, occ2TopoMap));
	}
	return S;
}

Standard_EXPORT ShellPtr BrepTranslator::fromOCC(const TopoDS_CompSolid& occCompSolid) {
	Hash2TopoMap occ2TopoMap;
	auto S = std::make_shared<Shell>();
	TopExp_Explorer itFace(occCompSolid, TopAbs_ShapeEnum::TopAbs_FACE);
	for (; itFace.More(); itFace.Next())
	{
		const TopoDS_Face& occFace = TopoDS::Face(itFace.Value());
		S->AddCoface(fromOCC(occFace, occ2TopoMap));
	}
	return S;
}

Standard_EXPORT TopoDS_Vertex BrepTranslator::toOCC(BRep_Builder BB, const VertexPtr& V,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {

	TopoDS_Vertex OCCV;
	Handle(TopoDS_TVertex) TV;
	auto it = topo2OccMap.find(V.get());
	if (it != topo2OccMap.end()) {
		TV = Handle(TopoDS_TVertex)::DownCast(it->second);
	}
	else {
		BB.MakeVertex(OCCV, V->GetPnt(), V->GetTol());
		TV = Handle(TopoDS_TVertex)::DownCast(OCCV.TShape());
		topo2OccMap[V.get()] = TV;
	}

	OCCV.TShape(TV);
	OCCV.Location(TopLoc_Location());
	OCCV.Orientation(TopAbs_FORWARD);

	return OCCV;
}

Standard_EXPORT TopoDS_Edge BrepTranslator::toOCC(BRep_Builder BB, const EdgePtr& E,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {
	TopoDS_Edge OCCE;
	Handle(TopoDS_TEdge) TE;
	auto it = topo2OccMap.find(E.get());
	if (it != topo2OccMap.end()) {
		TE = Handle(TopoDS_TEdge)::DownCast(it->second);
	}
	else {
		BB.MakeEdge(OCCE, E->GetCurve(), E->GetTolerance());
		const VertexPtr& fristV = E->GetFirstVertex();
		const VertexPtr& lastV = E->GetLastVertex();
		const TopoDS_Vertex& OCCTFirstV = toOCC(BB, fristV, topo2OccMap);
		TopoDS_Vertex OCCTLastV = toOCC(BB, lastV, topo2OccMap);
		OCCTLastV.Orientation(TopAbs_REVERSED);

		BB.Add(OCCE, OCCTFirstV);
		BB.Add(OCCE, OCCTLastV);

		//Refer: BRepLib_MakeEdge
		BB.Range(OCCE, E->GetFirstParam(), E->GetLastParam());
		TE = Handle(TopoDS_TEdge)::DownCast(OCCE.TShape());
		topo2OccMap[E.get()] = TE;
	}

	OCCE.TShape(TE);
	OCCE.Location(TopLoc_Location());
	OCCE.Orientation(TopAbs_FORWARD);

	return OCCE;
}

Standard_EXPORT TopoDS_Edge BrepTranslator::toOCC(BRep_Builder BB, const CoEdgePtr& CE, const FacePtr& F,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {
	TopoDS_Edge OCCE = toOCC(BB, CE->GetEdge(), topo2OccMap);

	OCCE.Orientation(CE->GetSameOrient() == true ? TopAbs_FORWARD : TopAbs_REVERSED);

	if (F != nullptr) {
		TopLoc_Location l;
		if (!CE->GetPcurve().IsNull()) {
			BB.UpdateEdge(OCCE, CE->GetPcurve(), F->GetSurface(), l, CE->GetEdge()->GetTolerance());
		}
	}

	return OCCE;
}

Standard_EXPORT TopoDS_Wire BrepTranslator::toOCC(BRep_Builder BB, const WirePtr& W, const FacePtr& F,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {
	TopoDS_Wire OCCW;
	const std::vector<CoEdgePtr>& coedges = W->GetCoEdges();
	BB.MakeWire(OCCW);
	for (int i = 0; i < coedges.size(); i++) {
		const TopoDS_Edge& E = toOCC(BB, coedges[i], F, topo2OccMap);
		BB.Add(OCCW, E);
	}
	return OCCW;
}

Standard_EXPORT TopoDS_Face BrepTranslator::toOCC(BRep_Builder BB, const FacePtr& F,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {

	TopoDS_Face OCCF;
	Handle(TopoDS_TFace) TF;
	auto it = topo2OccMap.find(F.get());
	if (it != topo2OccMap.end()) {
		TF = Handle(TopoDS_TFace)::DownCast(it->second);
	}
	else {
		BB.MakeFace(OCCF, F->GetSurface(), F->GetTolerance());
		const std::vector<WirePtr>& wires = F->GetWires();
		for (int i = 0; i < wires.size(); i++) {
			const TopoDS_Wire OCCW = toOCC(BB, wires[i], F, topo2OccMap);
			BB.Add(OCCF, OCCW);
		}

		TF = Handle(TopoDS_TFace)::DownCast(OCCF.TShape());
		topo2OccMap[F.get()] = OCCF.TShape();
	}

	OCCF.TShape(TF);
	OCCF.Location(TopLoc_Location());
	OCCF.Orientation(TopAbs_FORWARD);

	return OCCF;
}

Standard_EXPORT TopoDS_Face BrepTranslator::toOCC(BRep_Builder BB, const CoFacePtr& F,
	std::map<void*, Handle(Standard_Transient)>& topo2OccMap) {
	TopoDS_Face OCCF = toOCC(BB, F->GetFace(), topo2OccMap);
	OCCF.Orientation(F->GetSameOrient() == true ? TopAbs_FORWARD : TopAbs_REVERSED);
	return OCCF;
}

Standard_EXPORT TopoDS_Shell BrepTranslator::toOCC(const ShellPtr& S) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder BB;
	TopoDS_Shell OCCS;
	BB.MakeShell(OCCS);
	const std::vector<CoFacePtr>& coFaces = S->GetCoFaces();
	for (int i = 0; i < coFaces.size(); i++) {
		const TopoDS_Face& OCCF = toOCC(BB, coFaces[i], topo2OccMap);
		BB.Add(OCCS, OCCF);
	}
	return OCCS;
}

Standard_EXPORT  TopoDS_Vertex BrepTranslator::toOCC(const gp_Pnt& pt) {
	BRep_Builder BB;
	TopoDS_Vertex OCCV;
	BB.MakeVertex(OCCV, pt, 1.0e-3);
	return OCCV;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<gp_Pnt>& pts) {
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (auto pt : pts) {
		TopoDS_Vertex v = toOCC(pt);
		builder.Add(compound, v);
	}
	return compound;
}

Standard_EXPORT  TopoDS_Vertex BrepTranslator::toOCC(const VertexPtr& v) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Vertex occV = toOCC(builder, v, topo2OccMap);
	return occV;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<VertexPtr>& vs) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& v : vs) {
		TopoDS_Vertex occtV = toOCC(builder, v, topo2OccMap);
		builder.Add(compound, occtV);
	}
	return compound;
}

Standard_EXPORT  TopoDS_Edge BrepTranslator::toOCC(const EdgePtr& edge) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Edge occtEdge = toOCC(builder, edge, topo2OccMap);
	return occtEdge;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<EdgePtr>& edges) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& edge : edges) {
		TopoDS_Edge occtEdge = toOCC(builder, edge, topo2OccMap);
		builder.Add(compound, occtEdge);
	}
	return compound;
}
Standard_EXPORT  TopoDS_Edge BrepTranslator::toOCC(const CoEdgePtr& coedge) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Edge occCoedge = toOCC(builder, coedge, nullptr, topo2OccMap);
	return occCoedge;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<CoEdgePtr>& coedges) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& coedge : coedges) {
		TopoDS_Edge occCoedge = toOCC(builder, coedge, nullptr, topo2OccMap);
		builder.Add(compound, occCoedge);
	}
	return compound;
}
Standard_EXPORT  TopoDS_Wire BrepTranslator::toOCC(const WirePtr& wire) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	TopoDS_Wire occWire = toOCC(builder, wire, nullptr, topo2OccMap);
	return occWire;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<WirePtr>& wires) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& wire : wires) {
		TopoDS_Wire occWire = toOCC(builder, wire, nullptr, topo2OccMap);
		builder.Add(compound, occWire);
	}
	return compound;
}
Standard_EXPORT  TopoDS_Face BrepTranslator::toOCC(const FacePtr& face) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	TopoDS_Face occFace = toOCC(builder, face, topo2OccMap);
	return occFace;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<FacePtr>& faces) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& face : faces) {
		TopoDS_Face occFace = toOCC(builder, face, topo2OccMap);
		builder.Add(compound, occFace);
	}
	return compound;
}

Standard_EXPORT  TopoDS_Face BrepTranslator::toOCC(const CoFacePtr& coface) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	TopoDS_Face occFace = toOCC(builder, coface, topo2OccMap);
	return occFace;
}
Standard_EXPORT  TopoDS_Compound BrepTranslator::toOCC(const std::vector<CoFacePtr>& cofaces) {
	std::map<void*, Handle(Standard_Transient)> topo2OccMap;
	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);
	for (const auto& coface : cofaces) {
		TopoDS_Face occFace = toOCC(builder, coface, topo2OccMap);
		builder.Add(compound, occFace);
	}
	return compound;
}

Standard_EXPORT  TopoDS_Edge BrepTranslator::toOCC(const Handle(Geom_Curve)& curve, double s, double e) {
	BRep_Builder builder;
	TopoDS_Edge occE;
	builder.MakeEdge(occE, curve, distTol);

	gp_Pnt P;
	curve->D0(s, P);
	TopoDS_Vertex occFirstV = toOCC(P);

	curve->D0(e, P);
	TopoDS_Vertex occLastV = toOCC(P);
	occLastV.Orientation(TopAbs_REVERSED);

	builder.Add(occE, occFirstV);
	builder.Add(occE, occLastV);

	//Refer: BRepLib_MakeEdge
	builder.Range(occE, s, e);
	return occE;
}