#include "Topo.hxx"
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Poly_Triangulation.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <BndLib_Add3dCurve.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Plane.hxx>

/////////////////VERTEX/////////////////////////////
const gp_Pnt& Vertex::GetPnt() const {
	return this->myPnt;
}

void Vertex::SetPnt(const gp_Pnt& P) {
	this->myPnt = P;
}

void Vertex::SetTol(const double T) {
	if (T > myTolerance) {
		this->myTolerance = T;
	}
}

double Vertex::GetTol() const {
	return this->myTolerance;
}

void Vertex::AddEdgeIndex(size_t edgeIndex) {
	this->myEdges.emplace_back(edgeIndex);
}

//! Set bit flag.
void Vertex::SetFlag(int theFlag, bool theIsOn) {
	if (theIsOn) myFlags |= (int)theFlag;
	else         myFlags &= ~(int)theFlag;
}
/////////////////VERTEX/////////////////////////////


/////////////////EDGE/////////////////////////////

double Edge::GetTolerance() const {
	return this->myTolerance;
}

void Edge::SetTolerance(const double T) {
	this->myTolerance = T;
}

bool Edge::SameParameter() const{
	return (myFlags & ParameterMask) != 0;
}

void Edge::SameParameter(const bool S){
	if (S) myFlags |= ParameterMask;
	else   myFlags &= ~ParameterMask;
}

bool Edge::SameRange() const
{
	return (myFlags & RangeMask) != 0;
}

void Edge::SameRange(const bool S){
	if (S) myFlags |= RangeMask;
	else   myFlags &= ~RangeMask;
}


bool Edge::Degenerated() const{
	return (myFlags & DegeneratedMask) != 0;
}

void Edge::Degenerated(const bool S){
	if (S) myFlags |= DegeneratedMask;
	else   myFlags &= ~DegeneratedMask;
}

const Handle(Geom_Curve)& Edge::GetCurve() const {
	return this->myCurve;
}

void Edge::SetCurve(Handle(Geom_Curve)& C) {
	this->myCurve = C;
}


const VertexPtr& Edge::GetFirstVertex() const {
	return this->myFirst;
}

const VertexPtr& Edge::GetLastVertex() const {
	return this->myLast; 
}

const Box& Edge::GetOrCalcBox() {
	if (myFlags >> BoxCalcFlag & 1) {
		return myBox;
	}
	else {
		Bnd_Box box;
		GeomAdaptor_Curve C(myCurve);
		BndLib_Add3dCurve::Add(C, myFirstParam, myLastParam, myTolerance, box);
		myBox.m_min[0] = box.CornerMin().X();
		myBox.m_min[1] = box.CornerMin().Y();
		myBox.m_min[2] = box.CornerMin().Z();
		myBox.m_max[0] = box.CornerMax().X();
		myBox.m_max[1] = box.CornerMax().Y();
		myBox.m_max[2] = box.CornerMax().Z();

		myFlags = myFlags | (1 << BoxCalcFlag);
		return myBox;
	}
}

/////////////////EDGE/////////////////////////////


/////////////////COEDGE/////////////////////////////
bool CoEdge::GetSameOrient() const {
	return this->mySameOrient;
}

void CoEdge::SetSameOrient(bool sameDir){
	this->mySameOrient = sameDir;
}

const EdgePtr& CoEdge::GetEdge() const {
	return this->myEdge;
}

void CoEdge::SetEdge(const EdgePtr& edge) {
	this->myEdge = edge;
}

const Handle(Geom2d_Curve)& CoEdge::GetPcurve() const {
	return this->myPcurve;
}

void CoEdge::SetPcurve(Handle(Geom2d_Curve) pCurve)  {
	this->myPcurve = pCurve;
}

const std::vector<CoedgeIndex> CoEdge::GetTwins(const ShellPtr& shell) const{
	auto& coedges = myEdge->GetCoEdges();
	std::vector<CoedgeIndex> twins;
	auto& coedgeIndexs = myEdge->GetCoEdges();
	for (int i= 0; i < coedges.size(); i++){
		auto& cofaceI = shell->GetCoFaces()[coedgeIndexs[i].myFaceIndex];
		auto& wireI = cofaceI->GetFace()->GetWires()[coedgeIndexs[i].myWireIndex];
		auto& coedgeI = wireI->GetCoEdges()[coedgeIndexs[i].myCoedgeIndex];
		if (this != coedgeI.get()) {
			twins.push_back(coedgeIndexs[i]);
		}
	}

	return twins;
}

const CoedgeIndex CoEdge::GetTwin(const ShellPtr& shell) const {
	auto& coedgeIndexs = myEdge->GetCoEdges();
	auto& coface0 = shell->GetCoFaces()[coedgeIndexs[0].myFaceIndex];
	auto& wire0 = coface0->GetFace()->GetWires()[coedgeIndexs[0].myWireIndex];
	auto& coedge0 = wire0->GetCoEdges()[coedgeIndexs[0].myCoedgeIndex];

	return coedge0.get() == this ? coedgeIndexs[0] : coedgeIndexs[1];
}

/////////////////COEDGE/////////////////////////////

///////////////////WIRE//////////////////////////////////
Wire::Wire() {}

const std::vector<CoEdgePtr>& Wire::GetCoEdges() const {
	return this->myCoEdges;
}
///////////////////WIRE//////////////////////////////////////


///////////////////FACE//////////////////////////////////////
const Handle(Geom_Surface)& Face::GetSurface() const {
	return this->mySurface;
}

double Face::CetTolerance() const {
	return this->myTolerance;
}

void  Face::SetSurface(const Handle(Geom_Surface)& S) {
	this->mySurface = S;
}

void  Face::SetTolerance(const double T) {
	this->myTolerance = T;
}

const WirePtr& Face::GetOuterWire() const {
	return this->myWires[0];
}

const std::vector<WirePtr>& Face::GetWires() const {
	return this->myWires;
}

const std::vector<CoEdge*> Face::GetCoedges() const{
	std::vector<CoEdge*> coedgeRet;
	for (const auto& wire : myWires) {
		const auto& coedges = wire->GetCoEdges();
		for (const auto& coedge: coedges){
			coedgeRet.push_back(coedge.get());
		}
	}
	return std::move(coedgeRet);
}

const Box& Face::GetOrCalcBox() {

	if (myFlags >> BoxCalcFlag & 1) {
		return myBox;
	}
	else {
		if (!Handle(Geom_Plane)::DownCast(mySurface).IsNull()) {
			const std::vector<CoEdgePtr>& outerWire = myWires[0]->GetCoEdges();
			for (size_t i = 0; i < outerWire.size(); i++) {
				myBox.Add(outerWire[i]->GetEdge()->GetOrCalcBox());
			}

			myFlags = myFlags | (1 << BoxCalcFlag);//����Ϊ�����
			return myBox;
		}
		else {
			//TODO : ��չ��������
			throw;
		}
	}
}

///////////////////FACE//////////////////////////////////////


///////////////////COFACE//////////////////////////////////////
const FacePtr& CoFace::GetFace() const {
	return myFace; 
}

bool CoFace::GetSameOrient() const { 
	return mySameOrient; 
}


///////////////////COFACE//////////////////////////////////////


///////////////////SHELL//////////////////////////////////////
Standard_EXPORT const std::vector<CoFacePtr>& Shell::GetCoFaces() const {
	return myCoFaces; 
}
std::vector<CoFacePtr>& Shell::GetCoFacesWithoutConst() {
	return myCoFaces;
}

ShellPtr Shell::Reversed() {
	ShellPtr revShell = std::make_shared<Shell>();
	revShell->ReserveCoFaceSize(myCoFaces.size());
	for (int i = 0; i < myCoFaces.size(); i ++) {
		revShell->AddCoface(std::make_shared<CoFace>(myCoFaces[i]->GetFace(), !myCoFaces[i]->GetSameOrient()));
	}
	return revShell;
}

Standard_EXPORT void Shell::BuildBackwardReference() const {
	size_t faceSize = myCoFaces.size();
	myEdges.clear();
	myEdges.reserve(faceSize * 3);
	myVertices.clear();
	myVertices.reserve(faceSize * 2);
	for (size_t iFace = 0; iFace < faceSize; iFace++)
	{
		const FacePtr& face = myCoFaces[iFace]->GetFace();
		const std::vector<WirePtr>& wires = face->GetWires();
		size_t wireSize = wires.size();
		for (size_t iWire = 0; iWire < wireSize; iWire++)
		{
			const WirePtr& wire = wires[iWire];
			const std::vector<CoEdgePtr>& coedges = wire->GetCoEdges();
			size_t coedgeSize = coedges.size();
			for (size_t iCoedge = 0; iCoedge < coedgeSize; iCoedge++)
			{
				const CoEdgePtr& coedge = coedges[iCoedge];
				const EdgePtr& edge = coedge->GetEdge();
				edge->GetCoEdges().clear();
				edge->GetCoEdges().reserve(4);
				const VertexPtr& firstV = edge->GetFirstVertex();
				firstV->GetEdgeIndexs().clear();
				firstV->GetEdgeIndexs().reserve(3);
				const VertexPtr& lastV = edge->GetLastVertex();
				lastV->GetEdgeIndexs().clear();
				lastV->GetEdgeIndexs().reserve(3);
			}
		}
	}

	for (size_t iFace = 0; iFace < faceSize; iFace++) 
	{
		auto& coface = myCoFaces[iFace];
		//coface->SetIndex(iFace);
		const FacePtr& face = coface->GetFace();
		const std::vector<WirePtr>& wires = face->GetWires();
		size_t wireSize = wires.size();
		for (size_t iWire = 0; iWire < wireSize; iWire++)
		{
			const WirePtr& wire = wires[iWire];
			const std::vector<CoEdgePtr>& coedges = wire->GetCoEdges();
			size_t coedgeSize = coedges.size();
			for (size_t iCoedge = 0; iCoedge < coedgeSize; iCoedge++)
			{
				const CoEdgePtr& coedge = coedges[iCoedge];
				const EdgePtr& edge = coedge->GetEdge();

				if (!edge->IsVisited()) {
					myEdges.emplace_back(edge);
					edge->SetVisited();
				}

				CoedgeIndex coedgeIndex(iFace, iWire, iCoedge);
				edge->AddCoedgeIndex(coedgeIndex);
			}
		}
	}

	for (size_t iEdge = 0; iEdge < myEdges.size(); iEdge++){
		const VertexPtr& firstV = myEdges[iEdge]->GetFirstVertex();
		firstV->AddEdgeIndex(iEdge);

		if (!firstV->IsVisited()) {
			myVertices.emplace_back(firstV);
			firstV->SetVisited();
		}

		const VertexPtr& lastV = myEdges[iEdge]->GetLastVertex();
		lastV->AddEdgeIndex(iEdge);

		if (!lastV->IsVisited()) {
			myVertices.emplace_back(lastV);
			lastV->SetVisited();
		}
		myEdges[iEdge]->CleanVisited();
	}

	for (size_t iV = 0; iV < myVertices.size(); iV++) {
		myVertices[iV]->CleanVisited();
	}
}
///////////////////SHELL//////////////////////////////////////
