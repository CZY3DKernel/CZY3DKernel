#include "Boolean.hxx"
#include "Rtree.hxx"
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>
#include <BndLib_Add3dCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Bnd_Tools.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Line.hxx>
#include <Geom2d_Line.hxx>
#include <ElSLib.hxx>
#include "Topo.hxx"
#include <algorithm>
#include <list>
#include <unordered_set>
#include <ElSLib.hxx>
#include <set>
#include <numeric> 
#include <cstdlib> 
#include <stack>
#include "Quadg.hxx"
#include <cmath>
#include "BrepTranslator.hxx"
#include <BRep_Builder.hxx>
#include <TopoDS_Edge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <memory>

////#include "vld.h"
using namespace std;
//

const vector<VertexPtr>& BoolFaceInfo::AllVertices() {
	if (myAllVertices.empty()) {
		myAllVertices.insert(myAllVertices.end(), myVerticesOnEdge.begin(), myVerticesOnEdge.end());
		myAllVertices.insert(myAllVertices.end(), myVerticesOnFace.begin(), myVerticesOnFace.end());
		myAllVertices.insert(myAllVertices.end(), myVerticesSection.begin(), myVerticesSection.end());
		std::sort(myAllVertices.begin(), myAllVertices.end());
	}
	return myAllVertices;
}

template <typename T> static int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

static double AminusBdotC(const gp_Pnt& A, const gp_Pnt& B, const gp_Vec& C) {
	return (A.X() - B.X()) * C.X() + (A.Y() - B.Y()) * C.Y() + (A.Z() - B.Z()) * C.Z();
}

static double AminusBdotC(const gp_Pnt2d& A, const gp_Pnt2d& B, const gp_Vec2d& C) {
	return (A.X() - B.X()) * C.X() + (A.Y() - B.Y()) * C.Y();
}

static double LineDistanceToPoint(const Handle(Geom_Line)& aLine, const gp_Pnt& pt) {
	const gp_Dir& dir = aLine->Lin().Direction();
	gp_Pnt OR = aLine->Lin().Location();
	gp_Vec V(OR, pt);
	Standard_Real dist = V.Crossed(dir).Magnitude();
	return dist;
}


static Handle(Geom2d_Line) Line3dToLine2d(const Handle(Geom_Line)& aLine, const Handle(Geom_Plane)& aPlane) {
	double u, v;
	ElSLib::PlaneParameters(aPlane->Pln().Position(), aLine->Lin().Location(), u, v);
	double dx = aPlane->Pln().XAxis().Direction().Dot(aLine->Lin().Direction());
	double dy = aPlane->Pln().YAxis().Direction().Dot(aLine->Lin().Direction());
	Handle(Geom2d_Line) line2d = new Geom2d_Line(gp_Pnt2d(u,v), gp_Dir2d(dx, dy));
	return line2d;
}

static double DistanceWitgFlag(const Handle(Geom_Plane)& plane, const gp_Pnt& theP)
{
	const gp_Pnt& aLoc = plane->Pln().Position().Location();
	const gp_Dir& aDir = plane->Pln().Position().Direction();
	Standard_Real aD = (aDir.X() * (theP.X() - aLoc.X()) +
		aDir.Y() * (theP.Y() - aLoc.Y()) +
		aDir.Z() * (theP.Z() - aLoc.Z()));
	return aD;
}

//
static void GetVertices2ds(const WirePtr& wire, vector<gp_Pnt2d>& vertices) {
	const auto& coedges = wire->GetCoEdges();
	for (const auto& coedge : coedges) {
		gp_Pnt2d P;
		double param = coedge->GetFirstParamWithOrient();
		coedge->GetPcurve()->D0(param, P);
		vertices.push_back(P);
	}
}


void static SetIntersectionCoedge(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	edge->SetFlag(EdgeFlag::IsIntersectionEdgeFlag);
}

bool static IsIntersectionCoedge(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	return edge->GetFlag(EdgeFlag::IsIntersectionEdgeFlag);
}


void static SetCoedgeUnValid(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	edge->SetFlag(EdgeFlag::IsUnValidEdgeFlag);
}

bool static IsCoedgeUnValid(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	return edge->GetFlag(EdgeFlag::IsUnValidEdgeFlag);
}

void static SetCoedgeValid(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	edge->SetFlag(EdgeFlag::IsValidEdgeFlag);
}

bool IsCoedgeValid(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	return edge->GetFlag(EdgeFlag::IsValidEdgeFlag);
}

bool IsUnKnownValidCoedge(const CoEdgePtr& coedge) {
	return !IsCoedgeValid(coedge) && !IsCoedgeUnValid(coedge);
}

void CleanCoEdgeFlags(const CoEdgePtr& coedge) {
	EdgePtr edge = coedge->GetEdge();
	edge->CleanFlag(EdgeFlag::IsIntersectionEdgeFlag);
	edge->CleanFlag(EdgeFlag::IsUnValidEdgeFlag);
	edge->CleanFlag(EdgeFlag::IsValidEdgeFlag);
}

//
unique_ptr<BoolFaceInfo>& Boolean::GetBooleanFaceInfo(const FacePtr& face) {
	std::unique_ptr<BoolFaceInfo>& faceBoolInfoRef = face2InfoMap[face];
	if (!faceBoolInfoRef) {
		faceBoolInfoRef = std::unique_ptr<BoolFaceInfo>(new BoolFaceInfo());
	}
	return faceBoolInfoRef;
}
//
unique_ptr<OriginEdgeBoolInfo>& Boolean::GetBooleanEdgeInfo(const EdgePtr& edge) {
	unique_ptr<OriginEdgeBoolInfo>& boolEdgeInfo = edge2InfoMap[edge];
	if (!boolEdgeInfo) {
		boolEdgeInfo = std::unique_ptr<OriginEdgeBoolInfo>(new OriginEdgeBoolInfo());
		auto first = std::unique_ptr<Pave>(new Pave(edge->GetFirstVertex(), edge->GetFirstParam()));
		auto last = std::unique_ptr<Pave>(new Pave(edge->GetLastVertex(), edge->GetLastParam()));
		boolEdgeInfo->myPaveBlocks.push_back(std::unique_ptr<PaveBlock>(new PaveBlock(first.get(), last.get())));
		boolEdgeInfo->paves.push_back(std::move(first));
		boolEdgeInfo->paves.push_back(std::move(last));
	}
	return boolEdgeInfo;
}
//
bool Boolean::SplitPaveBlocks(unique_ptr<OriginEdgeBoolInfo>& boolEdgeInfo) {
	vector<unique_ptr<PaveBlock>> newPaveBlocks;
	newPaveBlocks.reserve(boolEdgeInfo->myPaveBlocks.size() * 2);
	vector<unique_ptr<PaveBlock>>& paveBlocks = boolEdgeInfo->myPaveBlocks;
	for (int i = 0; i < paveBlocks.size(); i++) {
		unique_ptr<PaveBlock>& paveBlock = paveBlocks[i];
		if (!paveBlock->mySplitPaves.empty()) {
			sort(paveBlock->mySplitPaves.begin(), paveBlock->mySplitPaves.end(), 
				[](Pave* a, Pave* b) { return a->param < b->param; });
			for (int j = 0; j < paveBlock->mySplitPaves.size(); j++) {
				if (j == 0) {
					newPaveBlocks.push_back(std::unique_ptr<PaveBlock>(new PaveBlock(paveBlock->myFirst, paveBlock->mySplitPaves[j])));
				}else {
					newPaveBlocks.push_back(std::unique_ptr<PaveBlock>(new PaveBlock(paveBlock->mySplitPaves[j - 1], paveBlock->mySplitPaves[j])));
				}

				if (paveBlock->myCommonBlockIndex != -1) {
					return false;
				}
			}
			newPaveBlocks.push_back(std::unique_ptr<PaveBlock>(new PaveBlock(paveBlock->mySplitPaves.back(), paveBlock->myLast)));
		}
		else {
			newPaveBlocks.push_back(std::move(paveBlock));
		}	
	}
	boolEdgeInfo->myPaveBlocks.clear();
	std::vector<unique_ptr<PaveBlock>>().swap(boolEdgeInfo->myPaveBlocks);
	boolEdgeInfo->myPaveBlocks = std::move(newPaveBlocks);

	return true;
}
//
Box Boolean::MakeBoxForVertex(const VertexPtr& V) {
	Box aBox;
	const gp_Pnt& pt = V->GetPnt();
	const double tol = V->GetTol();
	aBox.m_min[0] = pt.X() - tol;
	aBox.m_min[1] = pt.Y() - tol;
	aBox.m_min[2] = pt.Z() - tol;

	aBox.m_max[0] = pt.X() + tol;
	aBox.m_max[1] = pt.Y() + tol;
	aBox.m_max[2] = pt.Z() + tol;

	return aBox;
}
//
void Boolean::GetIntersectionPairs(){
	//ShellA RTree
	std::unique_ptr<ON_RTree> aBoxTreeA = std::unique_ptr<ON_RTree>(new ON_RTree());
	int indexIntA = 0;
	myShells[0]->BuildBackwardReference();
	auto& cofacesA = myShells[0]->GetCoFaces();
	for (size_t iFace = 0; iFace < cofacesA.size(); iFace++) {
		const Box& aBox = cofacesA[iFace]->GetFace()->GetOrCalcBox();
		aBoxTreeA->Insert(aBox.m_min, aBox.m_max, indexIntA++);
	}

	auto& edgesA = myShells[0]->GetEdges();
	for (size_t iEdge = 0; iEdge < edgesA.size(); iEdge++) {
		const Box& aBox = edgesA[iEdge]->GetOrCalcBox();
		aBoxTreeA->Insert(aBox.m_min, aBox.m_max, indexIntA++);
	}

	auto& verticesA = myShells[0]->GetVertices();
	for (size_t iVertex = 0; iVertex < verticesA.size(); iVertex++) {
		const Box& aBox = MakeBoxForVertex(verticesA[iVertex]);
		aBoxTreeA->Insert(aBox.m_min, aBox.m_max, indexIntA++);
	}

	//ShellB RTree
	std::unique_ptr<ON_RTree> aBoxTreeB = std::unique_ptr<ON_RTree>(new ON_RTree());
	int indexIntB = 0;
	myShells[1]->BuildBackwardReference();
	auto& cofacesB = myShells[1]->GetCoFaces();
	for (size_t iFace = 0; iFace < cofacesB.size(); iFace++) {
		const Box& aBox = cofacesB[iFace]->GetFace()->GetOrCalcBox();
		aBoxTreeB->Insert(aBox.m_min, aBox.m_max, indexIntB++);
	}

	auto& edgesB = myShells[1]->GetEdges();
	for (size_t iEdge = 0; iEdge < edgesB.size(); iEdge++) {
		const Box& aBox = edgesB[iEdge]->GetOrCalcBox();
		aBoxTreeB->Insert(aBox.m_min, aBox.m_max, indexIntB++);
	}

	auto& verticesB = myShells[1]->GetVertices();
	for (size_t iVertex = 0; iVertex < verticesB.size(); iVertex++) {
		const Box& aBox = MakeBoxForVertex(verticesB[iVertex]);
		aBoxTreeB->Insert(aBox.m_min, aBox.m_max, indexIntB++);
	}

	//GetIntersectionPair
	int totalfaceSizeA = myShells[0]->GetCoFaces().size();
	int totalFaceEdgeSizeA = myShells[0]->GetEdges().size() + totalfaceSizeA;
	int totalfaceSizeB = myShells[1]->GetCoFaces().size();
	int totalFaceEdgeSizeB = myShells[1]->GetEdges().size() + totalfaceSizeB;

	std::vector<ON_2dex> a_result;
	ON_RTree::Search(*aBoxTreeA, *aBoxTreeB, myFuzzyValue, a_result);
	for (auto pair : a_result) {
		std::unique_ptr<IntersectionPair> intersectionPair = std::unique_ptr<IntersectionPair>(new IntersectionPair());
		intersectionPair->first[0] = 0;
		if (pair.i >= totalFaceEdgeSizeA) {
			intersectionPair->first[1] = 0;
			intersectionPair->first[2] = pair.i - totalFaceEdgeSizeA;
		}
		else if (pair.i >= totalfaceSizeA) {
			intersectionPair->first[1] = 1;
			intersectionPair->first[2] = pair.i - totalfaceSizeA;
		}
		else {
			intersectionPair->first[1] = 2;
			intersectionPair->first[2] = pair.i;
		}

		//////////intersectionPairB///////////////
		intersectionPair->last[0] = 1;
		if (pair.j >= totalFaceEdgeSizeB) {
			intersectionPair->last[1] = 0;
			intersectionPair->last[2] = pair.j - totalFaceEdgeSizeB;
		}
		else if (pair.j >= totalfaceSizeB) {
			intersectionPair->last[1] = 1;
			intersectionPair->last[2] = pair.j - totalfaceSizeB;
		}
		else {
			intersectionPair->last[1] = 2;
			intersectionPair->last[2] = pair.j;
		}
		int index = intersectionPair->first[1] + intersectionPair->last[1];
		if (intersectionPair->first[1] == 2 || intersectionPair->last[1] == 2) {
			index++;
		}

		//VV VE EE VF EF FF
		myTopoIntpair[index].push_back(std::move(intersectionPair));
	}
};

VertexPtr Boolean::makeVertexFromVV(const VertexPtr& va, const VertexPtr& vb) {
	VertexPtr newVertex;
	int m, n;
	double aR[2], dR, aD, aEps;
	gp_Pnt aP[2];
	//
	aEps = RealEpsilon();
	aP[0] = va->GetPnt();
	aR[0] = va->GetTol();
	aP[1] = vb->GetPnt();
	aR[1] = vb->GetTol();
	m = 0; // max R
	n = 1; // min R
	if (aR[0] < aR[1]) {
		m = 1;
		n = 0;
	}
	//
	dR = aR[m] - aR[n]; // dR >= 0.
	gp_Vec aVD(aP[m], aP[n]);
	aD = aVD.Magnitude();
	//
	if (aD <= dR || aD < aEps) {
		newVertex = make_shared<Vertex>(aP[m], aR[m]);
	}
	else {
		double aRr;
		gp_XYZ aXYZr;
		gp_Pnt aPr;
		//
		aRr = 0.5*(aR[m] + aR[n] + aD);
		aXYZr = 0.5*(aP[m].XYZ() + aP[n].XYZ() - aVD.XYZ()*(dR / aD));
		aPr.SetXYZ(aXYZr);
		newVertex = make_shared<Vertex>(aPr, aRr);
	}
	return newVertex;
}

void Boolean::PerformVV() {
	const vector<unique_ptr<IntersectionPair>>& vv = myTopoIntpair[0];
	for (auto& pair : vv){
		const VertexPtr& vf = myShells[0]->GetVertices()[pair->first[2]];
		const VertexPtr& vl = myShells[1]->GetVertices()[pair->last[2]];
		if (vf->GetPnt().SquareDistance(vl->GetPnt()) <= myFuzzyValue * myFuzzyValue) {

			const VertexPtr& newVertex = makeVertexFromVV(vf, vl);
			vertex2InfoMap[vf] = newVertex;
			vertex2InfoMap[vl] = newVertex;

			//shellA: newVertex put to paves, oldV replace with newVertex
			for (auto index : vf->GetEdgeIndexs()) {
				EdgePtr e = myShells[0]->GetEdges()[index];
				unique_ptr<OriginEdgeBoolInfo>& booleanE = GetBooleanEdgeInfo(e);
				if (booleanE->paves.front()->vertex == vf) {
					booleanE->paves.front()->vertex = newVertex;
				}else {
					booleanE->paves.back()->vertex = newVertex;
				}
			}
			//shellB: newVertex put to paves, oldV replace with newVertex
			for (auto index : vl->GetEdgeIndexs()) {
				EdgePtr e = myShells[1]->GetEdges()[index];
				unique_ptr<OriginEdgeBoolInfo>& booleanE = GetBooleanEdgeInfo(e);
				if (booleanE->paves.front()->vertex == vl) {
					booleanE->paves.front()->vertex = newVertex;
				}
				else {
					booleanE->paves.back()->vertex = newVertex;
				}
			}

			//shellA: put v to adjecent faces
			vector<size_t>& firstEdgeIndex = vf->GetEdgeIndexs();
			auto& firstEdges = myShells[0]->GetEdges();
			for (auto val : firstEdgeIndex) {
				auto& edge = firstEdges[val];
				const vector<CoedgeIndex>& coEdges = edge->GetCoEdges();
				auto& faces = myShells[0]->GetCoFaces();
				for (const auto& coEdge : coEdges){
					auto& face = faces[coEdge.myFaceIndex]->GetFace();
					auto& boolFace = GetBooleanFaceInfo(face);
					if (boolFace->myVerticesOnEdge.empty() || 
						boolFace->myVerticesOnEdge.back() != newVertex) {
						boolFace->myVerticesOnEdge.push_back(newVertex);
					}
				}
			}
			
			//shellB : put v to adjecent faces
			vector<size_t>& lastEdgeIndex = vl->GetEdgeIndexs();
			auto& lastEdges = myShells[1]->GetEdges();
			for (auto val : lastEdgeIndex) {
				auto& edge = lastEdges[val];
				const vector<CoedgeIndex>& coEdges = edge->GetCoEdges();
				auto& faces = myShells[1]->GetCoFaces();
				for (auto coEdge : coEdges) {
					auto& face = faces[coEdge.myFaceIndex]->GetFace();
					auto& boolFace = GetBooleanFaceInfo(face);
					if (boolFace->myVerticesOnEdge.empty() || 
						boolFace->myVerticesOnEdge.back() != newVertex) {
						boolFace->myVerticesOnEdge.push_back(newVertex);
					}
				}
			}
		}
	}
};
//
bool Boolean::PerformVE() {
	for (auto& VEPair : myTopoIntpair[1]){
		// get V and E
		size_t* firstTopo = VEPair->first;
		size_t* lastTopo = VEPair->last;
		VertexPtr v;
		EdgePtr e;
		int vShellIndex = -1;
		int eShellIndex = -1;

		if (firstTopo[1] == 0) {//VE
			 v = myShells[0]->GetVertices()[firstTopo[2]];
			 e = myShells[1]->GetEdges()[lastTopo[2]];
			 vShellIndex = firstTopo[0];
			 eShellIndex = lastTopo[0];
		}
		else {//EV
			 v = myShells[1]->GetVertices()[lastTopo[2]];
			 e = myShells[0]->GetEdges()[firstTopo[2]];
			 vShellIndex = lastTopo[0];
			 eShellIndex = firstTopo[0];
		}
		// end get V and E

		Handle(Geom_Line) aLine = Handle(Geom_Line)::DownCast(e->GetCurve());
		if (!aLine.IsNull()) {
			//V��E�Ķ˵��н���VV�Ѿ�����
			auto it = vertex2InfoMap.find(v);

			auto it2 = vertex2InfoMap.find(e->GetFirstVertex());
			if (it != vertex2InfoMap.end() && it2 != vertex2InfoMap.end() &&
				it2->second == it->second) {
				continue;
			}
			auto it3 = vertex2InfoMap.find(e->GetLastVertex());
			if (it != vertex2InfoMap.end() && it3 != vertex2InfoMap.end() &&
				it3->second == it->second) {
				continue;
			}
			const gp_Dir& dir = aLine->Lin().Direction();
			gp_Pnt OR = aLine->Lin().Location();
			gp_Vec V(OR, v->GetPnt());
			Standard_Real dist = V.Dot(dir);
			if ((dist >= e->GetFirstParam()) && (dist <= e->GetLastParam())){
				gp_Pnt MyP = OR.Translated(dist*dir);
				double ptDist = MyP.Distance(v->GetPnt());
				if (ptDist <= myFuzzyValue) {
					VertexPtr newVertex = make_shared<Vertex>();
					newVertex->SetPnt(v->GetPnt());
					newVertex->SetTol(ptDist);

					vertex2InfoMap[v] = newVertex;
					const std::vector<EdgePtr>& edges = myShells[vShellIndex]->GetEdges();
					for (auto index : v->GetEdgeIndexs()) {
						unique_ptr<OriginEdgeBoolInfo>& originEdgeBoolInfo = GetBooleanEdgeInfo(edges[index]);
						for (auto& pave : originEdgeBoolInfo->paves) {
							if (pave->vertex == v) {
								pave->vertex = newVertex;
							}
						}
					}

					auto& originEdgeBoolInfo = GetBooleanEdgeInfo(e);
					originEdgeBoolInfo->paves.push_back(std::unique_ptr<Pave>(new Pave(newVertex, dist)));
					originEdgeBoolInfo->myPaveBlocks[0]->mySplitPaves.push_back(originEdgeBoolInfo->paves.back().get());

					// put v to adjecent faces
					const vector<CoedgeIndex>& coedges = e->GetCoEdges();
					auto& coFaces = myShells[eShellIndex]->GetCoFaces();
					for (size_t j = 0; j < coedges.size(); j++){
						const FacePtr& face = coFaces[coedges[j].myFaceIndex]->GetFace();
						GetBooleanFaceInfo(face)->myVerticesOnEdge.push_back(newVertex);
					}

					// put v to adjecent faces
					vector<size_t>& firstEdgeIndex = v->GetEdgeIndexs();
					auto& firstEdges = myShells[vShellIndex]->GetEdges();
					for (auto val : firstEdgeIndex) {
						const EdgePtr& edge = firstEdges[val];
						const vector<CoedgeIndex>& coEdges = edge->GetCoEdges();
						auto& faces = myShells[vShellIndex]->GetCoFaces();
						for (auto coEdge : coEdges) {
							const FacePtr& face = faces[coEdge.myFaceIndex]->GetFace();
							auto& boolFace = GetBooleanFaceInfo(face);
							if (boolFace->myVerticesOnEdge.empty() ||
								boolFace->myVerticesOnEdge.back() != newVertex) {
								boolFace->myVerticesOnEdge.push_back(newVertex);
							}
						}
					}
				}
			}
		}else {
		    //TODO
		}
	}

	//Split Pave Block
	for (auto& it : edge2InfoMap) {
		if (!SplitPaveBlocks(it.second)) {
			return false;
		}
	}

	return true;
}
//
VertexPtr Boolean::ComputeLineLine(const Handle(Geom_Line)& aLineA, const Handle(Geom_Line)& aLineB,
	const EdgePtr& eA, const EdgePtr& eB) {

	VertexPtr newVertex;
	Standard_Real aTol = myFuzzyValue * myFuzzyValue;
	const gp_Lin& aL1 = aLineA->Lin();
	const gp_Lin& aL2 = aLineB->Lin();
	const gp_Dir& aD1 = aL1.Direction();
	const gp_Dir& aD2 = aL2.Direction();

	Standard_Real anAngle = aD1.Angle(aD2);
	Standard_Boolean IsCoincide = anAngle < Precision::Angular();
	if (IsCoincide) {
		if (aL1.SquareDistance(aL2.Location()) > aTol)
			return newVertex;
	}
	auto& OriginEdgeBoolInfoA = GetBooleanEdgeInfo(eA);
	auto& OriginEdgeBoolInfoB = GetBooleanEdgeInfo(eB);
	vector<unique_ptr<PaveBlock>>& paveBlocksA = OriginEdgeBoolInfoA->myPaveBlocks;
	vector<unique_ptr<PaveBlock>>& paveBlocksB = OriginEdgeBoolInfoB->myPaveBlocks;

	const gp_XYZ& l1Dir = aLineA->Lin().Direction().XYZ();
	const gp_XYZ& l2Dir = aLineB->Lin().Direction().XYZ();
	const gp_XYZ& crossVec = l1Dir.Crossed(l2Dir);
	const gp_XYZ& diff = aLineB->Lin().Location().XYZ().Subtracted(aLineA->Lin().Location().XYZ());
	double OneOverSinAngle2 = 1.0 / crossVec.SquareModulus();

	double t1 = diff.DotCross(l2Dir, crossVec) * OneOverSinAngle2;
	double t2 = diff.DotCross(l1Dir, crossVec) * OneOverSinAngle2;

	if (t1 > eA->GetFirstParam() && t1 < eA->GetLastParam() &&
		t2 > eB->GetFirstParam() && t2 < eB->GetLastParam()) {
		gp_Pnt pt, pt2;
		aLineA->D0(t1, pt);
		aLineB->D0(t2, pt2);
		if (pt.SquareDistance(pt2) < myFuzzyValue*myFuzzyValue) {
			list<unique_ptr<Pave>>& paves = GetBooleanEdgeInfo(eA)->paves;
			for (auto& pave : paves) {
				if (pave->vertex->GetPnt().SquareDistance(pt) < myFuzzyValue*myFuzzyValue) {
					return newVertex;
				}
			}
			newVertex = make_shared<Vertex>(pt, distTol);
			OriginEdgeBoolInfoA->paves.push_back(std::unique_ptr<Pave>(new Pave(newVertex, t1)));
			OriginEdgeBoolInfoB->paves.push_back(std::unique_ptr<Pave>(new Pave(newVertex, t2)));
			for (int i = 0; i < paveBlocksA.size(); i++) {
				if (t1 > paveBlocksA[i]->myFirst->param && t1 < paveBlocksA[i]->myLast->param) {
					paveBlocksA[i]->mySplitPaves.push_back(OriginEdgeBoolInfoA->paves.back().get());
					break;
				}
			}
			for (int i = 0; i < paveBlocksB.size(); i++) {
				if (t2 > paveBlocksB[i]->myFirst->param && t2 < paveBlocksB[i]->myLast->param) {
					paveBlocksB[i]->mySplitPaves.push_back(OriginEdgeBoolInfoB->paves.back().get());
					break;
				}
			}
		}
	}

	return newVertex;
}
//
void Boolean::PerformEE() {
	const auto& ee = myTopoIntpair[2];
	for (int i = 0; i < ee.size(); i++){
		size_t* firstTopo = ee[i]->first;
		size_t* lastTopo = ee[i]->last;
		auto eA = myShells[0]->GetEdges()[firstTopo[2]];
		auto eB = myShells[1]->GetEdges()[lastTopo[2]];
		Handle(Geom_Line) aLineA = Handle(Geom_Line)::DownCast(eA->GetCurve());
		Handle(Geom_Line) aLineB = Handle(Geom_Line)::DownCast(eB->GetCurve());
		if (!aLineA.IsNull() && !aLineB.IsNull()) {
			auto newVertex = ComputeLineLine(aLineA, aLineB, eA, eB);
			if (newVertex) {
				//put pave to adjecent faces
				const vector<CoedgeIndex>& coEdges = eA->GetCoEdges();
				const auto& faces = myShells[0]->GetCoFaces();
				for (auto coEdge : coEdges) {
					auto& face = faces[coEdge.myFaceIndex]->GetFace();
					auto& boolFace = GetBooleanFaceInfo(face);
					boolFace->myVerticesOnEdge.push_back(newVertex);
				}
				const vector<CoedgeIndex>& coEdgesB = eB->GetCoEdges();
				const auto& facesB = myShells[1]->GetCoFaces();
				for (auto coEdge : coEdgesB) {
					const auto& faceB = facesB[coEdge.myFaceIndex]->GetFace();
					auto& boolFaceB = GetBooleanFaceInfo(faceB);
					boolFaceB->myVerticesOnEdge.push_back(newVertex);
				}
			}
		}
		else {
		    //TODO:
		}
	}

	//Split Pave Block
	for (auto& it : edge2InfoMap) {
		SplitPaveBlocks(it.second);
	}

	//same domain pave block
	for (auto& ePair : ee){
		size_t* firstTopo = ePair->first;
		size_t* lastTopo = ePair->last;
		auto eA = myShells[0]->GetEdges()[firstTopo[2]];
		auto eB = myShells[1]->GetEdges()[lastTopo[2]];
		auto& OriginEdgeBoolInfoA = GetBooleanEdgeInfo(eA);
		auto& OriginEdgeBoolInfoB = GetBooleanEdgeInfo(eB);
		vector<unique_ptr<PaveBlock>>& paveBlocksA = OriginEdgeBoolInfoA->myPaveBlocks;
		vector<unique_ptr<PaveBlock>>& paveBlocksB = OriginEdgeBoolInfoB->myPaveBlocks;
		for (int i = 0; i < paveBlocksA.size(); i++) {
			auto& paveBlockA = paveBlocksA[i];
			for (int j = 0; j < paveBlocksB.size(); j++) {
				auto& paveBlockB = paveBlocksB[j];
				bool sameDirCoincidence = paveBlockA->myFirst->vertex == paveBlockB->myFirst->vertex &&
					paveBlockA->myLast->vertex == paveBlockB->myLast->vertex;
				bool reverseDirCoincidence = paveBlockA->myFirst->vertex == paveBlockB->myLast->vertex &&
					paveBlockA->myLast->vertex == paveBlockB->myFirst->vertex;
				if (sameDirCoincidence || reverseDirCoincidence) {
					vector<PaveBlock*> commonBlock;
					commonBlock.push_back(paveBlockA.get());
					commonBlock.push_back(paveBlockB.get());

					paveBlockA->myCommonBlockIndex = myCommonBlocks.size();
					paveBlockB->myCommonBlockIndex = myCommonBlocks.size();
					myCommonBlocks.push_back(commonBlock);
				}
			}
		}
	}
}


// withoutOn
bool Boolean::IsPointInPolygonWithoutOn(double x, double y, vector<gp_Pnt2d>& vertices) {
	// Translate the vertices
	int size = vertices.size();

	double xMin = vertices[0].X();
	double xMax = vertices[0].X();
	double yMin = vertices[0].Y();
	double yMax = vertices[0].Y();

	for (int i = 0; i < size; i++) {
		if (vertices[i].X() < xMin) {
			xMin = vertices[i].X();
		}
		if (vertices[i].X() > xMax) {
			xMax = vertices[i].X();
		}
		if (vertices[i].Y() < yMin) {
			yMin = vertices[i].Y();
		}
		if (vertices[i].Y() > yMax) {
			yMax = vertices[i].Y();
		}

		vertices[i].SetX(vertices[i].X() - x);
		vertices[i].SetY(vertices[i].Y() - y);
	}

	if (x < xMin || x > xMax || y < yMin || y > yMax) {
		return false;
	}

	// Compute quadrant number for the vertices
	std::vector<int> quad(size);
	for (int i = 0; i < size; i++) {
		bool posX = vertices[i].X() > 0;
		bool posY = vertices[i].Y() > 0;
		bool negX = !posX;
		bool negY = !posY;
		quad[i] = (negX & posY) + 2 * (negX & negY) + 3 * (posX & negY);
	}

	// Compute the vertex quadrant changes for each test point.
	std::vector<int> diffQuad;
	std::transform(quad.begin() + 1, quad.end(), quad.begin(), std::back_inserter(diffQuad),
		[](int a, int b) { return a - b; });
	diffQuad.push_back(quad.front() - quad.back());

	for (int i = 0; i < size; i++) {
		if (abs(diffQuad[i]) == 3) {
			diffQuad[i] = - sgn(diffQuad[i]);
		}
		else if (abs(diffQuad[i]) == 2) {
			diffQuad[i] = 2 * sgn(vertices[i].X() * vertices[(i + 1) % size].Y()
				- vertices[(i + 1) % size].X() * vertices[i].Y());
		}
	}

	int sum = std::accumulate(diffQuad.begin(), diffQuad.end(), 0);
	return sum != 0;
}

//
bool Boolean::PointInPolyhedronFaceWithoutOn(const gp_Pnt& pt, const FacePtr& face) {
	double U, V;
	Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(face->GetSurface());
	ElSLib::Parameters(aPlane->Pln(), pt, U, V);
	gp_Pnt2d pt2d(U, V);
	const auto& wires = face->GetWires();
	for (int i = 0; i < wires.size(); i++) {
		const auto& wire = wires[i];
		vector<gp_Pnt2d> polygon;
		const auto& coedges = wire->GetCoEdges();
		for (int j = 0; j < coedges.size(); j++) {
			double param = coedges[j]->GetFirstParamWithOrient();
			gp_Pnt2d start2d;
			coedges[j]->GetPcurve()->D0(param, start2d);
			polygon.push_back(start2d);
		}
		bool isInPolygon = IsPointInPolygonWithoutOn(pt2d.X(), pt2d.Y(), polygon);
		if (i == 0 && !isInPolygon) {
			return false;
		}
		else if( i != 0 && isInPolygon) {
			return false;
		}
	}
	return true;
}
//
void Boolean::PerformVF() {
	for (int i = 0; i < myTopoIntpair[3].size(); i++) {
		auto& VFPair = myTopoIntpair[3][i];
		size_t* shellATopo = VFPair->first;
		size_t* shellBTopo = VFPair->last;
		VertexPtr v;
		FacePtr f;
		size_t vShellIndex = -1;
		size_t fShellIndex = -1;

		if (shellATopo[1] == 0) {//VF
			v = myShells[shellATopo[0]]->GetVertices()[shellATopo[2]];
			f = myShells[shellBTopo[0]]->GetCoFaces()[shellBTopo[2]]->GetFace();
			vShellIndex = shellATopo[0];
			fShellIndex = shellBTopo[0];
		}
		else {//FV
			v = myShells[shellBTopo[0]]->GetVertices()[shellBTopo[2]];
			f = myShells[shellATopo[0]]->GetCoFaces()[shellATopo[2]]->GetFace();
			vShellIndex = shellBTopo[0];
			fShellIndex = shellATopo[0];
		}

		auto it = vertex2InfoMap.find(v);
		if (it != vertex2InfoMap.end()) {
			auto& boolFaceInfo = GetBooleanFaceInfo(f);
			if (std::find(boolFaceInfo->myVerticesOnEdge.begin(),
				boolFaceInfo->myVerticesOnEdge.end(), it->second) !=
				boolFaceInfo->myVerticesOnEdge.end()) {
				continue;
			}
		}

		if (myIsPloyHendral) {
			Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(f->GetSurface());
			double dist = aPlane->Pln().Distance(v->GetPnt());
			if (dist < myFuzzyValue) {
				bool bInFace = PointInPolyhedronFaceWithoutOn(v->GetPnt(), f);
				if (bInFace) {
					VertexPtr newVertex = make_shared<Vertex>();
					newVertex->SetPnt(v->GetPnt());
					newVertex->SetTol(dist);
					vertex2InfoMap[v] = newVertex;

					const std::vector<EdgePtr>& edges = myShells[vShellIndex]->GetEdges();
					for (auto index : v->GetEdgeIndexs()) {
						unique_ptr<OriginEdgeBoolInfo>& originEdgeBoolInfo = GetBooleanEdgeInfo(edges[index]);
						for (auto& pave : originEdgeBoolInfo->paves) {
							if (pave->vertex == v) {
								pave->vertex = newVertex;
							}
						}
					}
					// put v to adjecent faces
					GetBooleanFaceInfo(f)->myVerticesOnFace.push_back(newVertex);
					vector<size_t>& firstEdgeIndex = v->GetEdgeIndexs();
					const vector<EdgePtr>& firstEdges = myShells[vShellIndex]->GetEdges();
					for (auto val : firstEdgeIndex) {
						const auto& edge = firstEdges[val];
						const vector<CoedgeIndex>& coEdges = edge->GetCoEdges();
						const vector<CoFacePtr>& faces = myShells[vShellIndex]->GetCoFaces();
						for (auto coEdge : coEdges) {
							const auto& face = faces[coEdge.myFaceIndex]->GetFace();
							auto& boolFace = GetBooleanFaceInfo(face);
							if (boolFace->myVerticesOnEdge.empty() ||
								boolFace->myVerticesOnEdge.back() != newVertex) {
								boolFace->myVerticesOnEdge.push_back(newVertex);
							}
						}
					}
				}
			}
		}
		else {
		   // TODO :
		}
	}
}
//
#define DBL_MAX  1.7976931348623158e+308 // max value
bool Boolean::LineSegmentPlaneIntersect(const Handle(Geom_Line)& line, 
	const double startParam, const double endParam,
	const Handle(Geom_Plane)& plane, double& line_parameter){
	bool rc = false;
	double a, b, d, fd, t;

	gp_Pnt startPt;
	gp_Pnt endPt;
	line->D0(startParam, startPt);
	line->D0(endParam, endPt);
	//static double AminusBdotC(const gp_Pnt& A, const gp_Pnt& B, const gp_Vec& C) {
	a = DistanceWitgFlag(plane, startPt);
	b = DistanceWitgFlag(plane, endPt);
	d = a - b;
	if (d == 0.0) {
		if (fabs(a) < fabs(b)) {
			t = 0.0;
		}
		else if (fabs(b) < fabs(a)) {
			t = 1.0;
		}
		else {
			t = 0.5;
		}
	}
	else {
		d = 1.0 / d;
		fd = fabs(d);
		if (fd > 1.0 && (fabs(a) >= DBL_MAX / fd || fabs(b) >= DBL_MAX / fd)) {
			// double overflow - line is probably parallel to plane
			t = 0.5;
		}
		else {
			t = a * d;
			rc = true;
		}
	}
		line_parameter = t * (endParam - startParam) + startParam;
	return rc;
}

bool Boolean::FindPrevEdgeAndNextEdgeAccordingVertex(const FacePtr& face, const VertexPtr& v, 
	CoEdge*& prevE, CoEdge*& nextE) {

	const auto& wires = face->GetWires();
	for (auto& wire : wires){
		const auto& coedges = wire->GetCoEdges();
		int size = coedges.size();
		for (size_t i = 0; i < size; i++) {
			const auto& coedge = coedges[i];
			const auto& edgeInfo = edge2InfoMap.find(coedge->GetEdge());
			if (edgeInfo != edge2InfoMap.end()) {
				const auto& boolEdge = *edgeInfo->second;

				if (boolEdge.myPaveBlocks[0]->myFirst->vertex == v) {
					if (coedge->GetSameOrient()) {
						prevE = coedges[(i - 1 + size) % size].get();
						nextE = coedge.get();
					}
					else {
						prevE = coedge.get();
						nextE = coedges[(i + 1) % size].get();
					}
					return true;
				}

				if (boolEdge.myPaveBlocks.back()->myLast->vertex == v) {
					if (!coedge->GetSameOrient()) {
						prevE = coedges[(i - 1 + size) % size].get();
						nextE = coedge.get();
					}
					else {
						prevE = coedge.get();
						nextE = coedges[(i + 1) % size].get();
					}
					return true;
				}

				for (size_t j = 0; j < boolEdge.myPaveBlocks.size(); j++)
				{
					const auto& paveBlock = boolEdge.myPaveBlocks[j];
					if (paveBlock->myFirst->vertex == v || paveBlock->myLast->vertex == v) {
						prevE = coedge.get();
						nextE = coedge.get();
						return true;
					}
				}
			}
		}
	}

	return false;
}


//
bool Boolean::PerformEF() {
	for (int i = 0; i < myTopoIntpair[4].size(); i++) {
		auto& EFPair = myTopoIntpair[4][i];
		size_t* firstTopo = EFPair->first;
		size_t* lastTopo = EFPair->last;
		EdgePtr e;
		FacePtr f;
		size_t eIndex;
		size_t fIndex;

		if (firstTopo[1] == 1) {//EF
			e = myShells[firstTopo[0]]->GetEdges()[firstTopo[2]];
			f = myShells[lastTopo[0]]->GetCoFaces()[lastTopo[2]]->GetFace();
			eIndex = firstTopo[0];
			fIndex = lastTopo[0];
		}
		else {//FE
			e = myShells[lastTopo[0]]->GetEdges()[lastTopo[2]];
			f = myShells[firstTopo[0]]->GetCoFaces()[firstTopo[2]]->GetFace();
			eIndex = lastTopo[0];
			fIndex = firstTopo[0];
		}
        
		if (myIsPloyHendral) {
			unique_ptr<OriginEdgeBoolInfo>& booleanEdge = GetBooleanEdgeInfo(e);
			const Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(f->GetSurface());
			const Handle(Geom_Line) aLine = Handle(Geom_Line)::DownCast(e->GetCurve());
			double lineParam = 0.0;
			bool suc = LineSegmentPlaneIntersect(aLine, e->GetFirstParam(), e->GetLastParam(), aPlane, lineParam);
			if (suc) {
				if (lineParam > e->GetLastParam() + distTol || lineParam < e->GetFirstParam() - distTol) {
					continue;
				}

				bool bEdgeEndPoint = false;
				gp_Pnt pt;
				aLine->D0(lineParam, pt);
				for (auto& pave : booleanEdge->paves)
				{
					if (pave->vertex->GetPnt().SquareDistance(pt) < myFuzzyValue * myFuzzyValue) {
						bEdgeEndPoint = true;
					    break;
					}
				}

				if (bEdgeEndPoint) {
					continue;
				}

				bool bInFace = PointInPolyhedronFaceWithoutOn(pt, f);
				if (bInFace) {
					VertexPtr newVertex = make_shared<Vertex>(pt, distTol);
					auto& paveBlocks = booleanEdge->myPaveBlocks;
					for (int j = 0; j < paveBlocks.size(); j++) {
						if (lineParam > paveBlocks[j]->myFirst->param && lineParam < paveBlocks[j]->myLast->param) {
							booleanEdge->paves.push_back(std::unique_ptr<Pave>(new Pave(newVertex, lineParam)));
							paveBlocks[j]->mySplitPaves.push_back(booleanEdge->paves.back().get());
							break;
						}
					}

					//put pave to adjecent faces
					const vector<CoedgeIndex>& coEdges = e->GetCoEdges();
					const vector<CoFacePtr>& faces = myShells[eIndex]->GetCoFaces();
					for (const auto& coEdge : coEdges) {
						const FacePtr& face = faces[coEdge.myFaceIndex]->GetFace();
						auto& boolFace = GetBooleanFaceInfo(face);
						boolFace->myVerticesOnEdge.push_back(newVertex);
					}
					GetBooleanFaceInfo(f)->myVerticesSection.push_back(newVertex);
				}
			}
		}
		else {
			// TODO :
		}
	}

	//Split Pave Block
	for (auto& it : edge2InfoMap) {
		if (!SplitPaveBlocks(it.second)) {
			return false;
		}
	}

	//same domain pave block, put into myEdgeOnFaceBoolInfos
	for (int efPairI = 0; efPairI < myTopoIntpair[4].size(); efPairI++) {
		const auto& efPair = myTopoIntpair[4][efPairI];
		size_t* firstTopo = efPair->first;
		size_t* lastTopo = efPair->last;
		EdgePtr e;
		FacePtr f;
		size_t eIndex;
		size_t fIndex;

		if (firstTopo[1] == 1) {//EF
			e = myShells[firstTopo[0]]->GetEdges()[firstTopo[2]];
			f = myShells[lastTopo[0]]->GetCoFaces()[lastTopo[2]]->GetFace();
			eIndex = firstTopo[0];
			fIndex = lastTopo[0];
		}
		else {//FE
			e = myShells[lastTopo[0]]->GetEdges()[lastTopo[2]];
			f = myShells[firstTopo[0]]->GetCoFaces()[firstTopo[2]]->GetFace();
			eIndex = lastTopo[0];
			fIndex = firstTopo[0];
		}

		if (myIsPloyHendral) {
			unique_ptr<OriginEdgeBoolInfo>& booleanEdge = GetBooleanEdgeInfo(e);
			auto& paveBlocks = booleanEdge->myPaveBlocks;
			for (int j = 0; j < paveBlocks.size(); j++) {
				unique_ptr<PaveBlock>& paveBlock = paveBlocks[j];
				Pave* paveA = paveBlock->myFirst;
				Pave* paveB = paveBlock->myLast;

				auto& boolFaceInfo = GetBooleanFaceInfo(f);
				bool containsPaveAB = boolFaceInfo->ContainsVertex(paveA->vertex) &&
					boolFaceInfo->ContainsVertex(paveB->vertex);

				if (containsPaveAB) {
					if (paveBlock->myCommonBlockIndex == -1) {
						Handle(Geom2d_Line) aLine2d = Line3dToLine2d(
							Handle(Geom_Line)::DownCast(e->GetCurve()),Handle(Geom_Plane)::DownCast(f->GetSurface()));
						CoEdge* ce1;
						CoEdge* ce2;
						PositionState position = PositionState::UnKnown;
						////paveA on face's edge
						if (FindPrevEdgeAndNextEdgeAccordingVertex(f, paveA->vertex, ce1, ce2)) {
							position = EdgeOnFacePosition(ce1, ce2, aLine2d->Direction());
							if (position == PositionState::In) {
								myEdgeOnFaceBoolInfos.push_back(std::unique_ptr<EdgeOnFaceBoolInfo>(new EdgeOnFaceBoolInfo(aLine2d)));
									boolFaceInfo->myEdgeOnFaceBoolInfos.push_back(myEdgeOnFaceBoolInfos.back().get());
									paveBlock->myPaveBlockOnFaceIndex = myEdgeOnFaceBoolInfos.size() - 1;
							}
							else if (position == PositionState::Out) {
							}
							else {
								bool bIN = PointInPolyhedronFaceWithoutOn(paveA->vertex->GetPnt(), f);
									if (bIN) {
										myEdgeOnFaceBoolInfos.push_back(std::unique_ptr<EdgeOnFaceBoolInfo>(new EdgeOnFaceBoolInfo(aLine2d)));
										boolFaceInfo->myEdgeOnFaceBoolInfos.push_back(myEdgeOnFaceBoolInfos.back().get());
										paveBlock->myPaveBlockOnFaceIndex = myEdgeOnFaceBoolInfos.size() - 1;
									}
							}
					
						}//paveB on face's edge
						else if (FindPrevEdgeAndNextEdgeAccordingVertex(f, paveB->vertex, ce1, ce2)) {
							if(EdgeOnFacePosition(ce1, ce2, aLine2d->Direction()) == PositionState::Out) {
								myEdgeOnFaceBoolInfos.push_back(std::unique_ptr<EdgeOnFaceBoolInfo>(new EdgeOnFaceBoolInfo(aLine2d)));
								boolFaceInfo->myEdgeOnFaceBoolInfos.push_back(myEdgeOnFaceBoolInfos.back().get());
								paveBlock->myPaveBlockOnFaceIndex = myEdgeOnFaceBoolInfos.size() - 1;
							}
							else if (position == PositionState::In) {
							}
							else {
								bool bIN = PointInPolyhedronFaceWithoutOn(paveA->vertex->GetPnt(), f);
								if (bIN) {
									myEdgeOnFaceBoolInfos.push_back(std::unique_ptr<EdgeOnFaceBoolInfo>(new EdgeOnFaceBoolInfo(aLine2d)));
									boolFaceInfo->myEdgeOnFaceBoolInfos.push_back(myEdgeOnFaceBoolInfos.back().get());
									paveBlock->myPaveBlockOnFaceIndex = myEdgeOnFaceBoolInfos.size() - 1;
								}
							}
						}// paveA, paveB in face
						else{
							bool bIN = PointInPolyhedronFaceWithoutOn(paveA->vertex->GetPnt(), f);
							if (bIN) {
								myEdgeOnFaceBoolInfos.push_back(std::unique_ptr<EdgeOnFaceBoolInfo>(new EdgeOnFaceBoolInfo(aLine2d)));
								boolFaceInfo->myEdgeOnFaceBoolInfos.push_back(myEdgeOnFaceBoolInfos.back().get());
								paveBlock->myPaveBlockOnFaceIndex = myEdgeOnFaceBoolInfos.size() - 1;
							}
						}
					}
				}
			}
		}
	}
	return true;
}
//
bool Boolean::PlanePlaneIntersect(const Handle(Geom_Plane)& sA,
	const Handle(Geom_Plane)& sB, Handle(Geom_Line)& resultLine,
	Handle(Geom2d_Line)& pCurveA, Handle(Geom2d_Line)& pCurveB, bool bPlanACrossPlaneB) {
	bool rc = false;
	gp_XYZ NC;
	if (bPlanACrossPlaneB) {
		NC = sA->Position().Direction().XYZ().Crossed(sB->Position().Direction().XYZ());
	}else {
		NC = sB->Position().Direction().XYZ().Crossed(sA->Position().Direction().XYZ());
	}
	if (NC.Modulus() < 1.0e-11) {
		return rc;
	}
	else {
		// P = ((P0*N0)(n1^n2) + (P1*n1)(n2^n0) +(P2*n2)(n0^n1))/(n0*(n1^n2))
		const gp_Pnt& pA = sA->Location();
		const gp_Pnt& pB = sB->Location();
		gp_Pnt pC = pA;
		pC.BaryCenter(0.5, pB, 0.5);

		const gp_XYZ& NA = sA->Position().Direction().XYZ();
		const gp_XYZ& NB = sB->Position().Direction().XYZ();
		NC.Normalize();

		const gp_XYZ& v1 = NB.Crossed(NC);
		const gp_XYZ& v2 = NC.Crossed(NA);
		const gp_XYZ& v3 = NA.Crossed(NB);
		double coff = 1.0 / NA.Dot(v1);
		gp_XYZ pt;
		pt.SetLinearForm(pA.XYZ().Dot(NA) * coff, v1, pB.XYZ().Dot(NB) * coff, v2, pC.XYZ().Dot(NC) * coff, v3);
		resultLine = new Geom_Line(pt, NC);
		double U, V;
		ElSLib::Parameters(sA->Pln(), pt, U, V);
		gp_Pnt2d uvA(U, V);
		ElSLib::Parameters(sB->Pln(), pt, U, V);
		gp_Pnt2d uvB(U, V);

		gp_Dir2d dirA(NC.Dot(sA->Pln().Position().XDirection().XYZ()),
			NC.Dot(sA->Pln().Position().YDirection().XYZ()));
		gp_Dir2d dirB(NC.Dot(sB->Pln().Position().XDirection().XYZ()),
			NC.Dot(sB->Pln().Position().YDirection().XYZ()));
		pCurveA  = new Geom2d_Line(uvA, dirA);
		pCurveB = new Geom2d_Line(uvB, dirB);

		rc = true;
		return rc;
	}
};
//


bool Boolean::CollectPavesForPlanPlaneIntresectLine(const CoFacePtr& fA, const CoFacePtr& fB,
	const Handle(Geom_Line)& aLine, vector<pair<VertexPtr, PaveOnIntersectionCurve>>& validVertices) {

	unordered_map <VertexPtr, PaveOnIntersectionCurve, VertexPtrHash, VertexPtrEqual> validVertexsSetA;
	unordered_map <VertexPtr, PaveOnIntersectionCurve, VertexPtrHash, VertexPtrEqual> validVertexsSetB;
	vector<PaveBlock*> coincidencePaveBlocks;

	auto& boolFaceAInfo = GetBooleanFaceInfo(fA->GetFace());
	auto& boolFaceBInfo = GetBooleanFaceInfo(fB->GetFace());
	auto& vAs = boolFaceAInfo->AllVertices();
	auto& vBs = boolFaceBInfo->AllVertices();

	std::unordered_set <VertexPtr, VertexPtrHash, VertexPtrEqual> commonVertex;
	std::set_intersection(vAs.begin(), vAs.end(), vBs.begin(), vBs.end(), std::inserter(commonVertex, commonVertex.end()));
	if (commonVertex.size() >= 2) {
		GetValidPavesFromFace(fA,  coincidencePaveBlocks, commonVertex, validVertexsSetA, aLine);
		GetValidPavesFromFace(fB,  coincidencePaveBlocks, commonVertex, validVertexsSetB, aLine);
	}

	for (auto& vertex : validVertexsSetB) {
		auto it = validVertexsSetA.find(vertex.first);
		if (it == validVertexsSetA.end()) {
			validVertexsSetA.insert(std::move(vertex));
		}
		else{
			if (vertex.second.left == PositionState::Out) {
				it->second.left = PositionState::Out;
			}
			if (vertex.second.right == PositionState::Out) {
				it->second.right = PositionState::Out;
			}
		}
	}

	validVertices.reserve(validVertexsSetA.size());
	for (auto& vertex : validVertexsSetA){
		validVertices.push_back(std::move(vertex));
	}
	
	std::sort(validVertices.begin(), validVertices.end(),
		[](const pair<VertexPtr, PaveOnIntersectionCurve>& a, 
			const pair<VertexPtr, PaveOnIntersectionCurve>& b) {
		return a.second.param < b.second.param;
	});

	for (auto& coincidence : coincidencePaveBlocks) { 
		auto& va = coincidence->myFirst->vertex;
		auto& vb = coincidence->myLast->vertex;
		int size = validVertices.size() - 1;
		for (int i = 0; i < size; i++) {
			auto& vc = validVertices[i];
			auto& vd = validVertices[i + 1];
			if ((vc.first == va && vd.first == vb) || (vc.first == vb && vd.first == va)) {
				vc.second.right = PositionState::On;
				vd.second.left = PositionState::On;
			}
		}
	}

	return true;
}

void Boolean::GetValidPavesFromFace(const CoFacePtr& fA,
	std::vector<PaveBlock*> &coincidencePaveBlock,
	const std::unordered_set <VertexPtr, VertexPtrHash, VertexPtrEqual>& commonVertex,
	unordered_map <VertexPtr, PaveOnIntersectionCurve, VertexPtrHash, VertexPtrEqual>& validVertexsSet,
	const Handle(Geom_Line) & aLine) {
	const auto& wiresA = fA->GetFace()->GetWires();
	for (auto& wire : wiresA) {
		const vector<CoEdgePtr>& coedges = wire->GetCoEdges();
		for (int coedgeI = 0; coedgeI < coedges.size(); coedgeI++) {
			const auto& coedge = coedges[coedgeI];
			const auto& edge = coedge->GetEdge();
			auto booleanEdge = edge2InfoMap.find(edge);
			if (booleanEdge != edge2InfoMap.end()) {
				const auto& paveBlocks = booleanEdge->second->myPaveBlocks;
				vector<VertexPtr> mayValidVerticesOnCoedge;
				vector<CoEdge*> ce1s;
				vector<CoEdge*> ce2s;

				Pave* pave = paveBlocks.front()->myFirst;
				bool firstMayValid = commonVertex.find(pave->vertex) != commonVertex.end();
				if (firstMayValid) {
					mayValidVerticesOnCoedge.push_back(pave->vertex);
					if (coedge->GetSameOrient()) {
						ce1s.push_back(coedges[(coedgeI - 1 + coedges.size()) % coedges.size()].get());
						ce2s.push_back( coedge.get());
					}
					else {
						ce1s.push_back(coedge.get());
						ce2s.push_back(coedges[(coedgeI + 1) % coedges.size()].get());
					}
				}

				for (int i = 0; i < paveBlocks.size(); i++) {
					const auto& paveBlock = paveBlocks[i];
					pave = paveBlock->myLast;
					bool lastMayValid = commonVertex.find(pave->vertex) != commonVertex.end();
					if (lastMayValid) {
						mayValidVerticesOnCoedge.push_back(pave->vertex);
						if (i == paveBlocks.size() - 1) {
							if (coedge->GetSameOrient()) {
								ce1s.push_back(coedge.get());
								ce2s.push_back(coedges[(coedgeI + 1) % coedges.size()].get());
							}
							else {
								ce1s.push_back(coedges[(coedgeI - 1 + coedges.size()) % coedges.size()].get());
								ce2s.push_back(coedge.get());
							}
						}
						else {
							ce1s.push_back(coedge.get());
							ce2s.push_back(coedge.get());
						}
					}

					if (firstMayValid && lastMayValid) {
						coincidencePaveBlock.push_back(paveBlock.get());
					}

					firstMayValid = lastMayValid;
				}

				if (mayValidVerticesOnCoedge.empty()) {
					continue;
				}
				else if (mayValidVerticesOnCoedge.size() == 1) {
					auto it = validVertexsSet.find(mayValidVerticesOnCoedge[0]);
					if (it == validVertexsSet.end()) {
						double u = AminusBdotC(mayValidVerticesOnCoedge[0]->GetPnt(),
							aLine->Lin().Location(), aLine->Lin().Direction());
						auto plane = Handle(Geom_Plane)::DownCast(fA->GetFace()->GetSurface())->Pln();
						auto d3 = aLine->Lin().Direction();
						double du = plane.Position().XDirection().Dot(d3);
						double dv = plane.Position().YDirection().Dot(d3);

						PositionState position0 = EdgeOnFacePosition(ce1s[0], ce2s[0], gp_Dir2d(-du, -dv));
						PositionState position1 = EdgeOnFacePosition(ce1s[0], ce2s[0], gp_Dir2d(du, dv));
						validVertexsSet.insert({ mayValidVerticesOnCoedge[0], PaveOnIntersectionCurve(u, position0, position1)});
					}
				}
				else {
					int i = 0;
					for (const auto& mayValid :mayValidVerticesOnCoedge) {
						auto it = validVertexsSet.find(mayValid);
						if (it == validVertexsSet.end() && LineDistanceToPoint(aLine, mayValid->GetPnt()) < myFuzzyValue) {
							double u = AminusBdotC(mayValid->GetPnt(), aLine->Lin().Location(), aLine->Lin().Direction());
							auto plane = Handle(Geom_Plane)::DownCast(fA->GetFace()->GetSurface())->Pln();
							auto d3 = aLine->Lin().Direction();
							double du = plane.Position().XDirection().Dot(d3);
							double dv = plane.Position().YDirection().Dot(d3);
							PositionState position0 = EdgeOnFacePosition(ce1s[i], ce2s[i], gp_Dir2d(-du, -dv));
							PositionState position1 = EdgeOnFacePosition(ce1s[i], ce2s[i], gp_Dir2d(du, dv));
							validVertexsSet.insert({ mayValid, PaveOnIntersectionCurve(u, position0, position1) });
						}
						i++;
					}

					continue;
				}
			}
		}
	}
}

//
void Boolean::PerformFF() {
	for (int i = 0; i < myTopoIntpair[5].size(); i++) {
		auto& FFPare = myTopoIntpair[5][i];
		size_t* faceAIndex = FFPare->first;
		size_t* faceBIndex = FFPare->last;
		const auto& fA = myShells[0]->GetCoFaces()[faceAIndex[2]];
		const auto& fB = myShells[1]->GetCoFaces()[faceBIndex[2]];

		if (myIsPloyHendral) {
			Handle(Geom_Plane) aPlaneA = Handle(Geom_Plane)::DownCast(fA->GetFace()->GetSurface());
			Handle(Geom_Plane) aPlaneB = Handle(Geom_Plane)::DownCast(fB->GetFace()->GetSurface());
			Handle(Geom_Line) aLine;
			Handle(Geom2d_Line) aPcurveA, aPcurveB;
			// coedge0(sameOrition = true) on A   
			// coedge1(sameOrition = false)  on A 
			// coedge2(sameOrition = true)  on B  
			// coedge3(sameOrition = false)  on B 
			// bPlanACrossPlaneB keeps coedge0 is valid
			bool bPlanACrossPlaneB = (fA->GetSameOrient() == fB->GetSameOrient()) == (BooleanType::INTERSECT == myTp);
			bool suc = PlanePlaneIntersect(aPlaneA, aPlaneB, aLine, aPcurveA, aPcurveB, bPlanACrossPlaneB);
			if (suc) {
				vector<pair<VertexPtr, PaveOnIntersectionCurve>> validVertex;
				CollectPavesForPlanPlaneIntresectLine(fA, fB, aLine, validVertex);
				if (validVertex.size() > 1) {
					int A = faceAIndex[2];
					int B = faceBIndex[2];

					mySectionEdgeBoolInfos.push_back(
						std::unique_ptr<SectionEdgeBoolInfo> (new SectionEdgeBoolInfo(std::move(validVertex), 
							aLine, aPcurveA, aPcurveB, A, B)));
					auto& faceInfoA = GetBooleanFaceInfo(fA->GetFace());
					faceInfoA->mySectionEdgeBoolInfos.push_back(mySectionEdgeBoolInfos.back().get());
					auto& faceInfoB = GetBooleanFaceInfo(fB->GetFace());
					faceInfoB->mySectionEdgeBoolInfos.push_back(mySectionEdgeBoolInfos.back().get());
				}
			}
		}
		else {
		    // TODO:
			throw;
		}
	}
}

void Boolean::BuildEdge() {
	for (auto& sectionEdge : mySectionEdgeBoolInfos) {
		for (int i = 0; i < sectionEdge->myVertexs.size() - 1; i++) {
			//vector<pair<VertexPtr, pair<double, int>>> myVertexs;
			PositionState right = sectionEdge->myVertexs[i].second.right;
			PositionState left = sectionEdge->myVertexs[i + 1].second.left;
			if (right == PositionState::In && (left == PositionState::In || left == PositionState::UnKnown) ||
				left == PositionState::In && (right == PositionState::In || right == PositionState::UnKnown)) {
				EdgePtr edge = make_shared<Edge>(sectionEdge->myC3,
						sectionEdge->myVertexs[i].first, sectionEdge->myVertexs[i + 1].first,
						sectionEdge->myVertexs[i].second.param, sectionEdge->myVertexs[i + 1].second.param);
					sectionEdge->myEdges[i] = edge;
				}
		}
	}

	for (auto& originEdge : edge2InfoMap) {
		const auto& paveBlocks = originEdge.second->myPaveBlocks;
		for (auto& paveBlock : paveBlocks) {
			Pave* firstPave = paveBlock->myFirst;
			Pave* lastPave = paveBlock->myLast;
			EdgePtr newEdge = make_shared<Edge>(originEdge.first->GetCurve(),
				firstPave->vertex, lastPave->vertex, firstPave->param, lastPave->param);
			paveBlock->edge = newEdge;

			if (paveBlock->myCommonBlockIndex != -1) {
				auto& commonBlock = myCommonBlocks[paveBlock->myCommonBlockIndex];
				for (auto& block : commonBlock) {
					block->edge = newEdge;
				}
			}

			if (paveBlock->myPaveBlockOnFaceIndex != -1) {
				auto& edgeOnFace = myEdgeOnFaceBoolInfos[paveBlock->myPaveBlockOnFaceIndex];
				edgeOnFace->edge = newEdge;
			}
		}
	}
}


//
void Boolean::BuildCoEdge() {
	for (int iShell = 0; iShell < 2; iShell++){
		const auto& shell = myShells[iShell];
		const auto& coFaces = shell->GetCoFaces();
		for (int coFaceI = 0; coFaceI < coFaces.size(); coFaceI++) {
			const auto& coFace = coFaces[coFaceI];
			const auto& face = coFace->GetFace();
			auto booleanFace = face2InfoMap.find(face);
			if (booleanFace != face2InfoMap.end()) {
				//sectionEdgeBoolInfos
				const auto& sectionEdges = booleanFace->second->mySectionEdgeBoolInfos;
				for (const auto& sectionEdge : sectionEdges) {
					for (int i = 0; i < sectionEdge->myVertexs.size() - 1; i++) {
						//vector<pair<VertexPtr, pair<double, int>>> myVertexs;
						PositionState right = sectionEdge->myVertexs[i].second.right;
						PositionState left = sectionEdge->myVertexs[i + 1].second.left;
						if (right == PositionState::In && (left  == PositionState::In || left == PositionState::UnKnown) ||
							left == PositionState::In && (right == PositionState::In || right == PositionState::UnKnown)) {
							EdgePtr edge = sectionEdge->myEdges[i];

							if (myIsPloyHendral) {
								Handle(Geom2d_Curve) pCurve = iShell == 0 ? sectionEdge->myC2A : sectionEdge->myC2B;
								auto coedgeA = make_shared<CoEdge>(edge, pCurve, sectionEdge->myVertexs[i].second.param,
									sectionEdge->myVertexs[i + 1].second.param, true);
								SetIntersectionCoedge(coedgeA);
								if ((iShell == 0 && coFace->GetSameOrient()) || (iShell == 1 && !coFace->GetSameOrient())) {
									SetCoedgeValid(coedgeA);
								} else {
									SetCoedgeUnValid(coedgeA);
								}
								booleanFace->second->myAllCoedges.push_back(std::move(coedgeA));

								auto coedgeB = make_shared<CoEdge>(edge, pCurve, sectionEdge->myVertexs[i].second.param,
									sectionEdge->myVertexs[i + 1].second.param, false);
								SetIntersectionCoedge(coedgeB);
								if ((iShell == 0 && !coFace->GetSameOrient()) || (iShell == 1 && coFace->GetSameOrient())) {
									SetCoedgeValid(coedgeB);
								} else {
									SetCoedgeUnValid(coedgeB);
								}
								booleanFace->second->myAllCoedges.push_back(std::move(coedgeB));
							}
							else {
								//TODO:
							}
						}
					}
				}

				//edgeOnFaceCoedges
				//vector<unique_ptr<EdgeOnFaceBoolInfo>>
				const auto& edgeOnFaces = booleanFace->second->myEdgeOnFaceBoolInfos;
				for (auto& edgeOnFace : edgeOnFaces) {
					CoEdgePtr coedgeA = make_shared<CoEdge>(edgeOnFace->edge, edgeOnFace->pCurve,
						edgeOnFace->edge->GetFirstParam(), edgeOnFace->edge->GetLastParam(), true);
					SetIntersectionCoedge(coedgeA);
					booleanFace->second->myAllCoedges.push_back(std::move(coedgeA));

					CoEdgePtr coedgeB = make_shared<CoEdge>(edgeOnFace->edge, edgeOnFace->pCurve,
						edgeOnFace->edge->GetFirstParam(), edgeOnFace->edge->GetLastParam(), false);
					SetIntersectionCoedge(coedgeB);
					booleanFace->second->myAllCoedges.push_back(std::move(coedgeB));
				}
				//OriginEdgeCoedges
				const auto& wires = face->GetWires();
				for (auto& wire : wires) {
					const auto& coedges = wire->GetCoEdges();
					for (auto& coedge : coedges) {
						const auto& edge = coedge->GetEdge();
						auto boolEdgeInfo = edge2InfoMap.find(edge);
						if (boolEdgeInfo != edge2InfoMap.end()) {
							const auto& paveBlocks = boolEdgeInfo->second->myPaveBlocks;
							for (auto& paveBlock : paveBlocks) {
								Pave* firstPave = paveBlock->myFirst;
								Pave* lastPave = paveBlock->myLast;
								EdgePtr newEdge = paveBlock->edge;
								Handle(Geom2d_Curve) pCurve = coedge->GetPcurve();
								double firstParam = firstPave->param;
								double lastParam = lastPave->param;
								bool bSameDir = coedge->GetSameOrient();
								if (paveBlock->myCommonBlockIndex != -1 && newEdge->GetCurve() != edge->GetCurve()) {
									bool bCommonSameDir = newEdge->GetFirstVertex() == firstPave->vertex;
									pCurve = Line3dToLine2d(Handle(Geom_Line)::DownCast(newEdge->GetCurve()),
										Handle(Geom_Plane)::DownCast(face->GetSurface()));
									Handle(Geom2d_Line) pLine2d = Handle(Geom2d_Line)::DownCast(pCurve);
									
									gp_Pnt2d firstPt2d;
									gp_Pnt2d lastPt2d;

									coedge->GetPcurve()->D0(firstPave->param, firstPt2d);
									coedge->GetPcurve()->D0(lastPave->param, lastPt2d);
									double param1 = AminusBdotC(firstPt2d, pLine2d->Location(), pLine2d->Direction());
									double param2 = AminusBdotC(lastPt2d, pLine2d->Location(), pLine2d->Direction());
									firstParam = bCommonSameDir ? param1 : param2;
									lastParam = bCommonSameDir ? param2 : param1;
									bSameDir = coedge->GetSameOrient() == bCommonSameDir;
								}

								CoEdgePtr newCoedge = make_shared<CoEdge>(newEdge, pCurve,firstParam, lastParam, bSameDir);
								if (paveBlock->myCommonBlockIndex != -1 || paveBlock->myPaveBlockOnFaceIndex != -1) {
									SetIntersectionCoedge(newCoedge);
								}
								booleanFace->second->myAllCoedges.push_back(std::move(newCoedge));
							}
						}
						else {
							booleanFace->second->myAllCoedges.push_back(make_shared<CoEdge>(
								coedge->GetEdge(), coedge->GetPcurve(),
								coedge->GetFirstParam(), coedge->GetLastParam(), coedge->GetSameOrient()));
						}
					}
				}
			}
		}
	}
};
//
vector<int> Boolean::PopLoop(vector<int>& list, vector<CoEdgePtr>& coedges) {
	const CoEdgePtr& currentCoedge = coedges[list.back()];
	const VertexPtr& lastV = currentCoedge->GetLastVertexWithOrient();
	auto it = std::find_if(list.rbegin(), list.rend(), [&coedges, &lastV](int index) 
	{ return coedges[index]->GetFirstVertexWithOrient() == lastV; });

	vector<int> loop;
	if (it != list.rend()) {
		auto forward_it = it.base() - 1;
		std::move(forward_it, list.end(), std::back_inserter(loop));
		list.erase(forward_it, list.end());
		return loop;
	}
	else {
		return loop;
	}
}

bool Boolean::BuildLoop(){
	for (int iShell = 0; iShell <= 1; iShell++){
		const auto& shell = myShells[iShell];
		const auto& coFaces = shell->GetCoFaces();

		bool suc = true;
		for (int coFaceI = 0; coFaceI < coFaces.size(); coFaceI++) {
			const auto& coFace = coFaces[coFaceI];
			const auto& face = coFace->GetFace();
			auto booleanFace = face2InfoMap.find(face);
			if (booleanFace != face2InfoMap.end()){
				vector<CoEdgePtr>& allCoedges = booleanFace->second->myAllCoedges;
				std::set<int> used;
				vector<vector<int>> loops;

				for (int i = 0; i < allCoedges.size(); i++){
					if (used.find(i) != used.end()) {
						continue;
					}
					vector<int> list;
					list.push_back(i);
					used.insert(i);
					while (true){
						const CoEdgePtr& currentCoedge = allCoedges[list.back()];
						auto& lastVertex = currentCoedge->GetLastVertexWithOrient();

						vector<int> candidates;
						int coincidenceIndex = -1;
						for (int j = 0; j < allCoedges.size(); j++){
							const CoEdgePtr& coedge = allCoedges[j];
							if (coedge->GetFirstVertexWithOrient() == lastVertex) {
								if (coedge->GetEdge() != currentCoedge->GetEdge()) {
									candidates.push_back(j);
								}
								else {
									coincidenceIndex = j;
								}
							}
						}
						int nextHalfEdge = FindMaxLeftTurningCoedge(allCoedges,list.back(), candidates, coincidenceIndex);

						if (nextHalfEdge == list[0]) {
							break;
						}

						if (nextHalfEdge == -1 || used.find(nextHalfEdge) != used.end()) {
							suc = false;
							break;
						}

						used.insert(nextHalfEdge);
						list.push_back(nextHalfEdge);
					}
					if (suc) {
						loops.push_back(std::move(list));
					}
					else {
						break;
					}
				}
				int size = 0;
				for (auto& loop : loops){
					WirePtr wire = make_shared<Wire>();
					wire->ReserveCoedgeSize(loop.size());
					for (int i = 0; i < loop.size(); i++) {
						wire->AddCoedge(std::move(allCoedges[loop[i]]));
					}
					booleanFace->second->myWires.push_back(std::move(wire));
					size += loop.size();
				}

				if (size != allCoedges.size()) {
					return false;
				}
			}
		}
	}
	return true;
}

static double roughArea2D(const WirePtr& wire) {
	auto& coedges = wire->GetCoEdges();
	int size = coedges.size();
	vector<double> x(size);
	vector<double> y(size);

	double sumX = 0.0;
	for (int coedgeI = 0; coedgeI < coedges.size(); coedgeI ++) {
		const auto& coedge = coedges[coedgeI];
		Handle(Geom2d_Curve) pCurve = coedge->GetPcurve();
		gp_Pnt2d P;
		pCurve->D0(coedge->GetFirstParamWithOrient(), P);
		sumX += P.X();
		x[coedgeI] = P.X();
		y[coedgeI] = P.Y();
	}

	double meanX = sumX / size;
	double area = 0.0;
	for (int i = 0; i < size; i++){
		area += (x[(i + 1)% size] - meanX)*(y[(i + 2)% size] - y[i]);
	}
	return area / 2.0;
}

//
void Boolean::BuildFace() {
	myFaces.reserve(myShells[0]->GetCoFaces().size() + myShells[1]->GetCoFaces().size());
	for (int iShell = 0; iShell <= 1; iShell++) {
		const auto& shell = myShells[iShell];
		const auto& coFaces = shell->GetCoFaces();
		for (int j = 0; j < coFaces.size();  j++) {
			const auto& coface = coFaces[j];
			const auto& face = coface->GetFace();
			auto booleanFace = face2InfoMap.find(face);
			if (booleanFace != face2InfoMap.end()) {
				auto& wires = booleanFace->second->myWires;
				vector<pair<int, double>> outer;
				vector<pair<int, double>> inner;
				for (int i = 0; i < wires.size(); i++) {
					double area = roughArea2D(wires[i]);
					if (area > 1.0e-10) {
						outer.push_back({ i, area });
					}
					else {
						inner.push_back({ i, area });
					}
				}

				// begin nested
				std::sort(outer.begin(), outer.end(),
					[](const std::pair<int, double>& pair1, const std::pair<int, double>& pair2) {
					return pair1.second < pair2.second;
				});

				vector<vector<int>> subFaceWireIndex(outer.size());
				for (int i = 0; i < outer.size(); i++) {
					subFaceWireIndex[i].push_back(outer[i].first);
				}

				if (outer.size() == 1) {
					for (auto innerIndex : inner) {
						subFaceWireIndex[0].push_back(innerIndex.first);
					}
				}
				else {
					for (const auto& innerIndex : inner) {
						double innerArea = innerIndex.second;
						const auto& innerWire = wires[innerIndex.first];
						auto innerVs = innerWire->GetVertices();
						for (int outerIndex = 0; outerIndex < outer.size(); outerIndex++) {
							const auto& outerWire = wires[outer[outerIndex].first];
							double outerArea = outer[outerIndex].second;

							if (std::abs((outerArea + innerArea)) < 1.0e-6) {
								continue;
							}

							const auto& outerVs = outerWire->GetVertices();
							vector<gp_Pnt2d> pt2ds;
							GetVertices2ds(outerWire, pt2ds);

							bool bFindOut = false;

							for (int index = 0; index < innerVs.size(); index++) {
								auto it = std::find(outerVs.begin(), outerVs.end(), innerVs[index]);
								if (it == outerVs.end()) {
									gp_Pnt2d P;
									double param = wires[innerIndex.first]->GetCoEdges()[index]->GetFirstParamWithOrient();
									wires[innerIndex.first]->GetCoEdges()[index]->GetPcurve()->D0(param, P);
									if (IsPointInPolygonWithoutOn(P.X(), P.Y(), pt2ds)) {
										subFaceWireIndex[outerIndex].push_back(innerIndex.first);
										bFindOut = true;
										break;
									}
									else {
										break;
									}
								}
							}

							if (!bFindOut) {
								vector<gp_Pnt2d> pt2ds2;
								GetVertices2ds(outerWire, pt2ds2);
								const auto& coedges = innerWire->GetCoEdges();
								for (int i = 0; i < coedges.size(); i++){
									gp_Pnt2d P;
									const auto& coedge = coedges[i];
									coedge->GetPcurve()->D0((coedge->GetFirstParam() + coedge->GetLastParam())*0.5, P);
									if (IsPointInPolygonWithoutOn(P.X(), P.Y(), pt2ds2)) {
										subFaceWireIndex[outerIndex].push_back(innerIndex.first);
										bFindOut = true;
										break;
									}
								}
							}

							if (bFindOut) {
								break;
							}
						}
					}
				}
				for (auto faceWires : subFaceWireIndex) {
					FacePtr subFace = make_shared<Face>();
					subFace->SetSurface(coface->GetFace()->GetSurface());
					for (auto wireIndex : faceWires) {
						subFace->AddWire(std::move(wires[wireIndex]));
					}
					myFaces.push_back(make_shared<CoFace>(subFace, coface->GetSameOrient()));
				}
			}
			else {
				myFaces.push_back(make_shared<CoFace>(coface->GetFace(), coface->GetSameOrient()));
			}
		}

		if (iShell == 0) {
			myFacesASize = myFaces.size();
		}
	}
};

vector<int> Boolean::GetCoFacesGroup(int iCoFace, const ShellPtr& shell) {
	const std::vector<CoFacePtr>& cofaces = shell->GetCoFaces();
	stack<int> stack;
	stack.push(iCoFace);
	vector<int> group;
	while (!stack.empty()) {
		int currentFaceIndex = stack.top();
		stack.pop();
		if (cofaces[currentFaceIndex]->IsVisited()) {
			continue;
		}
		group.push_back(currentFaceIndex);
		cofaces[currentFaceIndex]->SetVisited();
		for (const auto& wire : cofaces[currentFaceIndex]->GetFace()->GetWires()) {
			for (const auto& coedge : wire->GetCoEdges()) {
				auto twinCoedges = coedge->GetTwins(shell);
				for (const CoedgeIndex& twin : twinCoedges){
					const CoFacePtr& coFace = shell->GetCoFaces()[twin.myFaceIndex];
					 const CoEdgePtr& twinCoedge = coFace->GetFace()->GetWires()
						 [twin.myWireIndex]->GetCoEdges()[twin.myCoedgeIndex];
					if (!IsIntersectionCoedge(twinCoedge)) {
						stack.push(twin.myFaceIndex);
					}
				}
			}
		}
	}
	return group;
}

vector<vector<int>> Boolean::GetGroups(const ShellPtr& shell) {
	vector<vector<int>> groups;
	const auto& cofaces = shell->GetCoFaces();

	for (int iCoFace = 0; iCoFace < cofaces.size(); iCoFace++) {
		const CoFacePtr& coface = cofaces[iCoFace];
		if (coface->IsVisited()) {
			continue;
		}

		const vector<int>& group = GetCoFacesGroup(iCoFace, shell);
		if (group.size() == 0) {
			return vector<vector<int>>();
		}
		groups.push_back(group);
	}

	for (auto& coface : cofaces){
		coface->CleanVisited();
	}
	return groups;
}

int Boolean::IsCoFaceValidByIntersectionBoundery(const CoFacePtr& coface) {
	 for (const auto& wire : coface->GetFace()->GetWires()) {
		 for (const auto& coedge : wire->GetCoEdges()) {
			 if (IsIntersectionCoedge(coedge)) {
				 if (IsCoedgeValid(coedge) && !IsCoedgeUnValid(coedge)) {
					 return 1;
				 }
				 if (!IsCoedgeValid(coedge) && IsCoedgeUnValid(coedge)) {
					 return -1;
				 }
			 }
		 }
	 }
	 return 0;
 }
 
 PositionState Boolean::FaceInOutStatus(const CoFacePtr& coface, const CoEdgePtr& coedge,
	 const CoFacePtr& faceAdj0, const CoEdgePtr& coedge0, 
	 const CoFacePtr& faceAdj1, const CoEdgePtr& coedge1) {
	 
	 double faceAdj0TofaceAngle = 0.0;
	 double faceAdj0ToFaceAdj1Angle = 0.0;

	 gp_Pnt2d pt2d;
	 gp_Pnt P;
	 gp_Vec Dx, Dy, T;
	 gp_Dir N;
	 Handle(Geom2d_Curve) pCurve;
	 // faceAdj0 innerSide
	 pCurve = coedge0->GetPcurve();
	 double param = 0.5*(coedge0->GetFirstParam() + coedge0->GetLastParam());
	 pCurve->D0(param, pt2d);
	 faceAdj0->GetFace()->GetSurface()->D1(pt2d.X(), pt2d.Y(), P, Dx, Dy);
	 coedge0->GetEdge()->GetCurve()->D1(param, P, T);
	 N = Dx^Dy;
	 if (!faceAdj0->GetSameOrient()) {
		 N.Reverse();
	 }
	 if (coedge0->GetSameOrient() != faceAdj0->GetSameOrient()) {
		 T.Reverse();
	 }
	 gp_Dir InnerSide0 = N^T;
	 gp_Dir axis = N^InnerSide0;
	 
	 //face innerside
	 pCurve = coedge->GetPcurve();
	 param = 0.5*(coedge->GetFirstParam() + coedge->GetLastParam());
	 pCurve->D0(param, pt2d);
	 coface->GetFace()->GetSurface()->D1(pt2d.X(), pt2d.Y(), P, Dx, Dy);
	 coedge->GetEdge()->GetCurve()->D1(param, P, T);
	 N = Dx ^ Dy;
	 if (!coface->GetSameOrient()) {
		 N.Reverse();
	 }
	 if (coedge->GetSameOrient() != coface->GetSameOrient()) {
		 T.Reverse();
	 }
	 gp_Dir InnerSide = N ^ T;
	 faceAdj0TofaceAngle = AngleWithRefRoughly(InnerSide0, InnerSide, axis);

	 // faceAdj1 innerSide
	 if (faceAdj0->GetFace()->GetSurface() == faceAdj1->GetFace()->GetSurface()
		 && faceAdj0->GetSameOrient() == faceAdj1->GetSameOrient()) {
		 faceAdj0ToFaceAdj1Angle = M_PI;
	 }
	 else {
		 pCurve = coedge1->GetPcurve();
		 param = 0.5*(coedge1->GetFirstParam() + coedge1->GetLastParam());
		 pCurve->D0(param, pt2d);
		 faceAdj1->GetFace()->GetSurface()->D1(pt2d.X(), pt2d.Y(), P, Dx, Dy);
		 coedge1->GetEdge()->GetCurve()->D1(param, P, T);
		 N = Dx ^ Dy;
		 if (!faceAdj1->GetSameOrient()) {
			 N.Reverse();
		 }
		 if (coedge1->GetSameOrient() != faceAdj1->GetSameOrient()) {
			 T.Reverse();
		 }
		 gp_Dir InnerSide1 = N ^ T;
		 faceAdj0ToFaceAdj1Angle = AngleWithRefRoughly(InnerSide0, InnerSide1, axis);
	 }

	 double angleTol = 1.0e-5;
	 if (std::abs(faceAdj0TofaceAngle) < angleTol ||
		 std::abs(faceAdj0TofaceAngle - 2 * M_PI) < angleTol ||
		 std::abs(faceAdj0ToFaceAdj1Angle) < angleTol ||
		 std::abs(faceAdj0ToFaceAdj1Angle - 2 * M_PI) < angleTol ||
		 std::abs(faceAdj0ToFaceAdj1Angle - faceAdj0TofaceAngle) < angleTol) {
		 return PositionState::UnKnown;
	 }
	 else{
		 auto position = faceAdj0TofaceAngle > faceAdj0ToFaceAdj1Angle ? PositionState::Out : PositionState::In;
		 return position;
	 }
 }


//groups[0] valid faces
//groups[1] unvalid faces
//groups[2] unKnown faces
vector<vector<int>> Boolean::ClassificationFaces(const ShellPtr& shell) {
	 vector<vector<int>> groups(3);
	 const auto& cofaces = shell->GetCoFaces();
	 for (int iCoFace = 0; iCoFace < cofaces.size(); iCoFace++) {
		 const CoFacePtr& coface = cofaces[iCoFace];
		 int valid = IsCoFaceValidByIntersectionBoundery(coface);
		 if (valid == 1) {
			 groups[0].push_back(iCoFace);
		 }
		 else if (valid == -1) {
			 groups[1].push_back(iCoFace);
		 }
		 else {
			 groups[2].push_back(iCoFace);
		 }
	 }
	 return groups;
}

bool Boolean::GroupValidition(PositionState pos, BooleanType tp, bool sameDirNormal, bool isShellA) {
	if (pos == PositionState::In && tp == BooleanType::INTERSECT) {
		return true;
	}
	else if (pos == PositionState::Out && tp == BooleanType::INTERSECT) {
		return false;
	}
	else if (pos == PositionState::In && tp == BooleanType::UNION) {
		return false;
	}
	else if (pos == PositionState::Out && tp == BooleanType::UNION) {
		return true;
	}
	else {
		// PositionState::ON
		if (sameDirNormal && isShellA) {
			return true;
		}
		else {
			return false;
		}
	}
}

PositionState Boolean::IsPointInPolyHendralWithoutOn(const gp_Pnt& queriePt, const ShellPtr& shell) {
	const auto& cofaces = shell->GetCoFaces();
	double solidAngle = 0.0;
	for (auto& coface : cofaces){
		auto dSolidAngle = [&coface, &queriePt](double u, double v) -> double {
			gp_Pnt P;
			gp_Vec D1U;
			gp_Vec D1V;
			coface->GetFace()->GetSurface()->D1(u, v, P, D1U, D1V);
			gp_Vec r = gp_Vec(queriePt, P);
			gp_Dir n = D1U.Crossed(D1V);
			if (!coface->GetSameOrient()) {
				n.Reverse();
			}
			double len = r.Magnitude();
			return r.Dot(n) / (len * len * len);
		};
		solidAngle += QUAUDG::preform2(dSolidAngle, coface, 1.0e-2, 1.0e-2);
	}
	//(-PI, PI)out : -1
	//(PI, 3PI)unknown : 0 
	//(3PI, 5PI)in : 1
	//soldAngle = 4pi * round((M_PI * 2 - soldAngle) / 4pi) + soldAngle;
	solidAngle = std::abs(solidAngle);
	PositionState position = PositionState::UnKnown;
	bool bReverse = shell == myShells[1] && myReversedShellBFaces;
	if (solidAngle > -M_PI && solidAngle < M_PI) {
		position = bReverse ? PositionState::In : PositionState::Out;
	}else if (solidAngle > M_PI * 3 && solidAngle < M_PI * 5) {
		position = bReverse ? PositionState::Out : PositionState::In;
	}else{
		position =  PositionState::UnKnown;
	}

	return position;
}
 
void Boolean::BuildConnexityBlock() {
	auto shell = std::make_shared<Shell>();
	shell->AddCoFaces(std::move(myFaces));
	shell->BuildBackwardReference();
	const vector<vector<int>>& groups = GetGroups(shell);
	const vector<vector<int>>& validClassification = ClassificationFaces(shell);
	myRet.reserve(myFaces.size());

	for (int i = 0; i < groups.size(); i++){
		auto group = groups[i];
		bool validFaces = std::any_of(group.begin(), group.end(), [&validClassification](int e) {
			return std::binary_search(validClassification[0].begin(), validClassification[0].end(), e);
		});
		bool unValidFaces = std::any_of(group.begin(), group.end(), [&validClassification](int e) {
			return std::binary_search(validClassification[1].begin(), validClassification[1].end(), e);
		});
	
		if (validFaces && !unValidFaces) {
			myRet.insert(myRet.end(), group.begin(), group.end());
		}
		else if (!validFaces && unValidFaces) {
			//
		}
		else if (validFaces && unValidFaces) {
			myRet.clear();
			return;
		}
		else {
			// !validFaces && !unValidFaces, IN / OUT / ON
			bool bSameDirNormal = true;
			if (myIsPloyHendral && IsGroupOn(group, groups,shell, bSameDirNormal)) {
				//Position ON
				bool bOnValid = GroupValidition(PositionState::On, myTp, bSameDirNormal, isShellAGroup(group));
				if (bOnValid) {
					myRet.push_back(group[0]);
				}
			}
			else {
				//Position IN / OUT
				PositionState position = GroupShellPosition(group, shell);
				bool bValid = GroupValidition(position, myTp, true, isShellAGroup(group));
				if (bValid) {
					myRet.insert(myRet.end(), group.begin(), group.end());
				}
			}
		}
	}
	auto& coFaces = shell->GetCoFacesWithoutConst();
	myFaces = std::move(coFaces);
}

static unordered_set<EdgePtr> GetEdges(const CoFacePtr& coface) {
	unordered_set<EdgePtr> edgeSet;

	const auto& wires = coface->GetFace()->GetWires();
	for (auto& wire : wires) {
		const auto& coedges = wire->GetCoEdges();
		for (const auto& coedge : coedges) {
			edgeSet.insert(coedge->GetEdge());
		}
	}
	return edgeSet;
}

int Boolean::FindMaxLeftTurningCoedge(const vector<CoEdgePtr>& coedges, int current, const vector<int>& candidates, int coincidenceCoedge) {
	int ret = -1;
	if (candidates.empty() && coincidenceCoedge != -1) {
		ret = coincidenceCoedge;
	}
	else if (candidates.size() == 1) {
		ret = candidates[0];
	}
	else {
		gp_Pnt2d P;
		gp_Vec2d Dir;
		gp_Vec Ref(0, 0, 1);
		double lastParam = coedges[current]->GetLastParamWithOrient();
		coedges[current]->GetPcurve()->D1(lastParam, P, Dir);
		Dir.Normalize();
		if (coedges[current]->GetSameOrient()) {
			Dir.Reverse();
		}

		double maxAngle = -100.0;
		for (auto co : candidates){
			double angle = 0.0;
			if (co == current) {
				angle = M_PI;
			}
			else {
				gp_Vec2d Dir2;
				double firstParam = coedges[co]->GetFirstParamWithOrient();
				coedges[co]->GetPcurve()->D1(firstParam, P, Dir2);
				Dir2.Normalize();
				if (!coedges[co]->GetSameOrient()) {
					Dir2.Reverse();
				}
				gp_Dir d1(Dir.X(), Dir.Y(), 0);
				gp_Dir d2(Dir2.X(), Dir2.Y(), 0);
				angle = AngleWithRefRoughly(d1, d2, Ref);
			}

			if (angle > maxAngle) {
				ret = co;
				maxAngle = angle;
			}
		}
	}

	return ret;
}

bool Boolean::IsGroupOn(vector<int> group, vector<vector<int>> groups, const ShellPtr& shell, bool& bSameDirNormal) {
	//size == 1
	if (group.size() != 1) {
		return false;
	}

	unordered_set<EdgePtr> edgeSet;

	const auto& cofaces = shell->GetCoFaces();
	const auto& coface = cofaces[group[0]];
	const auto& wires = coface->GetFace()->GetWires();
	for (auto& wire : wires) {
		const auto& coedges = wire->GetCoEdges();
		for (const auto& coedge : coedges) {
			edgeSet.insert(coedge->GetEdge());
			bool bCoedgeOn = coedge->GetEdge()->GetCoEdges().size() >= 4;
			if (!bCoedgeOn) {
				return false;
			}
		}
	}

	//edgeSet: group all edges 
	int coFaceOnIndex = -1;
	for (auto groupOn : groups) {
		if (groupOn.size() == 1 && groupOn[0] != group[0]) {
			const auto& cofaceOn = cofaces[groupOn[0]];
			const auto coFaceOnEdges = GetEdges(cofaceOn);
			if (coFaceOnEdges.size() != edgeSet.size()) {
				continue;
			}

			bool sameSet = true;
			for (const auto& elem : edgeSet) {
				if (coFaceOnEdges.find(elem) == coFaceOnEdges.end()) {
					sameSet = false;
					break;
				}
			}
			if (sameSet) {
				coFaceOnIndex = groupOn[0];
			}
		}
	}

	//TODO: coedge0 may seam edge, if is seam edge, change another coedge

	if (coFaceOnIndex != -1) {
		const auto& coedge0 = wires[0]->GetCoEdges()[0];
		const auto& cofaceOn = cofaces[coFaceOnIndex];
		const auto& coedgesOn = cofaceOn->GetFace()->GetOuterWire()->GetCoEdges();
		auto it = std::find_if(coedgesOn.begin(), coedgesOn.end(), [&coedge0](const CoEdgePtr& coedge)
		             { return coedge->GetEdge() == coedge0->GetEdge(); });
		const auto& coedgeOn = *it;
		bSameDirNormal = (coedgeOn->GetSameOrient() == cofaceOn->GetSameOrient()) 
			== (coedge0->GetSameOrient() == coface->GetSameOrient());
		return true;

	}
	else {
		return false;
	}

}

PositionState Boolean::GroupShellPosition(std::vector<int> &group, std::shared_ptr<Shell> &groupShell) {
	int otherShell = isShellAGroup(group) ? 1 : 0;

	for (auto faceI : group) {
		auto& coface = groupShell->GetCoFaces()[faceI];
		auto& wires = coface->GetFace()->GetWires();
		for (auto& wire : wires) {
			auto& coedges = wire->GetCoEdges();
			for (auto& coedge : coedges) {
				auto& twins = coedge->GetEdge()->GetCoEdges();
				vector<CoedgeIndex*> otherShellCoedge;
			    if (twins.size() == 4) {
					for (auto& twin : twins) {
						int shellIndex = twin.myFaceIndex < myFacesASize ? 0 : 1;
						if (shellIndex == otherShell) {
							otherShellCoedge.push_back(&twin);
						}
					}
					CoedgeIndex* coedge0Index = otherShellCoedge[0];
					CoedgeIndex* coedge1Index = otherShellCoedge[1];
					const CoFacePtr& coface0 = groupShell->GetCoFaces()[coedge0Index->myFaceIndex];
					const CoFacePtr& coface1 = groupShell->GetCoFaces()[coedge1Index->myFaceIndex];
					const CoEdgePtr& coedge0 = coface0->GetFace()->GetWires()[coedge0Index->myWireIndex]
						->GetCoEdges()[coedge0Index->myCoedgeIndex];
					const CoEdgePtr& coedge1 = coface1->GetFace()->GetWires()[coedge1Index->myWireIndex]
						->GetCoEdges()[coedge1Index->myCoedgeIndex];

					PositionState position = FaceInOutStatus(coface, coedge, coface0, coedge0, coface1, coedge1);
					if (position != PositionState::UnKnown) {
						return position;
					}
				}
			}
		}
	}

	for (auto faceI : group) {
		auto& wires = groupShell->GetCoFaces()[faceI]->GetFace()->GetWires();
		for (auto& wire : wires) {
			auto& coedges = wire->GetCoEdges();
			for (auto& coedge : coedges) {
				if (coedge->GetEdge()->GetCoEdges().size() == 2) {
					auto& curve = coedge->GetEdge()->GetCurve();
					gp_Pnt pt;
					double param = 0.5 * (coedge->GetFirstParam() + coedge->GetLastParam());
					curve->D0(param, pt);
					PositionState position = IsPointInPolyHendralWithoutOn(pt, myShells[otherShell]);
					if (position != PositionState::UnKnown) {
						return position;
					}
				}
			}
		}
	}
	// TODO
	return PositionState::UnKnown;
}

PositionState Boolean::EdgeOnFacePosition(CoEdge * ce1, CoEdge * ce2, const gp_Dir2d& d32d) {
	double angleCe1ToCe2 = 0.0;
	double angleCe1ToLine2d = 0.0;

	auto dir1 = Handle(Geom2d_Line)::DownCast(ce1->GetPcurve())->Direction();
	if (ce1->GetSameOrient()) {
		dir1.Reverse();
	}
    gp_Dir d1(dir1.X(), dir1.Y(), 0);
	gp_Dir d3(d32d.X(), d32d.Y(), 0);
	gp_Dir ref(0, 0, 1);
	angleCe1ToCe2 = AngleWithRefRoughly(d1, d3, ref);
	if (ce1 == ce2) {
		angleCe1ToLine2d = M_PI;
	}
	else {
		auto dir2 = Handle(Geom2d_Line)::DownCast(ce2->GetPcurve())->Direction();
		if (!ce2->GetSameOrient()) {
			dir2.Reverse();
		}
		gp_Dir d2(dir2.X(), dir2.Y(), 0);
		angleCe1ToLine2d = AngleWithRefRoughly(d1, d2,ref);
	}

	double angleCalTol = 1.0e-6;
	PositionState position = PositionState::UnKnown;
	if (std::abs(angleCe1ToCe2) < angleCalTol || std::abs(std::abs(angleCe1ToCe2) - M_PI * 2) < angleCalTol) {
		position = PositionState::UnKnown;
	}
	else if (std::abs(angleCe1ToLine2d - angleCe1ToCe2) < angleCalTol) {
		position = PositionState::UnKnown;
	}
	else if (angleCe1ToCe2 > angleCe1ToLine2d) {
		position = PositionState::In;
	}
	else {
		position = PositionState::Out;
	}
	return position;
}

double Boolean::AngleWithRefRoughly(const gp_Dir& theD1, const gp_Dir& theD2, const gp_Dir& theDRef) {
	double aCosinus, aSinus, aBeta, aHalfPI, aScPr;
	gp_XYZ aXYZ;
	//
	aHalfPI = 0.5*M_PI;
	//
	const gp_XYZ& aXYZ1 = theD1.XYZ();
	const gp_XYZ& aXYZ2 = theD2.XYZ();
	aXYZ = aXYZ1.Crossed(aXYZ2);
	aSinus = aXYZ.Modulus();
	aCosinus = theD1 * theD2;
	//
	aBeta = 0.;
	if (aSinus >= 0.) {
		aBeta = aHalfPI * (1. - aCosinus);
	}
	else {
		aBeta = 2.*M_PI - aHalfPI * (3. + aCosinus);
	}
	//
	aScPr = aXYZ.Dot(theDRef.XYZ());
	if (aScPr < 0.) {
		aBeta = -aBeta;
	}
	if (aBeta < 0.0) {
		aBeta += 2.*M_PI;
	}
	return aBeta;
}

void Boolean::BuildShell() {
	auto shell = make_shared<Shell>();
	for (auto index : myRet) {
		shell->AddCoface(std::move(myFaces[index]));
	}
	myRetShells.push_back(shell);
}

void Boolean::Prepare() {
	if (myTp == BooleanType::SUBTRACT) {
		myShells[1] = myShells[1]->Reversed();
		myReversedShellBFaces = true;
		myTp = BooleanType::INTERSECT;
	}

	GetIntersectionPairs();

	const auto& cofacesA = myShells[0]->GetCoFaces();
	const auto& cofacesB = myShells[1]->GetCoFaces();
	myIsPloyHendral =  
		std::all_of(cofacesA.begin(), cofacesA.end(), [](const CoFacePtr& coface) {
			return coface->GetFace()->GetSurface()->DynamicType() == STANDARD_TYPE(Geom_Plane);}) &&
		std::all_of(cofacesB.begin(), cofacesB.end(), [](const CoFacePtr& coface) {
			return coface->GetFace()->GetSurface()->DynamicType() == STANDARD_TYPE(Geom_Plane);});
}

//
Standard_EXPORT bool Boolean::Perform() {

	Prepare();

	PerformVV();

	if (!PerformVE()) {
		return false;
	}

	PerformEE();

	PerformVF();

	if (!PerformEF()) {
		return false;
	}

	PerformFF();

	BuildEdge();

	BuildCoEdge();

	if (!BuildLoop()) {
		return false;
	}

	BuildFace();

	BuildConnexityBlock();

	BuildShell();

	return true;
}
