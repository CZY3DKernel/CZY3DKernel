
#ifndef _BOOLEAN_HeaderFile
#define _BOOLEAN_HeaderFile
#include "Topo.hxx"
#include <BOPAlgo_GlueEnum.hxx>
#include <BRepTools_History.hxx>
#include <Bnd_Box.hxx>
#include <Geom_Line.hxx>
#include <Geom_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <list>
#include "../Common/Extension_Export.hxx"

using namespace std;
class Geom_Plane;
class Geom_Line;
class Geom2d_Line;

enum BooleanType {
	INTERSECT,
	UNION,
	SUBTRACT,
	UNKNOWN
};

enum PositionState {
	In,
	On,
	Out,
	UnKnown
};

class IntersectionPair
{
	// 0: shellIndex
	// 1: V/E/F -> 0/1/2
	// 2: index
public:
	size_t first[3];
	size_t last[3];
};
//
class Pave {
public:
	Pave(const VertexPtr& v, double par) : vertex(v), param(par), edgeIndex(-1) {}
	VertexPtr vertex;
	double param;
	int edgeIndex;

	bool operator < (const Pave& theOther) const
	{
		return param < theOther.param;
	}
};

class PaveOnIntersectionCurve{
public:
	double param;
	PositionState left;
	PositionState right;

	PaveOnIntersectionCurve(double param, const PositionState& left, const PositionState& right) {
		this->param = param;
		this->left = left;
		this->right = right;
	}
};

class PaveBlock {
public:

	PaveBlock(Pave* first, Pave* last) {
		myFirst = first;
		myLast = last;
		myCommonBlockIndex = -1;
		myPaveBlockOnFaceIndex = -1;
		edge = nullptr;
	}

	Pave* myFirst;
	Pave* myLast;
	vector<Pave*> mySplitPaves;
	int myCommonBlockIndex; // Boolean::myCommonBlocks
	int myPaveBlockOnFaceIndex; //Boolean::myEdgeOnFaceBoolInfos
	EdgePtr edge;

	~PaveBlock() {
	}
};


class OriginEdgeBoolInfo {
public:
	list<unique_ptr<Pave>> paves;
	vector<unique_ptr<PaveBlock>> myPaveBlocks;
};


class SectionEdgeBoolInfo {
public:
	//int  -1 go out, +1 go into, 0 unknown
	//double param on myC3
	// coedge0(sameOrition = true) on A   
	// coedge1(sameOrition = false)  on A 
	// coedge2(sameOrition = true)  on B  
	// coedge3(sameOrition = false)  on B 
	vector<pair<VertexPtr, PaveOnIntersectionCurve>> myVertexs;
	Handle(Geom_Curve) myC3;
	Handle(Geom2d_Curve) myC2A;
	Handle(Geom2d_Curve) myC2B;
	vector<EdgePtr> myEdges;
	int myFaceAIndex;
	int myFaceBIndex;
	//-1:coedge0Unvalid  0:coedge0Unknown  1:coedge0Valid
	int isCoedge0Valid;

	SectionEdgeBoolInfo(vector<pair<VertexPtr,PaveOnIntersectionCurve>>&& vertexs,
		const Handle(Geom_Curve)& c3,
		const Handle(Geom2d_Curve)& c2A,
		const Handle(Geom2d_Curve)& c2B,
		int faceAIndex, int faceBIndex) : 
		myVertexs(std::move(vertexs)),
		myEdges(myVertexs.size()-1, nullptr),
		myC3(c3), myC2A(c2A), myC2B(c2B),
		myFaceAIndex(faceAIndex), myFaceBIndex(faceBIndex), isCoedge0Valid(1){
	}
};

class EdgeOnFaceBoolInfo {
public:
	Handle(Geom2d_Curve) pCurve;//on Face
	EdgePtr edge; // origin edge after split

	EdgeOnFaceBoolInfo(const Handle(Geom2d_Curve) pCurve) : pCurve(pCurve) {}
};

//
class BoolFaceInfo {

public:
	BoolFaceInfo() { }
public:
	vector<SectionEdgeBoolInfo*> mySectionEdgeBoolInfos;
	vector<EdgeOnFaceBoolInfo*> myEdgeOnFaceBoolInfos;

	vector<CoEdgePtr> myAllCoedges;
	vector<WirePtr> myWires;

	vector<VertexPtr> myVerticesOnEdge;
	vector<VertexPtr> myVerticesOnFace;
	vector<VertexPtr> myVerticesSection;
	vector<VertexPtr> myAllVertices;

	const vector<VertexPtr>& AllVertices();

public:
	bool ContainsVertex(const VertexPtr& vertex) {
		return std::find(myVerticesOnEdge.begin(), myVerticesOnEdge.end(), vertex) != myVerticesOnEdge.end() ||
			std::find(myVerticesOnFace.begin(), myVerticesOnFace.end(), vertex) != myVerticesOnFace.end() ||
			std::find(myVerticesSection.begin(), myVerticesSection.end(), vertex) != myVerticesSection.end();
	}
};

struct VertexPtrHash {
	size_t operator()(const VertexPtr& vertex) const {
		return reinterpret_cast<std::size_t>(vertex.get());
	}
};

struct VertexPtrEqual {
	bool operator()(const VertexPtr& lhs, const VertexPtr& rhs) const {
		return lhs == rhs;
	}
};

struct EdgePtrHash {
	size_t operator()(const EdgePtr& edge) const {
		size_t t = reinterpret_cast<std::size_t>(edge.get());
		return t;
	}
};

struct EdgePtrEqual {
	bool operator()(const EdgePtr& lhs, const EdgePtr& rhs) const {
		return lhs == rhs;
	}
};

struct FacePtrHash {
	size_t operator()(const FacePtr& face) const {
		return reinterpret_cast<std::size_t>(face.get());
	}
};

struct FacePtrEqual {
	bool operator()(const FacePtr& lhs, const FacePtr& rhs) const {
		return lhs == rhs;
	}
};

class Boolean {
public:
	EXTENSION_EXPORT Boolean() {
		myReversedShellBFaces = false;
		myFuzzyValue = 1.0e-3;
		myRunParallel = false;
		myIsPloyHendral = false;
		myFacesASize = -1;
		myGlue = BOPAlgo_GlueOff;
		myTp = UNKNOWN;
	}

	EXTENSION_EXPORT Boolean(ShellPtr shellA, ShellPtr shellB, BooleanType tp, double fuzzyValue) {
		myShells[0] = shellA;
		myShells[1] = shellB;
		myTp = tp;
		myFuzzyValue = fuzzyValue;

		myReversedShellBFaces = false;
		myRunParallel = false;
		myIsPloyHendral = false;
		myFacesASize = -1;
		myGlue = BOPAlgo_GlueOff;
	}

	EXTENSION_EXPORT bool Perform();

	EXTENSION_EXPORT const vector<ShellPtr>& GetResultShells() const { return myRetShells; }


private:
	void Prepare();

	void GetIntersectionPairs();

	void PerformVV();

	bool PerformVE();

	void PerformEE();

	void PerformVF();

	bool PerformEF();

	void PerformFF();

	void BuildEdge();

	void BuildCoEdge();

	bool BuildLoop();

	void BuildFace();

	void BuildConnexityBlock();

	void BuildShell();

	Box MakeBoxForVertex(const VertexPtr& V);

	bool SplitPaveBlocks(unique_ptr<OriginEdgeBoolInfo>& boolEdgeInfo);

	VertexPtr ComputeLineLine(const Handle(Geom_Line)& aLineA, const Handle(Geom_Line)& aLineB,
		const EdgePtr& eA, const EdgePtr& eB);

	bool PointInPolyhedronFaceWithoutOn(const gp_Pnt& pt, const FacePtr& face);

	bool CollectPavesForPlanPlaneIntresectLine(const CoFacePtr& fA, const CoFacePtr& fB,
		const Handle(Geom_Line)& aLine, vector<pair<VertexPtr, PaveOnIntersectionCurve>>& validVertices);

	void GetValidPavesFromFace(const CoFacePtr& fA,
		std::vector<PaveBlock*> &coincidencePaveBlock,
		const std::unordered_set <VertexPtr, VertexPtrHash, VertexPtrEqual>& commonVertex,
		unordered_map <VertexPtr, PaveOnIntersectionCurve, VertexPtrHash, VertexPtrEqual>& validVertexsSet,
		const Handle(Geom_Line) & aLine);

	unique_ptr<BoolFaceInfo>& GetBooleanFaceInfo(const FacePtr& face);

	unique_ptr<OriginEdgeBoolInfo>& GetBooleanEdgeInfo(const EdgePtr& edge);

	bool LineSegmentPlaneIntersect(const Handle(Geom_Line)& line, 
		const double startParam, const double endParam,
		const Handle(Geom_Plane)& plane, double& line_parameter);

	bool PlanePlaneIntersect(const Handle(Geom_Plane)& sA,
		const Handle(Geom_Plane)& sB, Handle(Geom_Line)& resultLine,
		Handle(Geom2d_Line)& pCurveA, Handle(Geom2d_Line)& pCurveB, bool bPlanACrossPlaneB);

	bool GroupValidition(PositionState pos, BooleanType tp, bool sameDirNormal, bool isShellA);

	vector<int> PopLoop(vector<int>& list, vector<CoEdgePtr>& coedges);

	PositionState IsPointInPolyHendralWithoutOn(const gp_Pnt& queriePt, const ShellPtr&  shell);

	bool IsPointInPolygonWithoutOn(double x, double y, vector<gp_Pnt2d>& vertices);

	VertexPtr makeVertexFromVV(const VertexPtr& va, const VertexPtr& vb);

	vector<int> GetCoFacesGroup(int iCoFace, const ShellPtr& shell);

	vector<vector<int>> GetGroups(const ShellPtr& shell);

	int IsCoFaceValidByIntersectionBoundery(const CoFacePtr& coface);

	vector<vector<int>> ClassificationFaces(const ShellPtr& shell);

	PositionState FaceInOutStatus(const CoFacePtr& face, const CoEdgePtr& coedge,
		const CoFacePtr& faceAdj0, const CoEdgePtr& coedge0,
		const CoFacePtr& faceAdj1, const CoEdgePtr& coedge1);

	PositionState EdgeOnFacePosition(CoEdge * ce1, CoEdge * ce2, const gp_Dir2d& d32d);

	PositionState GroupShellPosition(std::vector<int> &group, std::shared_ptr<Shell> &groupShell);

	bool FindPrevEdgeAndNextEdgeAccordingVertex(const FacePtr& face, const VertexPtr& v,
		CoEdge*& prevE, CoEdge*& nextE);

	double AngleWithRefRoughly(const gp_Dir& theD1, const gp_Dir& theD2, const gp_Dir& theDRef);

	bool isShellAGroup(const std::vector<int>& group) {
		return group[0] < myFacesASize;
	}

	int FindMaxLeftTurningCoedge(const vector<CoEdgePtr>& coedges, int current, const vector<int>& candidates, int coincidenceCoedge);

	bool IsGroupOn(vector<int> group, vector<vector<int>> groups, const ShellPtr& shell, bool& bSameDirNormal);

	public:
		ShellPtr myShells[2];//myShellA, myShellB
		BooleanType myTp;
		bool myReversedShellBFaces;
		bool myIsPloyHendral;
		vector<ShellPtr> myRetShells;

		// Options
		BOPAlgo_GlueEnum myGlue;   //!< Gluing mode management
		double myFuzzyValue;
		bool myRunParallel;
		Handle(BRepTools_History) myHistory;	

		std::vector<std::unique_ptr<IntersectionPair>> myTopoIntpair[6];
		std::unordered_map<VertexPtr, VertexPtr, VertexPtrHash, VertexPtrEqual> vertex2InfoMap;
		std::unordered_map<EdgePtr, std::unique_ptr<OriginEdgeBoolInfo>, EdgePtrHash, EdgePtrEqual> edge2InfoMap;
		std::unordered_map<FacePtr, std::unique_ptr<BoolFaceInfo>, FacePtrHash, FacePtrEqual> face2InfoMap;

		std::vector<std::vector<PaveBlock*>> myCommonBlocks;
		std::vector<std::unique_ptr<EdgeOnFaceBoolInfo>> myEdgeOnFaceBoolInfos;
		std::vector<std::unique_ptr<SectionEdgeBoolInfo>> mySectionEdgeBoolInfos;

		std::vector<CoFacePtr> myFaces;
		vector<int> myRet;
		int myFacesASize;
};

#endif
