
#ifndef _TOPO_HeaderFile
#define _TOPO_HeaderFile

#include <Standard_Handle.hxx>
#include <gp_Pnt.hxx>
#include<vector>
#include<memory>
#include "../Common/Extension_Export.hxx"

// TODO: plane direction = -1?
// V, E, CE ,W, F, CF, S

// Compared to OCCT's Brep，
// 1. The child entities store references to the parent entities
// 2. The entity stores references to its next and prior entities at its own level.
// 3. Remove the TopLoc_Location property to improve code readability and maintainability.
//    (It is possible to associate transformations with any topological primitive, although it
//    is strongly advised not to use them on primitives other than "bodies".)
// 4. Remove the Orientation of V and W.


//   *=============OCCT==========================BREP===============* 
//   |  TopoDS_TVertex + TopoDS_Vertex  |       Vertex              | 
//   |--------------------------------------------------------------|
//   |  TopoDS_TEdge + TopoDS_Edge      |    Edge + CoEdge          |
//   |--------------------------------------------------------------|
//   |  TopoDS_TWire + TopoDS_Wire      |        Wire               |
//   |--------------------------------------------------------------|
//   |  TopoDS_TFace + TopoDS_Face      |    Face + CoFace          |
//   *==============================================================*

//Topo
//         not support                  support                 support
//   *---------------------*    *---------------------*    *---------*
//   |                     |    |                     |    |         |
//   |                     |    |                     |    |         |
//   |                     |    |                     |    *---------*---------*
//   |      --------       |    |---------            |              |         |
//   |                     |    |                     |              |         |
//   |                     |    |                     |              *---------* 
//   |                     |    |                     |
//   *---------------------*    *---------------------*
    
class Geom2d_Curve;
class Geom_Curve;
class Geom_Surface;
class Shell;
#define distTol 0.001

class Vertex;
class Edge;
class Face;
class Shell;
class CoEdge;
class Wire;
class CoFace;

#define VertexPtr std::shared_ptr<Vertex>
#define EdgePtr std::shared_ptr<Edge>
#define FacePtr std::shared_ptr<Face>
#define ShellPtr std::shared_ptr<Shell>

#define CoEdgePtr std::shared_ptr<CoEdge>
#define WirePtr std::shared_ptr<Wire>
#define CoFacePtr std::shared_ptr<CoFace>

#define CurvePtr Handle(Geom_Curve)
#define Curve2dPtr Handle(Geom2d_Curve)
#define SurfacePtr Handle(Geom_Surface)

enum EdgeFlag{
     ParameterMask = 1,
     RangeMask = 2,
     DegeneratedMask = 3,
     VisitedFlag = 4,
     BoxCalcFlag = 5,
     SEAM = 6, // SEAM Edge
	 INTERNAL_TEMP = 7, // temp use for coedge INTERNAL
	 IsIntersectionEdgeFlag = 8, // 布尔运算使用到, 面面求交产生的交线边
	 IsUnValidEdgeFlag = 9, // 1: unvalid  0: unknown
	 IsValidEdgeFlag = 10, // 1: valid  0: unknown
};

enum FaceFlag {
	FaceVisitedFlag = 1,
	NaturalRestriction = 2
};

struct Box{
	double m_min[3];
	double m_max[3];

	Box() {
		m_min[0] = DBL_MAX;
		m_min[1] = DBL_MAX;
		m_min[2] = DBL_MAX;
		m_max[0] = -DBL_MAX;
		m_max[1] = -DBL_MAX;
		m_max[2] = -DBL_MAX;
	}

	void Add(const Box& Other) {
		if (m_min[0] > Other.m_min[0]) m_min[0] = Other.m_min[0];
		if (m_max[0] < Other.m_max[0]) m_max[0] = Other.m_max[0];
		if (m_min[1] > Other.m_min[1]) m_min[1] = Other.m_min[1];
		if (m_max[1] < Other.m_max[1]) m_max[1] = Other.m_max[1];
		if (m_min[2] > Other.m_min[2]) m_min[2] = Other.m_min[2];
		if (m_max[2] < Other.m_max[2]) m_max[2] = Other.m_max[2];
	}
};

struct CoedgeIndex{

	public:
		CoedgeIndex(size_t myFaceIndex, size_t myWireIndex, size_t myCoedgeIndex) {
			this->myFaceIndex = myFaceIndex;
			this->myWireIndex = myWireIndex;
			this->myCoedgeIndex = myCoedgeIndex;
		}

		size_t myFaceIndex;
		size_t myWireIndex;
		size_t myCoedgeIndex;
};

 class Vertex {
	public:
		Vertex() : myFlags(0), myTolerance(distTol), myIndexInShell(-1){}

		Vertex(const gp_Pnt& pnt, double tol) : myPnt(pnt), myTolerance(tol), myFlags(0), myIndexInShell(-1){}

		const gp_Pnt& GetPnt() const;

		void SetPnt(const gp_Pnt& P);

		void SetTol(const double T);

		double GetTol() const;

		std::vector<size_t>& GetEdgeIndexs() const { return myEdges; }

		//! Set bit flag.
		void SetFlag(int theFlag, bool theIsOn);

		void SetVisited() { myFlags = myFlags | (1 << VisitedFlag); };

		bool IsVisited() { return (myFlags >> VisitedFlag & 1); };

		void CleanVisited() { myFlags = myFlags & ~(1 << VisitedFlag); };

		void AddEdgeIndex(size_t edgeIndex);

		size_t GetVertexIndex() { return this->myIndexInShell; };

		void SetVertexIndex(size_t index) { this->myIndexInShell = index; };

 private:
		gp_Pnt myPnt;
		double myTolerance;
		int  myFlags;
		//cache: edge index in shell
		mutable std::vector<size_t> myEdges;
		mutable size_t myIndexInShell;
};

 class Edge{
	public:
		//! Creates an empty Edge.
		Edge() : myFlags(0), myTolerance(distTol), myFirstParam(0), myLastParam(0){};

		Edge(const CurvePtr& curve, const VertexPtr& first,
			const VertexPtr& last, double firstParam, double lastParam)
			: myCurve(curve), myFirst(first), myLast(last),
			myFirstParam(firstParam), myLastParam(lastParam), myFlags(0), myTolerance(distTol) {};

		double GetTolerance() const;

		void SetTolerance(const double T);

		bool SameRange() const;

		void SameRange(const bool S);

		bool SameParameter() const;

		void SameParameter(const bool S);

		bool Degenerated() const;

		void Degenerated(const bool S);

		const Handle(Geom_Curve)& GetCurve() const;

		void SetCurve(Handle(Geom_Curve)& C);

		const VertexPtr& GetFirstVertex() const;

		void SetFirstVertex(const VertexPtr& val) { myFirst = val; }

		const VertexPtr& GetLastVertex() const;

		void SetLastVertex(const VertexPtr& val) { myLast = val; }

		double GetFirstParam() const { return myFirstParam; }

		void SetFirstParam(double val) { myFirstParam = val; }

		double GetLastParam() const { return myLastParam; }

		void SetLastParam(double val) { myLastParam = val; }

		std::vector<CoedgeIndex>& GetCoEdges() { return myCoEdges; }

		void AddCoedgeIndex(CoedgeIndex val) { myCoEdges.emplace_back(val); }

		void SetVisited() { SetFlag(EdgeFlag::VisitedFlag); };

		bool IsVisited() { return GetFlag(EdgeFlag::VisitedFlag); };

		void CleanVisited() { CleanFlag(EdgeFlag::VisitedFlag); };

		const Box& GetOrCalcBox();

		void SetFlag( EdgeFlag flag) {
			myFlags = myFlags | (1 << flag);;
		}

		bool GetFlag( EdgeFlag flag) {
			return myFlags >> flag & 1;
		}

		void CleanFlag(EdgeFlag flag) {
			myFlags = myFlags & ~(1 << flag);
		}

 private:
	    VertexPtr myFirst;
	    VertexPtr myLast;
		Handle(Geom_Curve) myCurve;
		Box myBox;
		double myFirstParam;
		double myLastParam;
		double myTolerance;
		int myFlags;

		//cache: faceIndex， loopIndex，coEdgeIndex
		mutable std::vector<CoedgeIndex> myCoEdges;
};

 class CoEdge {
	public:
		CoEdge(): myFirstParam(0), myLastParam(0), mySameOrient(true){}

		CoEdge(const EdgePtr& edge, Handle(Geom2d_Curve) pcurve,
		double firstParam, double lastParam, bool sameOrient): myEdge(edge), myPcurve(pcurve), 
			myFirstParam(firstParam), myLastParam(lastParam), mySameOrient(sameOrient) {}

		bool GetSameOrient() const;

		void SetSameOrient(bool sameDir);

		const EdgePtr& GetEdge() const;

		void SetEdge(const EdgePtr& edge) ;

		const Handle(Geom2d_Curve)& GetPcurve() const;

		void SetPcurve(Handle(Geom2d_Curve) pCurve);

		double GetFirstParam() const { return myFirstParam; }

		double GetFirstParamWithOrient() const { return mySameOrient ? myFirstParam : myLastParam; }
		double GetLastParamWithOrient() const { return mySameOrient ? myLastParam : myFirstParam; }

		void SetFirstParam(double val) { myFirstParam = val; }

		double GetLastParam() const { return myLastParam; }

		void SetLastParam(double val) { myLastParam = val; }

		const VertexPtr GetFirstVertexWithOrient() const { return mySameOrient? myEdge->GetFirstVertex() : myEdge->GetLastVertex(); }

		const VertexPtr GetLastVertexWithOrient()  const{ return mySameOrient ? myEdge->GetLastVertex() : myEdge->GetFirstVertex(); }

		const std::vector<CoedgeIndex> GetTwins(const ShellPtr& shell) const;

		const CoedgeIndex GetTwin(const ShellPtr& shell) const;

 private:
	    EdgePtr myEdge;
		Handle(Geom2d_Curve) myPcurve;
		double myFirstParam;
		double myLastParam;
		bool mySameOrient;
 };

 class Wire {
	public:
		Wire();

		void ReserveCoedgeSize(int size) { myCoEdges.reserve(size); }

		const std::vector<CoEdgePtr>& GetCoEdges() const;

		void inline AddCoedge(CoEdgePtr&& coedge) {
			this->myCoEdges.push_back(std::move(coedge));
		};

		const std::vector<VertexPtr>& GetVertices() const{
			if (myVertices.empty()) {
				for (const auto& coedge : myCoEdges) {
					myVertices.push_back(coedge->GetFirstVertexWithOrient());
				}
			}
			return myVertices;
		};

	private:
		std::vector<CoEdgePtr> myCoEdges;
		mutable std::vector<VertexPtr> myVertices; // cache
};

 class Face {
	public:
		//! Creates an empty TFace.
		Face() : myFlags(0), myTolerance(distTol) {};

		const Handle(Geom_Surface)& GetSurface() const;

		double CetTolerance() const;

		void SetSurface(const Handle(Geom_Surface)& S);

		void SetTolerance(const double T);

		double GetTolerance() { return myTolerance; };

		const WirePtr& GetOuterWire() const;

		const std::vector<WirePtr>& GetWires() const;

		void AddWire(WirePtr&& wire){
			myWires.push_back(std::move(wire));
		}

		const Box& GetOrCalcBox();

		const std::vector<CoEdge*> GetCoedges() const;

		void SetFlags(int flag) {
			this->myFlags = flag;
		}

		int GetFlags() {
			return this->myFlags;
		}

 private:
		std::vector<WirePtr> myWires;
		Handle(Geom_Surface) mySurface;
		Box myBox;
		double myTolerance;
		int myFlags;
};

class CoFace {
	public:

		CoFace():mySameOrient(true){}

		CoFace(const FacePtr& myFace, bool mySameOrient){ 
			this->myFace = myFace; 
			this->mySameOrient = mySameOrient;
		}

		const FacePtr& GetFace() const;

		bool GetSameOrient() const;

		void SetSameOrient(bool sameOrient) {
			this->mySameOrient = sameOrient;
		}

		void SetVisited() { myFace->SetFlags(myFace->GetFlags() | (1 << FaceVisitedFlag)); };

		bool IsVisited() { return (myFace->GetFlags() >> FaceVisitedFlag & 1); };

		void CleanVisited() { myFace->SetFlags(myFace->GetFlags() & ~(1 << FaceVisitedFlag)); };

	private:
		FacePtr myFace;
		bool mySameOrient;
};

class Shell {
public:
	Shell() : myFlags(0) {}

	EXTENSION_EXPORT const std::vector<CoFacePtr>& GetCoFaces() const;

	std::vector<CoFacePtr>& GetCoFacesWithoutConst();

	ShellPtr Reversed();

	void inline AddCoface(CoFacePtr&& coFace) {
		this->myCoFaces.push_back(std::move(coFace));
	}

	void AddCoFaces(std::vector<CoFacePtr>&& coFaces) {
		this->myCoFaces = std::move(coFaces);
	}

	EXTENSION_EXPORT void BuildBackwardReference() const;

	EXTENSION_EXPORT const std::vector<EdgePtr>& GetEdges() const {
		return myEdges;
	}

	EXTENSION_EXPORT const std::vector<VertexPtr>& GetVertices() const {
		return myVertices;
	}

	void ReserveCoFaceSize(int size) { myCoFaces.reserve(size); }

private:
	std::vector<CoFacePtr> myCoFaces;
	mutable std::vector<EdgePtr> myEdges;//cache
	mutable std::vector<VertexPtr> myVertices;//cache
	int myFlags;
};
#endif
