#include "./FR_CheckValidity.h"

// FR includes
//#include <FR_MeshCheckTopology.h>
//#include <FR_MeshInfo.h>
//#include "./FR_Utils.h"

// OCCT includes
//#include <Bnd_Box2d.hxx>
//#include <BndLib_Add2dCurve.hxx>
//#include <BRep_Builder.hxx>
//#include <BRep_Tool.hxx>
//#include <BRepAdaptor_Surface.hxx>
//#include <BRepCheck_Analyzer.hxx>
//#include <BRepClass3d_SolidClassifier.hxx>
//#include <BRepMesh_Edge.hxx>
//#include <Geom_BSplineCurve.hxx>
//#include <Geom2dAdaptor_Curve.hxx>
//#include <Geom2dInt_GInter.hxx>
//#include <IntRes2d_Domain.hxx>
//#include <ShapeAnalysis_Edge.hxx>
#include <ShapeAnalysis_ShapeTolerance.hxx>
//#include <TopExp.hxx>
//#include <TopExp_Explorer.hxx>
//#include <TopoDS.hxx>
//#include <TopoDS_Edge.hxx>
//#include <TopTools_DataMapOfShapeListOfShape.hxx>
//#include <TopTools_IndexedMapOfShape.hxx>
//#include <TopTools_ListOfShape.hxx>

//-----------------------------------------------------------------------------
// Auxiliary functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

double FR_CheckValidity::MaxTolerance(const TopoDS_Shape& shape)
{
  ShapeAnalysis_ShapeTolerance TolerChecker;
  const double maxToler = TolerChecker.Tolerance(shape, 1); // 1 means max

  return maxToler;
}
