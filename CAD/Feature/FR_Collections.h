
#ifndef FR_Collections_h
#define FR_Collections_h

// FR includes
#include "./FR_ShapePartnerHasher.h"

// OpenCascade includes
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_DataMap.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

//! \ingroup ASI_CORE
//!
//! Indexed map of shapes distinguished only by their TShape pointers.
typedef NCollection_DataMap<int, TopoDS_Shape> FR_DataMapOfShape;

//! \ingroup ASI_CORE
//!
//! Indexed map of shapes distinguished only by their TShape pointers.
typedef NCollection_IndexedMap<TopoDS_Shape,
                               FR_ShapePartnerHasher> FR_IndexedMapOfTShape;

//! \ingroup ASI_CORE
//!
//! Indexed data map of shapes and lists of shapes with the keys distinguished only by their
//! TShape pointers.
typedef NCollection_IndexedDataMap<TopoDS_Shape,
                                   TopTools_ListOfShape,
                                   FR_ShapePartnerHasher> FR_IndexedDataMapOfTShapeListOfShape;

//! \ingroup ASI_CORE
//!
//! Indexed data map of shapes and maps of shapes with the keys distinguished only by their
//! TShape pointers.
typedef NCollection_IndexedDataMap<TopoDS_Shape,
                                   TopTools_IndexedMapOfShape,
                                   FR_ShapePartnerHasher> FR_IndexedDataMapOfTShapeIndexedMapOfShape;

//! \ingroup ASI_CORE
//!
//! Data map of shapes and their images distinguished only by their
//! TShape pointers.
typedef NCollection_DataMap<TopoDS_Shape, TopoDS_Shape,
                            FR_ShapePartnerHasher> FR_DataMapOfShapeShape;

#endif
