#include "./FR_RecognizeBaseFaces.h"

// FR includes
#include "./FR_AAGIterator.h"
#include "./FR_AAG.h"

// OpenCascade includes
#include <gp_Cylinder.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>

//-----------------------------------------------------------------------------

FR_RecognizeBaseFaces::FR_RecognizeBaseFaces(const Handle(FR_AAG)& aag)
//
: FR_Recognizer (aag)
{}

//-----------------------------------------------------------------------------

bool FR_RecognizeBaseFaces::Perform()
{
    m_result.Clear();
    const TopoDS_Shape shape = m_aag->GetMasterShape();
    // Loop over the AAG.
    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        // Get face to check the number of wires.
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());

        // Get the number of wires.
        TopTools_IndexedMapOfShape wires;
        TopExp::MapShapes(face, TopAbs_WIRE, wires);
        //
        const int numWires = wires.Extent();

        if (numWires > 1)
        {
            m_result.faces.Add(face);
            m_result.ids.Add(m_aag->GetFaceId(face));
        }
    }

  return true;
}
