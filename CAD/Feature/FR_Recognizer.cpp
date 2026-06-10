#include "./FR_Recognizer.h"

// FR includes
#include "./FR_AAG.h"

//-----------------------------------------------------------------------------

FR_Recognizer::FR_Recognizer(const TopoDS_Shape& masterCAD,
                                       const Handle(FR_AAG) & aag)
    //
    : m_master(masterCAD), m_aag(aag)
{
}

//-----------------------------------------------------------------------------

FR_Recognizer::FR_Recognizer(const Handle(FR_AAG)& aag)
//
: m_master          ( aag->GetMasterShape() ),
  m_aag             ( aag )
{}

//-----------------------------------------------------------------------------

FR_Recognizer::FR_Recognizer()
{}

//-----------------------------------------------------------------------------

FR_Recognizer::~FR_Recognizer()
{}

//-----------------------------------------------------------------------------

void FR_Recognizer::Init(const TopoDS_Shape& masterCAD)
{
  // Clean up and reinitialize inputs.
  m_master = masterCAD;
  m_aag.Nullify(); // AAG is not relevant anymore.

  // Clean up result.
  m_result.Clear();
}

//-----------------------------------------------------------------------------

bool FR_Recognizer::Classify(FR_Feature& feature,
                                  const bool       indicateProgress)
{
  // Contract check.
  if ( m_result.ids.IsEmpty() )
  {
    return true; // Nothing to classify.
  }

  // Narrow down the search space.
  Handle(FR_AAG) G = m_aag;
  //
  G->PushSubgraph(m_result.ids);
  {
    std::vector<FR_Feature> ccomps;
    G->GetConnectedComponents(ccomps);

    if ( indicateProgress )
    {
      //m_progress.SetMessageKey("Searching for isomorphisms");
      //m_progress.Init( ccomps.size() == 1 ? INT_MAX : int( ccomps.size() ) );
    }

    // Iterate over the connected components.
    for (auto cit = ccomps.cbegin();
         cit != ccomps.cend() /* && !m_progress.IsCancelling() */ ;
          ++cit )
    {
      const FR_Feature& featureFids = *cit;

      // Recognize features in each connected component separately.
      G->PushSubgraph(featureFids);
      {
        this->matchConnectedComponent(G, feature);
      }
      G->PopSubgraph(); // Pop connected component.

      if (indicateProgress)
      {
          //m_progress.StepProgress(1);
      }
    } // Loop for connected components.
  }
  G->PopSubgraph(); // Pop cylindrical holes.


  if (indicateProgress)
  {
      //m_progress.SetProgressStatus(ActAPI_ProgressStatus::Progress_Succeeded);
  }

  return true;
}

//-----------------------------------------------------------------------------

TopTools_IndexedMapOfShape& FR_Recognizer::ChangeResultFaces()
{
  return m_result.faces;
}

//-----------------------------------------------------------------------------

const TopTools_IndexedMapOfShape& FR_Recognizer::GetResultFaces() const
{
  return m_result.faces;
}

//-----------------------------------------------------------------------------

FR_Feature& FR_Recognizer::ChangeResultIndices()
{
  return m_result.ids;
}

//-----------------------------------------------------------------------------

const FR_Feature& FR_Recognizer::GetResultIndices() const
{
  return m_result.ids;
}

//-----------------------------------------------------------------------------

const Handle(FR_AAG)& FR_Recognizer::GetAAG() const
{
  return m_aag;
}

//-----------------------------------------------------------------------------

void FR_Recognizer::SetAAG(const Handle(FR_AAG)& aag)
{
  m_aag = aag;
}

//-----------------------------------------------------------------------------

void FR_Recognizer::matchConnectedComponent(const Handle(FR_AAG)&,
                                                 FR_Feature&)
{
  // Nothing is in base implementation. Subclass to provide your matching logic.
}
