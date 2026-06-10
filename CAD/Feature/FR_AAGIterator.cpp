#include "./FR_AAGIterator.h"

//-----------------------------------------------------------------------------

//! \return AAG being iterated.
const Handle(FR_AAG)& FR_AAGIterator::GetGraph() const
{
  return m_graph;
}

//! Initializes iterator with graph.
//! \param[in] graph graph to iterate.
void FR_AAGIterator::SetGraph(const Handle(FR_AAG)& graph)
{
  m_graph = graph;
}

//-----------------------------------------------------------------------------

//! Initializes iterator with graph.
//! \param[in] graph graph to iterate.
void FR_AAGRandomIterator::Init(const Handle(FR_AAG)& graph)
{
  FR_AAGIterator::SetGraph(graph);
  //
  m_it.Initialize( m_graph->GetNeighborhood().mx );
}


//! \return true if there are still some faces to iterate.
bool FR_AAGRandomIterator::More() const
{
  return m_it.More();
}

//! Moves iterator to another (not adjacent) face.
void FR_AAGRandomIterator::Next()
{
  m_it.Next();
}

//! Returns the neighbors of the current face.
//! \param[out] neighbors neighbors.
//! \return false is no neighbors available.
bool FR_AAGRandomIterator::GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const
{
  neighbors = m_it.Value();
  return true;
}

//! \return ID of the current face.
t_topoId FR_AAGRandomIterator::GetFaceId() const
{
  return m_it.Key();
}

//-----------------------------------------------------------------------------

//! Initializes iterator with graph.
//! \param[in] graph graph to iterate.
//! \param[in] set   indices to iterate.
void FR_AAGSetIterator::Init(const Handle(FR_AAG)&        graph,
                                  const TColStd_PackedMapOfInteger& set)
{
  FR_AAGIterator::SetGraph(graph);
  //
  m_it.Initialize(set);
}


//! \return true if there are still some faces to iterate.
bool FR_AAGSetIterator::More() const
{
  return m_it.More();
}

//! Moves iterator to another (not adjacent) face.
void FR_AAGSetIterator::Next()
{
  m_it.Next();
}

//! Returns the neighbors of the current face.
//! \param[out] neighbors neighbors.
//! \return false is no neighbors available.
bool FR_AAGSetIterator::GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const
{
  const t_topoId face_id = m_it.Key();
  if ( !m_graph->HasNeighbors(face_id) )
    return false;

  neighbors = m_graph->GetNeighbors(face_id);
  return true;
}

//! \return ID of the current face.
t_topoId FR_AAGSetIterator::GetFaceId() const
{
  return m_it.Key();
}
