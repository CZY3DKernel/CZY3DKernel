
#ifndef FR_AAGIterator_h
#define FR_AAGIterator_h

// FR includes
#include "./FR_AAG.h"

// OCCT includes
#include <TColStd_MapIteratorOfPackedMapOfInteger.hxx>

// Standard includes
#include <stack>

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Abstract AAG iterator.
class FR_AAGIterator : public Standard_Transient
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_AAGIterator, Standard_Transient)

public:

  //! Default constructor.
  FR_AAGIterator() : Standard_Transient()
  {}

  //! Creates and initializes iterator for AAG.
  //! \param[in] graph graph to iterate.
  FR_AAGIterator(const Handle(FR_AAG)& graph) : Standard_Transient()
  {
    this->SetGraph(graph);
  }

public:

  const Handle(FR_AAG)&
    GetGraph() const;

  void
    SetGraph(const Handle(FR_AAG)& graph);

public:

  //! \return true if current AAG exist.
  virtual bool
    More() const = 0;

  //! Selects next AAG.
  virtual void
    Next() = 0;

  //! Returns the neighbors of the current face.
  //! \param[out] neighbors neighbors.
  //! \return false is no neighbors available.
  virtual bool
    GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const = 0;

  //! \return 1-based ID of current face in AAG.
  virtual t_topoId
    GetFaceId() const = 0;

protected:

  Handle(FR_AAG) m_graph; //!< AAG being iterated.

};

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Random-order AAG iterator.
class FR_AAGRandomIterator : public FR_AAGIterator
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_AAGRandomIterator, FR_AAGIterator)

public:

  //! Creates and initializes iterator for AAG.
  //! \param[in] graph graph to iterate.
  FR_AAGRandomIterator(const Handle(FR_AAG)& graph) : FR_AAGIterator()
  {
    this->Init(graph);
  }

public:

   virtual void
    Init(const Handle(FR_AAG)& graph);

   virtual bool
    More() const;

   virtual void
    Next();

   virtual bool
    GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const;

  EXTENSION_EXPORT virtual t_topoId
    GetFaceId() const;

protected:

  FR_AdjacencyMx::t_mx::Iterator m_it; //!< Data map iterator.

};

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Random-order AAG iterator limited by a set of node indices.
class FR_AAGSetIterator : public FR_AAGIterator
{
  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_AAGSetIterator, FR_AAGIterator)

public:

  //! Creates and initializes iterator for AAG.
  //! \param[in] graph graph to iterate.
  //! \param[in] set   set of node indices which are the only indices allowed
  //!                  for iteration.
  FR_AAGSetIterator(const Handle(FR_AAG)&        graph,
                         const TColStd_PackedMapOfInteger& set) : FR_AAGIterator()
  {
    this->Init(graph, set);
  }

public:

   virtual void 
    Init(const Handle(FR_AAG)&        graph,
         const TColStd_PackedMapOfInteger& set);

   virtual bool
    More() const;

   virtual void
    Next();

   virtual bool
    GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const;

   virtual t_topoId
    GetFaceId() const;

protected:

  TColStd_MapIteratorOfPackedMapOfInteger m_it; //!< Data map iterator.

};

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Collection of rules for parameterized neighborhood iterator.
//! \sa FR_AAGNeighborsIterator
namespace FR_AAGIterationRule
{
  //! This rule is fully permissive. Using this rule you can iterate the
  //! entire connected component of AAG graph.
  class AllowAny : public Standard_Transient
  {
  public:

    // OCCT RTTI
    DEFINE_STANDARD_RTTI_INLINE(AllowAny, Standard_Transient)

  public:

    AllowAny() = default; //!< Default ctor.

  public:

    //! Iteration rule starting from a seed face. This rule
    //! is used at iterator initialization.
    bool IsBlocking(const t_topoId FR_NotUsed(seed)) { return false; }

    //! Iteration rule with two cursors: the `current` cursor moves
    //! slowly and points to a current seed face, while the `next`
    //! cursor iterates all the neighbors of the `current`. Once all
    //! the neighbors are visited, the `current` cursors moves to
    //! the next unvisited face.
    //!
    //! The iteration is done without repetitions, so if a face `X`
    //! was iterated as a neighbor of a face `Y`, it's not going to
    //! be iterated again for another face `Z` even if `X` and
    //! `Z` faces are topological neighbors.
    //!
    //! Each `next` face is guaranteed to be visited as a `current`
    //! at some earlier stage. If you'd like just to scan your model
    //! without much care of the iteration order, do not use the `current`
    //! cursor and check the `next` cursor only.
    //!
    //! \param[in] current 1-based ID of the current face.
    //! \param[in] next    1-based ID of the next current's unvisited
    //!                    neighbor face.
    //! \return true if further iteration should be stopped.
    bool IsBlocking(const t_topoId FR_NotUsed(current),
                    const t_topoId FR_NotUsed(next)) { return false; }
  };
}

//! \ingroup ASI_AFR
//!
//! AAG iterator visiting neighbor faces for the given seed in depth-first order.
//! Seed faces are also traversed by this iterator. Use this iterator to visit
//! all nodes constituting a connected component in AAG.
template <typename t_blockRule>
class FR_AAGNeighborsIterator : public FR_AAGIterator
{
public:

  //! Creates and initializes iterator for AAG.
  //! \param[in] graph graph to iterate.
  //! \param[in] seed  seed face.
  //! \param[in] rule  rule to block traversal of neighbor nodes.
  FR_AAGNeighborsIterator(const Handle(FR_AAG)& graph,
                               const t_topoId             seed,
                               const Handle(t_blockRule)& rule) : FR_AAGIterator()
  {
    this->SetBlockRule(rule);
    this->Init(graph, seed);
  }

public:

  //! Sets block rule to prevent further iterations on the current node.
  //! \param[in] rule blocking rule to set.
  void SetBlockRule(const Handle(t_blockRule)& rule)
  {
    m_blockRule = rule;
  }

public:

  //! Initializes iterator.
  //! \param[in] graph graph to iterate.
  //! \param[in] seed  1-based ID of the seed face to start iteration from.
  void Init(const Handle(FR_AAG)& graph,
            const t_topoId             seed)
  {
    FR_AAGIterator::SetGraph(graph);
    //
    m_iSeed = seed;
    m_visited.Clear();

    // Check whether the seed face should be added for traversal.
    if ( !m_blockRule->IsBlocking(m_iSeed) )
      m_fringe.push(m_iSeed);
    //
    m_visited.Add(m_iSeed);
  }

  //! \return true if there are still some faces to iterate.
  bool More() const
  {
    return !m_fringe.empty();
  }

  //! Moves iterator to another (adjacent) face.
  void Next()
  {
    // Let's throw an exception if there is nothing else to iterate.
    if ( !this->More() )
      Standard_ProgramError::Raise("No next item");

    // Take current.
    const t_topoId iCurrent = this->GetFaceId();
    m_fringe.pop(); // Top item is done.

    // Put all nodes pending for iteration to the fringe.
    const TColStd_PackedMapOfInteger& neighbors = m_graph->GetNeighbors(iCurrent);
    //
    for ( TColStd_MapIteratorOfPackedMapOfInteger nit(neighbors); nit.More(); nit.Next() )
    {
      const t_topoId iNext = nit.Key();
      //
      if ( m_visited.Contains(iNext) )
        continue;

      // Set as visited.
      m_visited.Add(iNext);

      // Check whether the current neighbor should be added for traversal.
      if ( m_blockRule->IsBlocking(iCurrent, iNext) )
        continue;

      // Set node to iterate.
      m_fringe.push(iNext);
    }
  }

  //! Returns the neighbors of the current face.
  //! \param[out] neighbors neighbors.
  //! \return false is no neighbors available.
  bool GetNeighbors(TColStd_PackedMapOfInteger& neighbors) const
  {
    const t_topoId face_id = this->GetFaceId();
    if ( !m_graph->HasNeighbors(face_id) )
      return false;

    neighbors = m_graph->GetNeighbors(face_id);
    return true;
  }

  //! \return ID of the current face.
  t_topoId GetFaceId() const
  {
    return m_fringe.top();
  }

protected:

  t_topoId                   m_iSeed;     //!< Seed node.
  TColStd_PackedMapOfInteger m_visited;   //!< Visited nodes.
  std::stack<t_topoId>       m_fringe;    //!< Where to return.
  Handle(t_blockRule)        m_blockRule; //!< Rule to block further iterations.

};

#endif
