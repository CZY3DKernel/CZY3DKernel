//
// Copyright (c) 1993-2022 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////

#include "Rtree.hxx"
/// Variables for finding a split partition
struct ON_RTreePartitionVars
{
  int m_partition[ON_RTree_MAX_NODE_COUNT+1];
  int m_total;
  int m_minFill;
  int m_taken[ON_RTree_MAX_NODE_COUNT+1];
  int m_count[2];
  ON_RTreeBBox m_cover[2];
  double m_area[2];

  ON_RTreeBranch m_branchBuf[ON_RTree_MAX_NODE_COUNT+1];
  int m_branchCount;
  ON_RTreeBBox m_coverSplit;
  double m_coverSplitArea;
}; 

typedef bool (*ON_RTreeSearchCallback)(void* a_context, ON__INT_PTR a_id );

struct ON_RTreeSearchResultCallback
{
  void* m_context;
  ON_RTreeSearchCallback m_resultCallback;
};

static void ChoosePartition(ON_RTreePartitionVars* a_parVars, int a_minFill);
static void CountRec(ON_RTreeNode* a_node, int& a_count);
static void PickSeeds(ON_RTreePartitionVars* a_parVars);
static void InitParVars(ON_RTreePartitionVars* a_parVars, int a_maxRects, int a_minFill);
static void GetBranches(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreePartitionVars* a_parVars);
static int PickBranch(ON_RTreeBBox* a_rect, ON_RTreeNode* a_node);
static void DisconnectBranch(ON_RTreeNode* a_node, int a_index);
static ON_RTreeBBox NodeCover(ON_RTreeNode* a_node);
static void InitRect(ON_RTreeBBox* a_rect);
static ON_RTreeBBox CombineRectHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB);
static double CalcRectVolumeHelper(const ON_RTreeBBox* a_rect);
static bool OverlapHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB);
static void ClassifyHelper(int a_index, int a_group, struct ON_RTreePartitionVars* a_parVars);
static bool SearchHelper(const ON_RTreeNode* a_node, ON_RTreeBBox* a_rect, ON_RTreeSearchResultCallback& a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_RTreeSearchResult& a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<ON_RTreeLeaf> &a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<int> &a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<void*> &a_result );
////////////////////////////////////////////////////////////////
//
// ON_RTreeIterator
//

ON_RTreeIterator::ON_RTreeIterator()
{ 
  Initialize(0);
}

ON_RTreeIterator::ON_RTreeIterator(const class ON_RTree& rtree)
{ 
  Initialize(rtree);
}

ON_RTreeIterator::~ON_RTreeIterator()
{ 
}

const ON_RTreeBranch* ON_RTreeIterator::Value() const
{
  return ( 0 != m_sp )
         ? &m_sp->m_node->m_branch[m_sp->m_branchIndex]
         : 0;
}

bool ON_RTreeIterator::Initialize(const ON_RTree& a_rtree)
{
  return Initialize(a_rtree.Root());
}

bool ON_RTreeIterator::Initialize(const ON_RTreeNode* a_node)
{ 
  m_sp = 0;
  m_root = ( 0 != a_node && a_node->m_count > 0 ) ? a_node : 0;
  return First();
}

bool ON_RTreeIterator::PushChildren(StackElement* sp, bool bFirstChild )
{ 
  StackElement* spmax = &m_stack[0] + MAX_STACK;
  const ON_RTreeNode* node = sp->m_node;
  m_sp = 0;
  // push first leaf coverted by this node onto the stack
  while( 0 != node && node->m_level >= 0 && node->m_count > 0 )
  {
    if ( 0 == node->m_level )
    {
      m_sp = sp;
      return true;
    }
    node = node->m_branch[sp->m_branchIndex].m_child;
    if( ++sp == spmax )
    {
      // Either this is a GIGANTIC R-tree, or, more likely, there is
      // a bug in the code that creates the R-tree and this R-tree
      // is horribly unbalanced. If the case is valid, then we must 
      // increase MAX_STACK and ship a service release.
      //ON_ERROR("ON_RTreeIterator::PushFirstChild - stack overflow");
      return false;
    }
    sp->m_node = node;
    sp->m_branchIndex = bFirstChild ? 0 : node->m_count-1; // 0 for first child
  }
  return false;
}

bool ON_RTreeIterator::First()
{ 
  m_sp = 0;
  if ( 0 == m_root || m_root->m_level < 0 || m_root->m_count <= 0 )
    return false;
  m_stack[0].m_node = m_root;
  m_stack[0].m_branchIndex = 0;
  return PushChildren(&m_stack[0],true);
}

bool ON_RTreeIterator::Last()
{ 
  m_sp = 0;
  if ( 0 == m_root || m_root->m_level < 0 || m_root->m_count <= 0 )
    return false;
  m_stack[0].m_node = m_root;
  m_stack[0].m_branchIndex = m_root->m_count - 1;
  return PushChildren(&m_stack[0],false);
}

bool ON_RTreeIterator::Next()
{ 
  if ( 0 == m_sp )
    return false; // invalid iterator

  if ( ++(m_sp->m_branchIndex) < m_sp->m_node->m_count )
    return true; // m_sp->m_node is always at leaf level

  // pop the stack until we find an element with room to move over.
  StackElement* sp0 = &m_stack[0];
  StackElement* sp = m_sp;
  m_sp = 0;
  while ( sp > sp0 )
  {        
    sp--; // pop the stack

    // ++(sp->m_branchIndex) moves to the next element in sp->m_node.
    if ( ++(sp->m_branchIndex) >= sp->m_node->m_count )
      continue; // this element is used up

    // Since we've popped the stack, we cannot be at the leaf level.
    // PushFirst() pushes the first child onto the stack until 
    // it reaches the leaf level.
    return PushChildren(sp,true);
  }
  return false; // we were at the last element and now there are no more.
}

bool ON_RTreeIterator::Prev()
{ 
  if ( 0 == m_sp )
    return false; // invalid iterator

  if ( --(m_sp->m_branchIndex) >= 0 )
    return true; // m_sp->m_node is always at leaf level

  // pop the stack until we find an element with room to move over.
  StackElement* sp0 = &m_stack[0];
  StackElement* sp = m_sp;
  m_sp = 0;
  while ( sp > sp0 )
  {        
    sp--; // pop the stack

    // --(sp->m_branchIndex) moves to the previous element in sp->m_node.
    if ( --(sp->m_branchIndex) < 0 )
      continue; // this element is used up

    // Since we've popped the stack, we cannot be at the leaf level.
    // PushFirst() pushes the first child onto the stack until 
    // it reaches the leaf level.
    return PushChildren(sp,false);
  }
  return false; // we were at the last element and now there are no more.
}


////////////////////////////////////////////////////////////////
//
// ON_RTree
//


ON_RTree::ON_RTree( size_t leaf_count ){
	m_root = AllocNode();
	m_root->m_level = 0;
}

ON_RTreeNode* ON_RTree::AllocNode() {
	ON_RTreeNode* newNode = new ON_RTreeNode;
	newNode->m_count = 0;
	newNode->m_level = -1;
	return newNode;
};

// Allocate space for a node in the list used in DeletRect to
// store Nodes that are too empty.
ON_RTreeListNode* ON_RTree::AllocListNode() {
	return new ON_RTreeListNode;
};

void ON_RTree::FreeNode(ON_RTreeNode* a_node) {
	delete a_node;
};


void ON_RTree::FreeListNode(ON_RTreeListNode* a_listNode) {
	delete a_listNode;
};


ON_RTree::~ON_RTree()
{
  RemoveAll();
}

bool ON_RTree::Insert2d(const double a_min[2], const double a_max[2], int a_element_id)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Insert(min3d,max3d,a_element_id);
}

bool ON_RTree::Insert(const double a_min[3], const double a_max[3], int a_element_id)
{
  bool rc;
  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));
  if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
  {  
    if ( 0 == m_root )
    {
      m_root = AllocNode();
      m_root->m_level = 0;
    }
    InsertRect(&rect, a_element_id, &m_root, 0);
    rc = true;
  }
  else
  {
    // invalid bounding box - don't let this corrupt the tree
    rc = false;
    //ON_ERROR("ON_RTree::Insert - invalid a_min[] or a_max[] input.");
  }
  return rc;
}

bool ON_RTree::Insert2d(const double a_min[2], const double a_max[2], void* a_element_id)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Insert(min3d,max3d,a_element_id);
}

bool ON_RTree::Insert(const double a_min[3], const double a_max[3], void* a_element_id)
{
  bool rc;
  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));
  if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
  {  
    if ( 0 == m_root )
    {
      m_root = AllocNode();
      m_root->m_level = 0;
    }

    // The ON__INT_PTR cast is safe because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_PUSH
// Disable warning C4311: 'type cast' : pointer truncation from 'void *' to 'ON__INT_PTR'
#pragma ON_PRAGMA_WARNING_DISABLE_MSC( 4311 )
#endif
    InsertRect(&rect, (ON__INT_PTR)a_element_id, &m_root, 0);
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_POP
#endif

    rc = true;
  }
  else
  {
    // invalid bounding box - don't let this corrupt the tree
    rc = false;
    //ON_ERROR("ON_RTree::Insert - invalid a_min[] or a_max[] input.");
  }
  return rc;
}

bool ON_RTree::Remove2d(const double a_min[2], const double a_max[2], int a_dataId)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Remove(min3d,max3d,a_dataId);
}

bool ON_RTree::Remove(const double a_min[3], const double a_max[3], int a_dataId)
{
  bool rc = false;
  if ( 0 != m_root )
  {
    ON_RTreeBBox rect;
    memcpy(rect.m_min,a_min,sizeof(rect.m_min));
    memcpy(rect.m_max,a_max,sizeof(rect.m_max));
    if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
    {  
      // RemoveRect() returns 0 on success
      rc = (0 ==  RemoveRect(&rect, a_dataId, &m_root));
    }
    else
    {
      // invalid bounding box - don't let this corrupt the tree
      //ON_ERROR("ON_RTree::Remove - invalid a_min[] or a_max[] input.");
    }
  }
  return rc;
}

bool ON_RTree::Remove2d(const double a_min[2], const double a_max[2],  void* a_dataId)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Remove(min3d,max3d,a_dataId);
}

bool ON_RTree::Remove(const double a_min[3], const double a_max[3], void* a_dataId)
{
  bool rc = false;
  if ( 0 != m_root )
  {
    ON_RTreeBBox rect;
    memcpy(rect.m_min,a_min,sizeof(rect.m_min));
    memcpy(rect.m_max,a_max,sizeof(rect.m_max));
    if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
    {  
      // RemoveRect() returns 0 on success
      // The ON__INT_PTR cast is save because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_PUSH
// Disable warning C4311: 'type cast' : pointer truncation from 'void *' to 'ON__INT_PTR'
#pragma ON_PRAGMA_WARNING_DISABLE_MSC( 4311 )
#endif
      rc = (0 ==  RemoveRect(&rect, (ON__INT_PTR)a_dataId, &m_root));
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_POP
#endif

    }
    else
    {
      // invalid bounding box - don't let this corrupt the tree
      //ON_ERROR("ON_RTree::Remove - invalid a_min[] or a_max[] input.");
    }
  }
  return rc;
}

bool ON_RTree::Search(
	ON_RTreeBBox* a_rect,
	bool  a_resultCallback(void* a_context, ON__INT_PTR a_id),
	void* a_context) const
{
  if ( 0 == m_root || 0 == a_rect )
    return false;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;
  return SearchHelper(m_root, a_rect, result);
}

bool ON_RTree::Search(const double a_min[3], const double a_max[3],
	bool  a_resultCallback(void* a_context, ON__INT_PTR a_id),void* a_context) const
{
	if (0 == m_root)
		return false;

	ON_RTreeBBox rect;
	memcpy(rect.m_min, a_min, sizeof(rect.m_min));
	memcpy(rect.m_max, a_max, sizeof(rect.m_max));

	ON_RTreeSearchResultCallback result;
	result.m_context = a_context;
	result.m_resultCallback = a_resultCallback;
	return SearchHelper(m_root, &rect, result);
}

bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
	bool  resultCallback(void* a_context, ON__INT_PTR a_id), void* a_context
) const {
	if (0 == m_root)
		return false;

	ON_RTreeBBox rect;
	memcpy(rect.m_min, a_min, 2 * sizeof(a_min[0]));
	rect.m_min[2] = 0.0;
	memcpy(rect.m_max, a_max, 2 * sizeof(a_max[0]));
	rect.m_max[2] = 0.0;

	ON_RTreeSearchResultCallback result;
	result.m_context = a_context;
	result.m_resultCallback = resultCallback;
	return SearchHelper(m_root, &rect, result);
}

bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          std::vector<ON_RTreeLeaf>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect, a_result);
}


bool ON_RTree::Search(const double a_min[3], const double a_max[3],
                          std::vector<ON_RTreeLeaf>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect, a_result);
}


bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          std::vector<void*>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect,  a_result);
}


bool ON_RTree::Search(const double a_min[3], const double a_max[3],
                          std::vector<void*>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect,  a_result);
}


bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
	std::vector<int>& a_result
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect,  a_result);
}

bool ON_RTree::Search(const double a_min[3], const double a_max[3],
	std::vector<int>& a_result) const{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect,  a_result);
}

bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          ON_RTreeSearchResult& a_result ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect, a_result);
}

bool ON_RTree::Search(const double a_min[3], const double a_max[3],
                          ON_RTreeSearchResult& a_result ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect, a_result);
}

struct ON_RTreePairSearchResult
{
  double m_tolerance;
  std::vector<ON_2dex>* m_result;
};


static bool PairSearchOverlapHelper( const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB, double tolerance )
{
  double dx,dy,dz,d;
  const double* mn;
  const double* mx;


  mx = a_rectA->m_max;
  mn = a_rectB->m_min;
  dx = *mn++ - *mx++;
  if ( dx > tolerance ) return false;
  dy = *mn++ - *mx++;
  if ( dy > tolerance ) return false;
  dz = *mn - *mx;
  if ( dz > tolerance ) return false;

  mx = a_rectB->m_max;
  mn = a_rectA->m_min;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dx ) dx = d;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dy ) dy = d;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dz ) dz = d;

  d  = (dx > 0.0) ? dx*dx : 0.0;
  d += (dy > 0.0) ? dy*dy : 0.0;
  d += (dz > 0.0) ? dz*dz : 0.0;

  return (d <= tolerance*tolerance);
}


static void PairSearchHelper( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        PairSearchHelper(a_branchA,branchB->m_child,a_result);
      }
      else
      {
		a_result->m_result->emplace_back((int)a_branchA->m_id, (int)branchB->m_id);
      }
    }
    branchB++;
  }
}

static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        PairSearchHelper(branchA->m_child,a_branchB,a_result);
      }
      else
      {
		a_result->m_result->emplace_back((int)branchA->m_id, (int)a_branchB->m_id);
      }
    }
    branchA++;
  }
}


static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
            PairSearchHelper(branchA->m_child,branchB->m_child,a_result);
          else
            PairSearchHelper(branchA->m_child,branchB,a_result);
        }
        else if ( a_nodeB->m_level > 0 )
        {
          PairSearchHelper(branchA,branchB->m_child,a_result);
        }
        else
        {
		  a_result->m_result->emplace_back((int)branchA->m_id, (int)branchB->m_id);
        }
      }
    }
    branchA++;
  }
}


bool ON_RTree::Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
	std::vector<ON_2dex>& a_result
          )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchResult r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_result = &a_result;
  PairSearchHelper(a_rtreeA.m_root,a_rtreeB.m_root,&r);
  return true;
}

typedef void (*ON_RTreePairSearchCallback)(void*, ON__INT_PTR, ON__INT_PTR);

struct ON_RTreePairSearchCallbackResult
{
  double m_tolerance;
  void* m_context;
  ON_RTreePairSearchCallback m_resultCallback;
};

typedef bool (*ON_RTreePairSearchCallbackBool)(void*, ON__INT_PTR, ON__INT_PTR);

struct ON_RTreePairSearchCallbackResultBool
{
  double m_tolerance;
  void* m_context;
  ON_RTreePairSearchCallbackBool m_resultCallbackBool;
};


typedef bool(*ON_RTreePairSearchCallbackBoolTolerance)(void*, ON__INT_PTR, ON__INT_PTR, double*);

struct ON_RTreePairSearchCallbackResultBoolTolerance
{
  double m_tolerance;
  void* m_context;
  ON_RTreePairSearchCallbackBoolTolerance m_resultCallbackBoolTolerance;
};

static void PairSearchHelper( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        PairSearchHelper(a_branchA,branchB->m_child,a_result);
      }
      else
      {
        a_result->m_resultCallback(a_result->m_context,a_branchA->m_id,branchB->m_id);
      }
    }
    branchB++;
  }
}

static bool PairSearchHelperBool( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        if ( !PairSearchHelperBool(a_branchA,branchB->m_child,a_result) )
          return false;
      }
      else
      {
        if ( !a_result->m_resultCallbackBool(a_result->m_context,a_branchA->m_id,branchB->m_id) )
          return false;
      }
    }
    branchB++;
  }
  return true;
}


static bool PairSearchHelperBoolTolerance(const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while (branchB < branchBmax)
  {
    if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
    {
      if (a_nodeB->m_level > 0)
      {
        if (!PairSearchHelperBoolTolerance(a_branchA, branchB->m_child, a_result))
          return false;
      }
      else
      {
        if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, a_branchA->m_id, branchB->m_id, &a_result->m_tolerance))
          return false;
      }
    }
    branchB++;
  }
  return true;
}


static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        PairSearchHelper(branchA->m_child,a_branchB,a_result);
      }
      else
      {
        a_result->m_resultCallback(a_result->m_context,branchA->m_id,a_branchB->m_id);
      }
    }
    branchA++;
  }
}

static bool PairSearchHelperBool( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        if ( !PairSearchHelperBool(branchA->m_child,a_branchB,a_result) )
          return false;
      }
      else
      {
        if ( !a_result->m_resultCallbackBool(a_result->m_context,branchA->m_id,a_branchB->m_id) )
          return false;
      }
    }
    branchA++;
  }
  return true;
}


static bool PairSearchHelperBoolTolerance(const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while (branchA < branchAmax)
  {
    if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
    {
      if (a_nodeA->m_level > 0)
      {
        if (!PairSearchHelperBoolTolerance(branchA->m_child, a_branchB, a_result))
          return false;
      }
      else
      {
        if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, branchA->m_id, a_branchB->m_id, &a_result->m_tolerance))
          return false;
      }
    }
    branchA++;
  }
  return true;
}



static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
            PairSearchHelper(branchA->m_child,branchB->m_child,a_result);
          else
            PairSearchHelper(branchA->m_child,branchB,a_result);
        }
        else if ( a_nodeB->m_level > 0 )
        {
          PairSearchHelper(branchA,branchB->m_child,a_result);
        }
        else
        {
          a_result->m_resultCallback(a_result->m_context,branchA->m_id,branchB->m_id);
        }
      }
    }
    branchA++;
  }
}


static bool PairSearchHelperBool( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
          {
            if ( !PairSearchHelperBool(branchA->m_child,branchB->m_child,a_result) )
              return false;
          }
          else
          {
            if ( !PairSearchHelperBool(branchA->m_child,branchB,a_result) )
              return false;
          }
        }
        else if ( a_nodeB->m_level > 0 )
        {
          if ( !PairSearchHelperBool(branchA,branchB->m_child,a_result) )
            return false;
        }
        else
        {
          if ( !a_result->m_resultCallbackBool(a_result->m_context,branchA->m_id,branchB->m_id) )
            return false;
        }
      }
    }
    branchA++;
  }
  return true;
}


static bool PairSearchHelperBoolTolerance(const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;

  if (a_nodeA->m_level > 0)
  {
    if (a_nodeB->m_level > 0)
    {
      // neither branchA nor branchB are leaf nodes
      while (branchA < branchAmax)
      {
        for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
            if (!PairSearchHelperBoolTolerance(branchA->m_child, branchB->m_child, a_result))
              return false;
          }
        }
        branchA++;
      }
    }
    else
    {
      // branchB nodes are leaves and branchA nodes are not
      while (branchA < branchAmax)
      {
        for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
            if (!PairSearchHelperBoolTolerance(branchA->m_child, branchB, a_result))
              return false;
          }
        }
        branchA++;
      }
    }
  }
  else if (a_nodeB->m_level > 0)
  {
    // branchA nodes are leaves and branchB nodes are not
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (!PairSearchHelperBoolTolerance(branchA, branchB->m_child, a_result))
            return false;
        }
      }
      branchA++;
    }
  }
  else
  {
    // branchA and branchB are leaf nodes
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, branchA->m_id, branchB->m_id, &a_result->m_tolerance))
            return false;
        }
      }
      branchA++;
    }
  }

  while (branchA < branchAmax)
  {
    for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
    {
      if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
      {
        if (a_nodeA->m_level > 0)
        {
          if (a_nodeB->m_level > 0)
          {
            if (!PairSearchHelperBoolTolerance(branchA->m_child, branchB->m_child, a_result))
              return false;
          }
          else
          {
            if (!PairSearchHelperBoolTolerance(branchA->m_child, branchB, a_result))
              return false;
          }
        }
        else if (a_nodeB->m_level > 0)
        {
          if (!PairSearchHelperBoolTolerance(branchA, branchB->m_child, a_result))
            return false;
        }
        else
        {
          if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, branchA->m_id, branchB->m_id, &a_result->m_tolerance))
            return false;
        }
      }
    }
    branchA++;
  }

  return true;
}

bool ON_RTree::Search( 
  const ON_RTree& a_rtreeA,
  const ON_RTree& a_rtreeB, 
  double tolerance,
  void  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
  void* a_context
  )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchCallbackResult r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallback = resultCallback;
  PairSearchHelper(a_rtreeA.m_root,a_rtreeB.m_root,&r);
  return true;
}

bool ON_RTree::Search( 
  const ON_RTree& a_rtreeA,
  const ON_RTree& a_rtreeB, 
  double tolerance,
  bool  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
  void* a_context
  )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchCallbackResultBool r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallbackBool = resultCallback;

  // Do not return false if PairSearchHelperBool() returns false.  The only reason
  // PairSearchHelperBool() returns false is that the user specified resultCallback()
  // terminated the search. This way a programmer with the ability to reason can
  // distinguish between a terminiation and a failure to start because input is
  // missing.
  PairSearchHelperBool(a_rtreeA.m_root,a_rtreeB.m_root,&r);

  return true;
}

bool ON_RTree::Search(
  const ON_RTree& a_rtreeA,
  const ON_RTree& a_rtreeB,
  double tolerance,
  bool  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB, double* tolerance),
  void* a_context
  )
{
  if (0 == a_rtreeA.m_root)
    return false;
  if (0 == a_rtreeB.m_root)
    return false;
  ON_RTreePairSearchCallbackResultBoolTolerance r;
  r.m_tolerance =tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallbackBoolTolerance = resultCallback;

  // Do not return false if PairSearchHelperBoolTolerance() returns false.  
  // The only reason PairSearchHelperBoolTolerance() returns false is that
  // the user specified resultCallback() terminated the search. 
  // This way a programmer with the ability to reason can distinguish between
  // a terminiation and a failure to start because input is missing.
  PairSearchHelperBoolTolerance(a_rtreeA.m_root, a_rtreeB.m_root, &r);

  return true;
}


#if 1

static void SingleTreeSearchHelper(const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;

  if (a_nodeB->m_level > 0)
  {
    // branchB's have children nodes and a_branchA is a leaf
    while (branchB < branchBmax)
    {
      if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
      {
        SingleTreeSearchHelper(a_branchA, branchB->m_child, a_result);
      }
      branchB++;
    }
  }
  else
  {
    // branchB's are leaves and a_branchA is a leaf
    while (branchB < branchBmax)
    {
      if (a_branchA < branchB)
      {
        if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
		  a_result->m_result->emplace_back((int)a_branchA->m_id, (int)branchB->m_id);
        }
      }
      branchB++;
    }
  }
}

static void SingleTreeSearchHelper(const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;

  if (a_nodeA->m_level > 0)
  {
    // branchA's have children nodes and a_branchB is a leaf
    while (branchA < branchAmax)
    {
      if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
      {
        SingleTreeSearchHelper(branchA->m_child, a_branchB, a_result);
      }
      branchA++;
    }
  }
  else
  {
    // branchA's are leaves and a_branchB is a leaf
    while (branchA < branchAmax)
    {
      if (branchA < a_branchB)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
        {
		  a_result->m_result->emplace_back((int)branchA->m_id, (int)a_branchB->m_id);
        }
      }
      branchA++;
    }
  }
}


static void SingleTreeSearchHelper(const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;

  if (a_nodeA->m_level > 0 || a_nodeB->m_level > 0)
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (a_nodeA->m_level > 0)
          {
            if (a_nodeB->m_level > 0)
              SingleTreeSearchHelper(branchA->m_child, branchB->m_child, a_result);
            else
              SingleTreeSearchHelper(branchA->m_child, branchB, a_result);
          }
          else // a_nodeB->m_level > 0
          {
            SingleTreeSearchHelper(branchA, branchB->m_child, a_result);
          }
        }
      }
      branchA++;
    }
  }
  else
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        // branchA and branchB are leaf nodes in the same same tree.
        // Don't test pairs twice and don't test a node against itself.
        if (branchA < branchB)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
			a_result->m_result->emplace_back((int)branchA->m_id, (int)branchB->m_id);
          }
        }
      }
      branchA++;
    }
  }

}

bool ON_RTree::Search(
  double tolerance,
  std::vector<ON_2dex>& a_result
  ) const
{
  if (0 == this->m_root)
    return false;
  ON_RTreePairSearchResult r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_result = &a_result;
  SingleTreeSearchHelper(this->m_root, this->m_root, &r);
  return true;
}

static void SingleTreeSearchHelper(const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;

  if (a_nodeB->m_level > 0)
  {
    // branchB's have children nodes and a_branchA is a leaf
    while (branchB < branchBmax)
    {
      if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
      {
        SingleTreeSearchHelper(a_branchA, branchB->m_child, a_result);
      }
      branchB++;
    }
  }
  else
  {
    // branchB's are all leaves
    while (branchB < branchBmax)
    {
      if (a_branchA < branchB)
      {
        if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          a_result->m_resultCallback(a_result->m_context, a_branchA->m_id, branchB->m_id);
        }
      }
      branchB++;
    }
  }
}

static bool SingleTreeSearchHelperBool(const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;

  if (a_nodeB->m_level > 0)
  {
    // branchB's have children nodes and a_branchA is a leaf
    while (branchB < branchBmax)
    {
      if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
      {
        if (!SingleTreeSearchHelperBool(a_branchA, branchB->m_child, a_result))
          return false;
      }
      branchB++;
    }
  }
  else
  {
    // branchB's are all leaves
    while (branchB < branchBmax)
    {
      if (a_branchA < branchB)
      {
        if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (!a_result->m_resultCallbackBool(a_result->m_context, a_branchA->m_id, branchB->m_id))
            return false;
        }
      }
      branchB++;
    }
  }

  return true;
}


static bool SingleTreeSearchHelperBoolTolerance(const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;

  if (a_nodeB->m_level > 0)
  {
    // branchB's have children nodes and a_branchA is a leaf
    while (branchB < branchBmax)
    {
      if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
      {
        if (!SingleTreeSearchHelperBoolTolerance(a_branchA, branchB->m_child, a_result))
          return false;
      }
      branchB++;
    }
  }
  else
  {
    // branchB's are all leaves
    while (branchB < branchBmax)
    {
      if (a_branchA < branchB)
      {
        if (PairSearchOverlapHelper(&a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, a_branchA->m_id, branchB->m_id, &a_result->m_tolerance))
            return false;
        }
      }
      branchB++;
    }
  }

  return true;
}


static void SingleTreeSearchHelper(const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;

  if (a_nodeA->m_level > 0)
  {
    // branchA's have children nodes and a_branchB is a leaf
    while (branchA < branchAmax)
    {
      if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
      {
        SingleTreeSearchHelper(branchA->m_child, a_branchB, a_result);
      }
      branchA++;
    }
  }
  else
  {
    // branchA's are all leaves
    while (branchA < branchAmax)
    {
      if (branchA < a_branchB)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
        {
          a_result->m_resultCallback(a_result->m_context, branchA->m_id, a_branchB->m_id);
        }
      }
      branchA++;
    }
  }
}

static bool SingleTreeSearchHelperBool(const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResultBool* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;

  if (a_nodeA->m_level > 0)
  {
    // branchA's have children nodes and a_branchB is a leaf
    while (branchA < branchAmax)
    {
      if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
      {
        if (!SingleTreeSearchHelperBool(branchA->m_child, a_branchB, a_result))
          return false;
      }
      branchA++;
    }
  }
  else
  {
    // branchA's are all leaves
    while (branchA < branchAmax)
    {
      if (branchA < a_branchB)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
        {
          if (!a_result->m_resultCallbackBool(a_result->m_context, branchA->m_id, a_branchB->m_id))
            return false;
        }
      }
      branchA++;
    }
  }

  return true;
}


static bool SingleTreeSearchHelperBoolTolerance(const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;

  if (a_nodeA->m_level > 0)
  {
    // branchA's have children nodes and a_branchB is a leaf
    while (branchA < branchAmax)
    {
      if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
      {
        if (!SingleTreeSearchHelperBoolTolerance(branchA->m_child, a_branchB, a_result))
          return false;
      }
      branchA++;
    }
  }
  else
  {
    // branchA's are all leaves
    while (branchA < branchAmax)
    {
      if (branchA < a_branchB)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance))
        {
          if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, branchA->m_id, a_branchB->m_id, &a_result->m_tolerance))
            return false;
        }
      }
      branchA++;
    }
  }

  return true;
}

static void SingleTreeSearchHelper(const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;

  if (a_nodeA->m_level > 0 || a_nodeB->m_level > 0)
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (a_nodeA->m_level > 0)
          {
            if (a_nodeB->m_level > 0)
              SingleTreeSearchHelper(branchA->m_child, branchB->m_child, a_result);
            else
              SingleTreeSearchHelper(branchA->m_child, branchB, a_result);
          }
          else // a_nodeB->m_level > 0
          {
            SingleTreeSearchHelper(branchA, branchB->m_child, a_result);
          }
        }
      }
      branchA++;
    }
  }
  else
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        // branchA and branchB are leaf nodes in the same same tree.
        // Don't test pairs twice and don't test a node against itself.
        if (branchA < branchB)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
            a_result->m_resultCallback(a_result->m_context, branchA->m_id, branchB->m_id);
          }
        }
      }
      branchA++;
    }
  }

}


static bool SingleTreeSearchHelperBool(const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;

  if (a_nodeA->m_level > 0 || a_nodeB->m_level > 0)
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (a_nodeA->m_level > 0)
          {
            if (a_nodeB->m_level > 0)
            {
              if (!SingleTreeSearchHelperBool(branchA->m_child, branchB->m_child, a_result))
                return false;
            }
            else
            {
              if (!SingleTreeSearchHelperBool(branchA->m_child, branchB, a_result))
                return false;
            }
          }
          else // a_nodeB->m_level > 0
          {
            if (!SingleTreeSearchHelperBool(branchA, branchB->m_child, a_result))
              return false;
          }
        }
      }
      branchA++;
    }
  }
  else
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        // branchA and branchB are leaf nodes in the same same tree.
        // Don't test pairs twice and don't test a node against itself.
        if (branchA < branchB)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
            if (!a_result->m_resultCallbackBool(a_result->m_context, branchA->m_id, branchB->m_id))
              return false;
          }
        }
      }
      branchA++;
    }
  }

  return true;
}



static bool SingleTreeSearchHelperBoolTolerance(const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBoolTolerance* a_result)
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;

  if (a_nodeA->m_level > 0 || a_nodeB->m_level > 0)
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
        {
          if (a_nodeA->m_level > 0)
          {
            if (a_nodeB->m_level > 0)
            {
              if (!SingleTreeSearchHelperBoolTolerance(branchA->m_child, branchB->m_child, a_result))
                return false;
            }
            else
            {
              if (!SingleTreeSearchHelperBoolTolerance(branchA->m_child, branchB, a_result))
                return false;
            }
          }
          else // a_nodeB->m_level > 0
          {
            if (!SingleTreeSearchHelperBoolTolerance(branchA, branchB->m_child, a_result))
              return false;
          }
        }
      }
      branchA++;
    }
  }
  else
  {
    while (branchA < branchAmax)
    {
      for (branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++)
      {
        // branchA and branchB are leaf nodes in the same same tree.
        // Don't test pairs twice and don't test a node against itself.
        if (branchA < branchB)
        {
          if (PairSearchOverlapHelper(&branchA->m_rect, &branchB->m_rect, a_result->m_tolerance))
          {
            if (!a_result->m_resultCallbackBoolTolerance(a_result->m_context, branchA->m_id, branchB->m_id, &a_result->m_tolerance))
              return false;
          }
        }
      }
      branchA++;
    }
  }

  return true;
}



bool ON_RTree::Search( 
  double tolerance,
  void  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
  void* a_context
  ) const
{
  if (0 == this->m_root)
    return false;
  ON_RTreePairSearchCallbackResult r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallback = resultCallback;
  SingleTreeSearchHelper(this->m_root, this->m_root, &r);
  return true;
}

bool ON_RTree::Search(
  double tolerance,
  bool  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB),
  void* a_context
  ) const
{
  if (0 == this->m_root)
    return false;
  ON_RTreePairSearchCallbackResultBool r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallbackBool = resultCallback;

  // Do not return false if PairSearchHelperBool() returns false.  The only reason
  // PairSearchHelperBool() returns false is that the user specified resultCallback()
  // terminated the search. This way a programmer with the ability to reason can
  // distinguish between a terminiation and a failure to start because input is
  // missing.
  SingleTreeSearchHelperBool(this->m_root, this->m_root, &r);

  return true;
}


bool ON_RTree::Search(
  double tolerance,
  bool  resultCallback(void* a_context, ON__INT_PTR a_idA, ON__INT_PTR a_idB, double* tolerance),
  void* a_context
  ) const
{
  if (0 == this->m_root)
    return false;
  ON_RTreePairSearchCallbackResultBoolTolerance r;
  r.m_tolerance = tolerance > 0.0 ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallbackBoolTolerance = resultCallback;

  // Do not return false if PairSearchHelperBool() returns false.  The only reason
  // PairSearchHelperBool() returns false is that the user specified resultCallback()
  // terminated the search. This way a programmer with the ability to reason can
  // distinguish between a terminiation and a failure to start because input is
  // missing.
  SingleTreeSearchHelperBoolTolerance(this->m_root, this->m_root, &r);

  return true;
}

#endif


int ON_RTree::ElementCount()
{
  int count = 0;

  if ( 0 != m_root )
    CountRec(m_root, count);
  
  return count;
}

const ON_RTreeNode* ON_RTree::Root() const
{
  return m_root;
}

Bnd_Box ON_RTree::BoundingBox() const
{
	double* m_min;
	double* m_max;
  if ( 0 != m_root && m_root->m_count > 0 )
  {
    m_min = m_root->m_branch[0].m_rect.m_min;
    m_max = m_root->m_branch[0].m_rect.m_max;
    for ( int i = 1; i < m_root->m_count; i++ )
    {
      if ( m_root->m_branch[i].m_rect.m_min[0] < m_min[0] )
        m_min[0] = m_root->m_branch[i].m_rect.m_min[0];
      if ( m_root->m_branch[i].m_rect.m_min[1] < m_min[1] )
        m_min[1] = m_root->m_branch[i].m_rect.m_min[1];
      if ( m_root->m_branch[i].m_rect.m_min[2] < m_min[2] )
        m_min[2] = m_root->m_branch[i].m_rect.m_min[2];

      if ( m_root->m_branch[i].m_rect.m_max[0] > m_max[0] )
        m_max[0] = m_root->m_branch[i].m_rect.m_max[0];
      if ( m_root->m_branch[i].m_rect.m_max[1] > m_max[1] )
        m_max[1] = m_root->m_branch[i].m_rect.m_max[1];
      if ( m_root->m_branch[i].m_rect.m_max[2] > m_max[2] )
        m_max[2] = m_root->m_branch[i].m_rect.m_max[2];
    }
  }
   Bnd_Box box;
   box.Update(m_min[0], m_min[1], m_min[2], m_max[0], m_max[1], m_max[2]);
   return box;
}

static void CountRec(ON_RTreeNode* a_node, int& a_count)
{
  if(a_node->IsInternalNode())  // not a leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      CountRec(a_node->m_branch[index].m_child, a_count);
    }
  }
  else // A leaf node
  {
    a_count += a_node->m_count;
  }
}

void ON_RTree::RemoveAll()
{
  RemoveAllRec(m_root);
  m_root = 0;
}

void ON_RTree::RemoveAllRec(ON_RTreeNode* a_node)
{
  if(a_node->IsInternalNode()) // This is an internal node in the tree
  {
    for(int index=0; index < a_node->m_count; ++index)
    {
      RemoveAllRec(a_node->m_branch[index].m_child);
    }
  }
  FreeNode(a_node); 
}



static void InitRect(ON_RTreeBBox* a_rect)
{
  for(int index = 0; index < 3; ++index)
  {
    a_rect->m_min[index] = (double)0;
    a_rect->m_max[index] = (double)0;
  }
}


// Inserts a new data rectangle into the index structure.
// Recursively descends tree, propagates splits back up.
// Returns 0 if node was not split.  Old node updated.
// If node was split, returns 1 and sets the pointer pointed to by
// new_node to point to the new node.  Old node updated to become one of two.
// The level argument specifies the number of steps up from the leaf
// level to insert; e.g. a data rectangle goes in at level = 0.

bool ON_RTree::InsertRectRec(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode* a_node, ON_RTreeNode** a_newNode, int a_level)
{
  int index;
  ON_RTreeBranch branch;
  ON_RTreeNode* otherNode;

  // Still above level for insertion, go down tree recursively
  if(a_node->m_level > a_level)
  {
    index = PickBranch(a_rect, a_node);
    if ( index < 0 )
    {
      return false;
    }
    if (!InsertRectRec(a_rect, a_id, a_node->m_branch[index].m_child, &otherNode, a_level))
    {
      // Child was not split
      a_node->m_branch[index].m_rect = CombineRectHelper(a_rect, &(a_node->m_branch[index].m_rect));
      return false;
    }
    else // Child was split
    {
      a_node->m_branch[index].m_rect = NodeCover(a_node->m_branch[index].m_child);
      branch.m_child = otherNode;
      branch.m_rect = NodeCover(otherNode);
      return AddBranch(&branch, a_node, a_newNode);
    }
  }
  else if(a_node->m_level == a_level) // Have reached level for insertion. Add rect, split if necessary
  {
    branch.m_rect = *a_rect;

    // The (ON_RTreeNode*) cast is safe because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_PUSH
// Disable warning C4312: 'type cast' : conversion from 'ON__INT_PTR' to 'ON_RTreeNode *' of greater size
#pragma ON_PRAGMA_WARNING_DISABLE_MSC( 4312 )
#endif
    branch.m_child = (ON_RTreeNode*)a_id;
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_POP
#endif

    // Child field of leaves contains id of data record
    return AddBranch(&branch, a_node, a_newNode);
  }

  // We should never get here
 // ON_ERROR("ON_RTree::InsertRectRec - bug in algorithm");
  return false;
}


// Insert a data rectangle into an index structure.
// InsertRect provides for splitting the root;
// returns 1 if root was split, 0 if it was not.
// The level argument specifies the number of steps up from the leaf
// level to insert; e.g. a data rectangle goes in at level = 0.
// InsertRect2 does the recursion.
//

bool ON_RTree::InsertRect(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode** a_root, int a_level)
{
  ON_RTreeNode* newRoot;
  ON_RTreeNode* newNode;
  ON_RTreeBranch branch;

  if(InsertRectRec(a_rect, a_id, *a_root, &newNode, a_level))  // Root split
  {
    newRoot = AllocNode();  // Grow tree taller and new root
    newRoot->m_level = (*a_root)->m_level + 1;
    branch.m_rect = NodeCover(*a_root);
    branch.m_child = *a_root;
    AddBranch(&branch, newRoot, nullptr);
    branch.m_rect = NodeCover(newNode);
    branch.m_child = newNode;
    AddBranch(&branch, newRoot, nullptr);
    *a_root = newRoot;
    return true;
  }

  return false;
}


// Find the smallest rectangle that includes all rectangles in branches of a node.

static ON_RTreeBBox NodeCover(ON_RTreeNode* a_node)
{
  int i;
  const ON_RTreeBranch* branch;
  ON_RTreeBBox rect;

  if ( (i = a_node->m_count) > 0 )
  {
    rect = a_node->m_branch[--i].m_rect;
    for ( branch = a_node->m_branch; i; i--, branch++ )
    {
#if (3 == 3)
      if ( rect.m_min[0] > branch->m_rect.m_min[0] )
        rect.m_min[0] = branch->m_rect.m_min[0];
      if ( rect.m_min[1] > branch->m_rect.m_min[1] )
        rect.m_min[1] = branch->m_rect.m_min[1];
      if ( rect.m_min[2] > branch->m_rect.m_min[2] )
        rect.m_min[2] = branch->m_rect.m_min[2];
      if ( rect.m_max[0] < branch->m_rect.m_max[0] )
        rect.m_max[0] = branch->m_rect.m_max[0];
      if ( rect.m_max[1] < branch->m_rect.m_max[1] )
        rect.m_max[1] = branch->m_rect.m_max[1];
      if ( rect.m_max[2] < branch->m_rect.m_max[2] )
        rect.m_max[2] = branch->m_rect.m_max[2];
#else
      for( int j = 0; j < 3; j++ )
      {
        if ( rect.m_min[j] > branch->m_rect.m_min[j] )
          rect.m_min[j] = branch->m_rect.m_min[j];
        if ( rect.m_max[j] < branch->m_rect.m_max[j] )
          rect.m_max[j] = branch->m_rect.m_max[j];
      }
#endif
    }
  }
  else
  {
    InitRect(&rect);
  }

  return rect;
}


// Add a branch to a node.  Split the node if necessary.
// Returns 0 if node not split.  Old node updated.
// Returns 1 if node split, sets *new_node to address of new node.
// Old node updated, becomes one of two.

bool ON_RTree::AddBranch(ON_RTreeBranch* a_branch, ON_RTreeNode* a_node, ON_RTreeNode** a_newNode)
{
  if(a_node->m_count < ON_RTree_MAX_NODE_COUNT)  // Split won't be necessary
  {
    a_node->m_branch[a_node->m_count] = *a_branch;
    ++a_node->m_count;

    return false;
  }
  else
  {
    SplitNode(a_node, a_branch, a_newNode);
    return true;
  }
}


// Disconnect a dependent node.
// Caller must return (or stop using iteration index) after this as count has changed

static void DisconnectBranch(ON_RTreeNode* a_node, int a_index)
{
  // Remove element by swapping with the last element to prevent gaps in array
  a_node->m_branch[a_index] = a_node->m_branch[a_node->m_count - 1];
  
  --a_node->m_count;
}


// Pick a branch.  Pick the one that will need the smallest increase
// in area to accomodate the new rectangle.  This will result in the
// least total area for the covering rectangles in the current node.
// In case of a tie, pick the one which was smaller before, to get
// the best resolution when searching.

static int PickBranch(ON_RTreeBBox* a_rect, ON_RTreeNode* a_node)
{
  bool firstTime = true;
  double increase;
  double bestIncr = -1.0;
  double area;
  double bestArea;
  int best;
  ON_RTreeBBox tempRect;

  best = -1;
  bestArea = -1.0;

  for(int index=0; index < a_node->m_count; ++index)
  {
    ON_RTreeBBox* curRect = &a_node->m_branch[index].m_rect;
    area = CalcRectVolumeHelper(curRect);
    tempRect = CombineRectHelper(a_rect, curRect);
    increase = CalcRectVolumeHelper(&tempRect) - area;
    if((increase < bestIncr) || firstTime)
    {
      best = index;
      bestArea = area;
      bestIncr = increase;
      firstTime = false;
    }
    else if((increase == bestIncr) && (area <=bestArea))
    {
      best = index;
      bestArea = area;
      bestIncr = increase;
    }
  }
  return best;
}


// Combine two rectangles into larger one containing both

ON_RTreeBBox CombineRectHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB)
{
  ON_RTreeBBox rect = *a_rectA;

#if (3 == 3)
  if ( rect.m_min[0] > a_rectB->m_min[0] )
    rect.m_min[0] = a_rectB->m_min[0];
  if ( rect.m_min[1] > a_rectB->m_min[1] )
    rect.m_min[1] = a_rectB->m_min[1];
  if ( rect.m_min[2] > a_rectB->m_min[2] )
    rect.m_min[2] = a_rectB->m_min[2];
  if ( rect.m_max[0] < a_rectB->m_max[0] )
    rect.m_max[0] = a_rectB->m_max[0];
  if ( rect.m_max[1] < a_rectB->m_max[1] )
    rect.m_max[1] = a_rectB->m_max[1];
  if ( rect.m_max[2] < a_rectB->m_max[2] )
    rect.m_max[2] = a_rectB->m_max[2];
#else
  for( int j = 0; j < 3; j++ )
  {
    if ( rect.m_min[j] > a_rectB->m_min[j] )
      rect.m_min[j] = a_rectB->m_min[j];
    if ( rect.m_max[j] < a_rectB->m_max[j] )
      rect.m_max[j] = a_rectB->m_max[j];
  }
#endif

  return rect;
}



// Split a node.
// Divides the nodes branches and the extra one between two nodes.
// Old node is one of the new ones, and one really new one is created.
// Tries more than one method for choosing a partition, uses best result.

void ON_RTree::SplitNode(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreeNode** a_newNode)
{
  ON_RTreePartitionVars localVars;
  int level;

  // Load all the branches into a buffer, initialize a_node to be empty
  level = a_node->m_level; // save m_level (The InitNode() call in GetBranches will set it to -1)
  GetBranches(a_node, a_branch, &localVars);

  // Find partition
  ChoosePartition(&localVars, ON_RTree_MIN_NODE_COUNT);

  // Put branches from buffer into 2 nodes according to chosen partition
  *a_newNode = AllocNode();
  (*a_newNode)->m_level = a_node->m_level = level; // restore m_level
  LoadNodes(a_node, *a_newNode, &localVars);
}

double CalcRectVolumeHelper(const ON_RTreeBBox* a_rect)
{
  double d, r;

  // Bounding sphere volume calculation is slower, but helps certain merge cases
#if ( 3 == ON_RTree_NODE_DIM)
  // 3d bounding sphere volume
  d = (a_rect->m_max[0] - a_rect->m_min[0]);
  r = d * d;
  d = (a_rect->m_max[1] - a_rect->m_min[1]);
  r += d * d;
  d = (a_rect->m_max[2] - a_rect->m_min[2]);
  r += d * d;
  //r = sqrt(r*0.5); // r = sqrt((dx^2 + dy^2 + dz^2)/2);
  //return (r * r * r * 4.1887902047863909846168578443727); // 4/3 pi r^3
  return r;
#elif ( 2 == ON_RTree_NODE_DIM )
  // 2d bounding circle volume
  d = (a_rect->m_max[0] - a_rect->m_min[0]);
  r = d * d;
  d = (a_rect->m_max[1] - a_rect->m_min[1]);
  r += d * d;
  r = sqrt(r*0.5); // r = sqrt((dx^2 + dy^2)/2);
  return (r * r * ON_PI);
#else

  // n-dim unit sphere volumes
  //  0.000000f, 2.000000f, 3.141593f, // Dimension  0,1,2
  //  4.188790f, 4.934802f, 5.263789f, // Dimension  3,4,5
  //  5.167713f, 4.724766f, 4.058712f, // Dimension  6,7,8
  //  3.298509f, 2.550164f, 1.884104f, // Dimension  9,10,11
  //  1.335263f, 0.910629f, 0.599265f, // Dimension  12,13,14
  //  0.381443f, 0.235331f, 0.140981f, // Dimension  15,16,17
  //  0.082146f, 0.046622f, 0.025807f, // Dimension  18,19,20 
  //return (unit_sphere_volume * radius^ON_RTree_NODE_DIM);

  // Faster rectangle volume calculation, but can cause poor merges
  d = a_rect->m_max[0] - a_rect->m_min[0];
  for(int i = 1; i < 3; ++i)
  {
    d *= a_rect->m_max[i] - a_rect->m_min[i];
  }
  return d;
#endif
}


// Load branch buffer with branches from full node plus the extra branch.

static void GetBranches(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreePartitionVars* a_parVars)
{
  // Load the branch buffer
  for(int index=0; index < ON_RTree_MAX_NODE_COUNT; ++index)
  {
    a_parVars->m_branchBuf[index] = a_node->m_branch[index];
  }
  a_parVars->m_branchBuf[ON_RTree_MAX_NODE_COUNT] = *a_branch;
  a_parVars->m_branchCount = ON_RTree_MAX_NODE_COUNT + 1;

  // Calculate rect containing all in the set
  a_parVars->m_coverSplit = a_parVars->m_branchBuf[0].m_rect;
  for(int index=1; index < ON_RTree_MAX_NODE_COUNT+1; ++index)
  {
    a_parVars->m_coverSplit = CombineRectHelper(&a_parVars->m_coverSplit, &a_parVars->m_branchBuf[index].m_rect);
  }
  a_parVars->m_coverSplitArea = CalcRectVolumeHelper(&a_parVars->m_coverSplit);

  a_node->m_count = 0;
  a_node->m_level = -1;
}


// Method #0 for choosing a partition:
// As the seeds for the two groups, pick the two rects that would waste the
// most area if covered by a single rectangle, i.e. evidently the worst pair
// to have in the same group.
// Of the remaining, one at a time is chosen to be put in one of the two groups.
// The one chosen is the one with the greatest difference in area expansion
// depending on which group - the rect most strongly attracted to one group
// and repelled from the other.
// If one group gets too full (more would force other group to violate min
// fill requirement) then other group gets the rest.
// These last are the ones that can go in either group most easily.

static void ChoosePartition(ON_RTreePartitionVars* a_parVars, int a_minFill)
{
  double biggestDiff;
  int group, chosen, betterGroup;
  
  InitParVars(a_parVars, a_parVars->m_branchCount, a_minFill);
  PickSeeds(a_parVars);

  while (((a_parVars->m_count[0] + a_parVars->m_count[1]) < a_parVars->m_total)
       && (a_parVars->m_count[0] < (a_parVars->m_total - a_parVars->m_minFill))
       && (a_parVars->m_count[1] < (a_parVars->m_total - a_parVars->m_minFill)))
  {
    biggestDiff = -1.0;
    chosen = 0;
    betterGroup = 0;
    for(int index=0; index<a_parVars->m_total; ++index)
    {
      if(!a_parVars->m_taken[index])
      {
        ON_RTreeBBox* curRect = &a_parVars->m_branchBuf[index].m_rect;
        ON_RTreeBBox rect0 = CombineRectHelper(curRect, &a_parVars->m_cover[0]);
        ON_RTreeBBox rect1 = CombineRectHelper(curRect, &a_parVars->m_cover[1]);
        double growth0 = CalcRectVolumeHelper(&rect0) - a_parVars->m_area[0];
        double growth1 = CalcRectVolumeHelper(&rect1) - a_parVars->m_area[1];
        double diff = growth1 - growth0;
        if(diff >= 0)
        {
          group = 0;
        }
        else
        {
          group = 1;
          diff = -diff;
        }

        if(diff > biggestDiff)
        {
          biggestDiff = diff;
          chosen = index;
          betterGroup = group;
        }
        else if((diff == biggestDiff) && (a_parVars->m_count[group] < a_parVars->m_count[betterGroup]))
        {
          chosen = index;
          betterGroup = group;
        }
      }
    }
    ClassifyHelper(chosen, betterGroup, a_parVars);
  }

  // If one group too full, put remaining rects in the other
  if((a_parVars->m_count[0] + a_parVars->m_count[1]) < a_parVars->m_total)
  {
    if(a_parVars->m_count[0] >= a_parVars->m_total - a_parVars->m_minFill)
    {
      group = 1;
    }
    else
    {
      group = 0;
    }
    for(int index=0; index<a_parVars->m_total; ++index)
    {
      if(!a_parVars->m_taken[index])
      {
        ClassifyHelper(index, group, a_parVars);
      }
    }
  }
}


// Copy branches from the buffer into two nodes according to the partition.

void ON_RTree::LoadNodes(ON_RTreeNode* a_nodeA, ON_RTreeNode* a_nodeB, ON_RTreePartitionVars* a_parVars)
{
  for(int index=0; index < a_parVars->m_total; ++index)
  {
    if(a_parVars->m_partition[index] == 0)
    {
      AddBranch(&a_parVars->m_branchBuf[index], a_nodeA, nullptr);
    }
    else if(a_parVars->m_partition[index] == 1)
    {
      AddBranch(&a_parVars->m_branchBuf[index], a_nodeB, nullptr);
    }
  }
}


// Initialize a ON_RTreePartitionVars structure.

static void InitParVars(ON_RTreePartitionVars* a_parVars, int a_maxRects, int a_minFill)
{
  a_parVars->m_count[0] = a_parVars->m_count[1] = 0;
  a_parVars->m_area[0] = a_parVars->m_area[1] = (double)0;
  a_parVars->m_total = a_maxRects;
  a_parVars->m_minFill = a_minFill;
  for(int index=0; index < a_maxRects; ++index)
  {
    a_parVars->m_taken[index] = false;
    a_parVars->m_partition[index] = -1;
  }
}



static void PickSeeds(ON_RTreePartitionVars* a_parVars)
{
  int seed0 = 0, seed1 = 1;
  double worst, waste;
  double area[ON_RTree_MAX_NODE_COUNT+1];

  for(int index=0; index<a_parVars->m_total; ++index)
  {
    area[index] = CalcRectVolumeHelper(&a_parVars->m_branchBuf[index].m_rect);
  }

  worst = -a_parVars->m_coverSplitArea - 1;
  for(int indexA=0; indexA < a_parVars->m_total-1; ++indexA)
  {
    for(int indexB = indexA+1; indexB < a_parVars->m_total; ++indexB)
    {
      ON_RTreeBBox oneRect = CombineRectHelper(&a_parVars->m_branchBuf[indexA].m_rect, &a_parVars->m_branchBuf[indexB].m_rect);
      waste = CalcRectVolumeHelper(&oneRect) - area[indexA] - area[indexB];
      if(waste > worst)
      {
        worst = waste;
        seed0 = indexA;
        seed1 = indexB;
      }
    }
  }
  ClassifyHelper(seed0, 0, a_parVars);
  ClassifyHelper(seed1, 1, a_parVars);
}

// Put a branch in one of the groups.

void ClassifyHelper(int a_index, int a_group, ON_RTreePartitionVars* a_parVars)
{
  a_parVars->m_partition[a_index] = a_group;
  a_parVars->m_taken[a_index] = true;

  if (a_parVars->m_count[a_group] == 0)
  {
    a_parVars->m_cover[a_group] = a_parVars->m_branchBuf[a_index].m_rect;
  }
  else
  {
    a_parVars->m_cover[a_group] = CombineRectHelper(&a_parVars->m_branchBuf[a_index].m_rect, &a_parVars->m_cover[a_group]);
  }
  a_parVars->m_area[a_group] = CalcRectVolumeHelper(&a_parVars->m_cover[a_group]);
  ++a_parVars->m_count[a_group];
}


// Delete a data rectangle from an index structure.
// Pass in a pointer to a ON_RTreeBBox, the tid of the record, ptr to ptr to root node.
// Returns 1 if record not found, 0 if success.
// RemoveRect provides for eliminating the root.

bool ON_RTree::RemoveRect(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode** a_root)
{
  ON_RTreeNode* tempNode;
  ON_RTreeListNode* reInsertList = nullptr;

  if(!RemoveRectRec(a_rect, a_id, *a_root, &reInsertList))
  {
    // Found and deleted a data item
    // Reinsert any branches from eliminated nodes
    while(reInsertList)
    {
      tempNode = reInsertList->m_node;

      for(int index = 0; index < tempNode->m_count; ++index)
      {
        InsertRect(&(tempNode->m_branch[index].m_rect),
                   tempNode->m_branch[index].m_id,
                   a_root,
                   tempNode->m_level);
      }
      
      ON_RTreeListNode* remLNode = reInsertList;
      reInsertList = reInsertList->m_next;
      
      FreeNode(remLNode->m_node);
      FreeListNode(remLNode);
    }
    
    // Check for redundant root (not leaf, 1 child) and eliminate
    if((*a_root)->m_count == 1 && (*a_root)->IsInternalNode())
    {
      tempNode = (*a_root)->m_branch[0].m_child;
      FreeNode(*a_root);
      *a_root = tempNode;
    }
    return false;
  }
  else
  {
    return true;
  }
}


// Delete a rectangle from non-root part of an index structure.
// Called by RemoveRect.  Descends tree recursively,
// merges branches on the way back up.
// Returns 1 if record not found, 0 if success.

bool ON_RTree::RemoveRectRec(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode* a_node, ON_RTreeListNode** a_listNode)
{
  if(a_node->IsInternalNode())  // not a leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      if(OverlapHelper(a_rect, &(a_node->m_branch[index].m_rect)))
      {
        if(!RemoveRectRec(a_rect, a_id, a_node->m_branch[index].m_child, a_listNode))
        {
          if(a_node->m_branch[index].m_child->m_count >= ON_RTree_MIN_NODE_COUNT)
          {
            // child removed, just resize parent rect
            a_node->m_branch[index].m_rect = NodeCover(a_node->m_branch[index].m_child);
          }
          else
          {
            // child removed, not enough entries in node, eliminate node
            ReInsert(a_node->m_branch[index].m_child, a_listNode);
            DisconnectBranch(a_node, index); // Must return after this call as count has changed
          }
          return false;
        }
      }
    }
    return true;
  }
  else // A leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      if(a_node->m_branch[index].m_id == a_id)
      {
        DisconnectBranch(a_node, index); // Must return after this call as count has changed
        return false;
      }
    }
    return true;
  }
}


// Decide whether two rectangles overlap.
bool OverlapHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB)
{
  const double* mn;
  const double* mx;

  mx = a_rectA->m_max;
  mn = a_rectB->m_min;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx   < *mn   ) return false;

  mx = a_rectB->m_max;
  mn = a_rectA->m_min;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx   < *mn   ) return false;

  return true;
}

static double DistanceToBoxHelper( 
  const double* pt, 
  double r, 
  const ON_RTreeBBox* a_rect
  )
{
  // If the sphere with center at pt and radius r intersects a_rect, then
  // the distance from pt to a_rect is returned. A value of 0.0 indicates
  // that pt is inside a_rect.  If the distance from pt to a_rect is
  // greater than r, then some number > r and <= actual distance from
  // pt to a_rect is returned as quickly as possible.

  double d[3], t;
  const double* mn;
  const double* mx;

  mn = a_rect->m_min;
  mx = a_rect->m_max;

  if ( *pt < *mn )
  {
    d[0] = *mn - *pt;
    if ( d[0] > r )
      return d[0];
  }
  else if ( *pt > *mx )
  {
    d[0] = *pt - *mx;
    if ( d[0] > r )
      return d[0];
  }
  else
  {
    d[0] = 0.0;
  }

  mn++;
  mx++;
  pt++;
  if ( *pt < *mn )
  {
    d[1] = *mn - *pt;
    if ( d[1] > r )
      return d[1];
    if ( d[1] > d[0] )
    {
      t = d[0]; d[0] = d[1]; d[1] = t;
    }
  }
  else if ( *pt > *mx )
  {
    d[1] = *pt - *mx;
    if ( d[1] > r )
      return d[1];
    if ( d[1] > d[0] )
    {
      t = d[0]; d[0] = d[1]; d[1] = t;
    }
  }
  else
  {
    d[1] = 0.0;
  }

  mn++;
  mx++;
  pt++;
  if ( *pt < *mn )
  {
    d[2] = *mn - *pt;
    if ( d[2] > r )
      return d[2];
    if ( d[2] > d[0] )
    {
      t = d[0]; d[0] = d[2]; d[2] = t;
    }
  }
  else if ( *pt > *mx )
  {
    d[2] = *pt - *mx;
    if ( d[2] > r )
      return d[2];
    if ( d[2] > d[0] )
    {
      t = d[0]; d[0] = d[2]; d[2] = t;
    }
  }
  else
  {
    d[2] = 0.0;
  }

  if ( d[0] > 0.0 )
  {
    d[1] /= d[0];
    d[2] /= d[0];
    d[0] *= sqrt(1.0 + d[1]*d[1] + d[2]*d[2]);
  }

  return d[0];
}

// Add a node to the reinsertion list.  All its branches will later
// be reinserted into the index structure.

void ON_RTree::ReInsert(ON_RTreeNode* a_node, ON_RTreeListNode** a_listNode)
{
  ON_RTreeListNode* newListNode;

  newListNode = AllocListNode();
  newListNode->m_node = a_node;
  newListNode->m_next = *a_listNode;
  *a_listNode = newListNode;
}


static
bool OverlapBoundedPlaneXYZHelper( const double* a_bounded_plane, const ON_RTreeBBox* a_rect )
{
  unsigned char flag = 0;
  double x, y, z, d, v;
  
  // check the 8 corners of the box minimizing the number of evaluations
  // and unrolling the loop for speed

  // corner = (min, min, min)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  y = a_bounded_plane[1]*a_rect->m_min[1];
  z = a_bounded_plane[2]*a_rect->m_min[2];
  d = a_bounded_plane[3];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
    flag = 1;
  else if ( v > a_bounded_plane[5] )
    flag = 2;
  else
    return true;
  
  // corner = (max, min, min)
  x = a_bounded_plane[0]*a_rect->m_max[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // corner = (max, max, min)
  y = a_bounded_plane[1]*a_rect->m_max[1];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
    
  // corner = (max, max, max)
  z = a_bounded_plane[2]*a_rect->m_max[2];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (min, max, max)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // corner = (min, min, max)
  y = a_bounded_plane[1]*a_rect->m_min[1];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (max, min, max)
  x = a_bounded_plane[0]*a_rect->m_max[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (min, max, min)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  y = a_bounded_plane[1]*a_rect->m_max[1];
  z = a_bounded_plane[2]*a_rect->m_min[2];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // Either all 8 box corners 
  // are below the min plane (flag=1)
  // or above the max plane (flag=2).
  return false;
}

static
bool SearchBoundedPlaneXYZHelper(const ON_RTreeNode* a_node, const double* a_bounded_plane, ON_RTreeSearchResultCallback& a_result ) 
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapBoundedPlaneXYZHelper(a_bounded_plane, &branch[i].m_rect))
        {
          if(!SearchBoundedPlaneXYZHelper(branch[i].m_child, a_bounded_plane, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapBoundedPlaneXYZHelper(a_bounded_plane, &branch[i].m_rect))
        {
          if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
          {
            // callback canceled search
            return false;
          }
        }
      }
    }
  }

  return true; // Continue searching
}

bool ON_RTree::Search(
  const double a_plane_eqn[4],
  double a_min,
  double a_max,
  bool  a_resultCallback(void* a_context, ON__INT_PTR a_id),
  void* a_context
  ) const
{
  if (    0 == m_root 
       || 0 == a_plane_eqn 
       || !(a_min <= a_max) 
       || (0.0 == a_plane_eqn[0] && 0.0 == a_plane_eqn[1] && 0.0 == a_plane_eqn[2])
     )
    return false;

  double bounded_plane[6];
  bounded_plane[0] = a_plane_eqn[0];
  bounded_plane[1] = a_plane_eqn[1];
  bounded_plane[2] = a_plane_eqn[2];
  bounded_plane[3] = a_plane_eqn[3];
  bounded_plane[4] = a_min;
  bounded_plane[5] = a_max;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;

  return SearchBoundedPlaneXYZHelper(m_root, bounded_plane, result);
}

// Search in an index tree or subtree for all data retangles that overlap the argument rectangle.

static
bool SearchHelper(const ON_RTreeNode* a_node, ON_RTreeBBox* a_rect, ON_RTreeSearchResultCallback& a_result ) 
{
  // NOTE: 
  //  Some versions of ON_RTree::Search shrink a_rect as the search progresses.
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
          {
            // callback canceled search
            return false;
          }
        }
      }
    }
  }

  return true; // Continue searching
}

static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<ON_RTreeLeaf> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return point to the branch
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
		  a_result.emplace_back(branch[i].m_rect, branch[i].m_id);
        }
      }
    }
  }

  return true; // Continue searching
}


static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<void*> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          // The (void*) cast is safe because branch[i].m_id is an ON__INT_PTR
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_PUSH
// Disable warning C4312: 'type cast' : conversion from 'const ON__INT_PTR' to 'void *' of greater size
#pragma ON_PRAGMA_WARNING_DISABLE_MSC( 4312 )
#endif
          a_result.emplace_back((void*)branch[i].m_id );
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma ON_PRAGMA_WARNING_POP
#endif
        }
      }
    }
  }

  return true; // Continue searching
}

static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, std::vector<int> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          a_result.emplace_back((int)branch[i].m_id );
        }
      }
    }
  }

  return true; // Continue searching
}

static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_RTreeSearchResult& a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if ( a_result.m_count >= a_result.m_capacity )
            return false; // No more space for results
          a_result.m_id[a_result.m_count++] = branch[i].m_id;
        }
      }
    }
  }

  return true; // Continue searching
}

