// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GGraph.cpp
//   Created May 03, 2011
// ----------------------------------------------------------------------

#include "GGraph.hpp"
#include "GVertex.hpp"
#include "GDEdge.hpp"

#include <iostream>

using std::cout;
using std::endl;

namespace cmpex {

//=======================================================================
/*
 * Constructors and destructor
 */

GGraph::GGraph() : vCnt_ (0), eCnt_ (0) {}

GGraph::~GGraph() {
  for (VCIter it = vertices_.begin(); it != vertices_.end(); ++it) {
    delete *it;
  }
  
  for (ECIter it = edges_.begin(); it != edges_.end(); ++it) {
    delete *it;
  }
}

//=======================================================================
/*
 * Factory method for vertex: creates GVertex object. 
 */

GVertex * GGraph::CreateVertex ( int idx )
{
  cout << "Vertex created" << endl;
  return new GVertex(idx, idx%ColNum(), idx/ColNum());
}

//=======================================================================
/*
 * Factory method for edge: creates GDEdge object. 
 */

GDEdge * GGraph::CreateEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal )
{
  cout << "Edge created" << endl;
  return new GDEdge(idx, src, dst, horizontal);
}

//=======================================================================
/*
 * Initializer method, creates required number of cols and rows. 
 */

void GGraph::Init ( int colNum, int rowNum )
{
  colNum_ = colNum;
  rowNum_ = rowNum;
  
  int vNum = colNum * rowNum;
  for (int i = 0; i < vNum; ++i) {
    AddVertex(CreateVertex(i));
  }
  
  ConnectVertices();
}

//=======================================================================
/*
 * Create edges and connect vertices.
 */

void GGraph::ConnectVertices () {
  for (VCIter it = vertices_.begin(); it != vertices_.end(); ++it) {
    GVertex * v = *it;

    // create horizontal edges
    if ( v->Idx()%ColNum() != ColNum()-1 ) {
      GVertex * cv = vertices_[v->Idx() + 1];
      GDEdge * eout = CreateEdge(edges_.size(), v, cv, true);
      GDEdge * ein = CreateEdge(edges_.size()+1, cv, v, true);
      v->EastOut(eout);
      v->EastIn(ein);
      cv->WestOut(ein);
      cv->WestIn(eout);
      AddEdge(eout);
      AddEdge(ein);
    }
    
    // create vertical edges
    if ( v->Idx()/ColNum() != RowNum()-1 ) {
      GVertex * cv = vertices_[v->Idx() + ColNum()];
      GDEdge * eout = CreateEdge(edges_.size(), v, cv, true);
      GDEdge * ein = CreateEdge(edges_.size()+1, cv, v, true);
      v->SouthOut(eout);
      v->SouthIn(ein);
      cv->NorthOut(ein);
      cv->NorthIn(eout);
      AddEdge(eout);
      AddEdge(ein);
    }
  }
}

//=======================================================================
/*
 * Prints graph.
 */

void GGraph::Print () const {
  for (VCIter it = vertices_.begin(); it != vertices_.end(); ++it) {
    GVertex * v = *it;
    std::cout << "Vertex " << v->Idx() << "; neighbours:";
    if (v->NorthOut()) {
      std::cout << " North " << v->NorthOut()->Dst()->Idx() << ";";
    }
    if (v->WestOut()) {
      std::cout << " West " << v->WestOut()->Dst()->Idx() << ";";
    }
    if (v->SouthOut()) {
      std::cout << " South " << v->SouthOut()->Dst()->Idx() << ";";
    }
    if (v->EastOut()) {
      std::cout << " East " << v->EastOut()->Dst()->Idx() << ";";
    }
    std::cout << std::endl;
  }
}

//=======================================================================

}; // namespace cmpex
