// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GVertex.cpp
//   Created May 03, 2011
// ----------------------------------------------------------------------

#include "GVertex.hpp"
#include "GDEdge.hpp"

namespace cmpex {

//=======================================================================
/*
 * Constructors and destructor
 */

GVertex::GVertex( int idx, int cidx, int ridx ) : 
  idx_ (idx), colIdx_ (cidx), rowIdx_ (ridx),
  northIn_ (0), northOut_ (0), westIn_ (0), westOut_ (0),
  southIn_ (0), southOut_ (0), eastIn_ (0), eastOut_ (0) {}

GVertex::~GVertex() {}

//=======================================================================

}; // namespace cmpex
