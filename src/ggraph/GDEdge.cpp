// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GDEdge.cpp
//   Created May 05, 2011
// ----------------------------------------------------------------------

#include "GDEdge.hpp"

namespace cmpex {

//=======================================================================
/*
 * Constructors and destructor
 */

GDEdge::GDEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal ) :
  idx_ (idx), src_ (src), dst_ (dst), horizontal_(horizontal) {}

GDEdge::~GDEdge () {}

//=======================================================================

}; // namespace cmpex
