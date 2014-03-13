#ifndef _DEFS_H_
#define _DEFS_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File Defs.hpp
//   Created May 03, 2011
// ----------------------------------------------------------------------

#include <cassert>
#include <limits>
#include <cstdlib>

namespace cmpex {

  //======================================================================
  // Common macros for cmpex
  //======================================================================

  #define DEBUG
  #ifdef DEBUG
    #define DASSERT(x) assert(x)
  #else
    #define DASSERT(x)
  #endif
  
  //======================================================================
  // Some min/max constants
  //======================================================================

  const double E_DOUBLE = 0.000001; // deviation for double presicion operations
  const double MIN_DOUBLE = std::numeric_limits<double>::min();
  const double MAX_DOUBLE = std::numeric_limits<double>::max();
  const double EMIN_DOUBLE = MIN_DOUBLE+E_DOUBLE; // e-close to min double
  const double EMAX_DOUBLE = MAX_DOUBLE-E_DOUBLE; // e-close to max double
  const int MIN_INT = std::numeric_limits<int>::min();
  const int MAX_INT = std::numeric_limits<int>::max();
  
  //======================================================================
  // Random numbers
  //======================================================================
  // Return uniformly distributed int from 0 to max-1.
  inline int RandInt ( int max )
  {
    return int( double(std::rand())/MAX_INT * max );
  }
  
  // Return uniformly distributed double from 0 to 1.
  inline double RandUDouble ()
  {
    return drand48();
  }
  
}; // namespace cmpex

#endif // _DEFS_H_
