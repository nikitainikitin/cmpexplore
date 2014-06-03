// ----------------------------------------------------------------------
//   Copyright 2011-2012 Nikita Nikitin <nikita.i.nikitin@gmail.com>
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
// ----------------------------------------------------------------------

#ifndef _DEFS_H_
#define _DEFS_H_

#include <cassert>
#include <limits>
#include <cstdlib>

namespace cmpex {

  //======================================================================
  // Common macros for cmpex
  //======================================================================

  //#define NDEBUG
  #ifdef NDEBUG
    #define DASSERT(x)
  #else
    #define DASSERT(x) assert(x)
  #endif
  
  #ifdef NDEBUG
    #define DEBUG(l, x)
  #else
    #define DEBUG(l, x) debug(l) << x
  #endif
  
  //======================================================================
  // Some types
  //======================================================================
  
  typedef unsigned int UShort;
  typedef unsigned int UInt;

  // a type with three states
  enum Tristate { TS_UNDEF = -1, TS_OFF, TS_ON };

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
  const UShort MAX_USHORT = std::numeric_limits<UShort>::max();
  const UInt MAX_UINT = std::numeric_limits<UInt>::max();
  
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
  
  //======================================================================
  // Definitions of the cmp package
  //======================================================================
  
  namespace cmp {
    
    // Component type
    enum CompType { CTPROCESSOR = 0, CTMEMORY, CTROUTER, CTDIRECTORY,
                    CTCLUSTER, NUMCT };
    
    // Memory type
    enum MemType { MTL2 = 0, MTL3, MTMAIN, NUMMT };
    
    // Interconnect type
    enum IcType { ITMESH = 0, ITBUS, ITURING, ITBRING, ITXBAR, NUMIT };
    
    // Router ports and directions.
    // RDIFACE is the direction in which different levels of hierarchy
    // are connected. It matches with one of the 'common' directions.
    // It is currently set to RDNORTH.
    enum RouteDir { RDNORTH = 0, RDIFACE = 0, RDWEST, RDSOUTH, RDEAST,
      RDPRIMARY, NUMRD };

    // Message type (cache-coherence protocol)
    // MSGNOCC - when CC is not considered, for compatibility
    enum MsgType { MSGNOCC = 0, MSGREQ = 0, MSGACK, MSGDATA, NUMMSG };

  }
  
} // namespace cmpex

#endif // _DEFS_H_
