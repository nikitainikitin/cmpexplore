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

#include "Memory.hpp"
#include "CmpConfig.hpp"

#include <iostream>

using std::cout;
using std::endl;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  
  namespace cmp {
    
//=======================================================================
/*
 * Constructors and destructor
 */

Memory::Memory ( UShort idx, UShort clIdx, cmp::MemType mt, Component * parent ) :
  Device (idx, clIdx, CTMEMORY, parent), mtype_ (mt), size_ (0.0), lat_ (0.0),
  mcIdx_ (-1), lambda_ (0.0), bufDelay_ (0.0), active_ (true) {}

Memory::~Memory () {}

//=======================================================================
/*
 * Returns true if component contains the processor 'idx'.
 * Always false for a memory.
 */

bool Memory::HasProcessor (UShort idx)
{
  return false;
}

//=======================================================================
/*
 * Returns true if component contains the memory 'idx'. Only true when
 * component IS the memory being enquired for (i.e. has the same index).
 */

bool Memory::HasMemory (UShort idx)
{
  return (idx == Idx()) ?  true : false;
}

//=======================================================================
/*
 * Returns hop-count distance from processor 'pIdx' to the Iface
 * component.
 */

int Memory::DistanceProcToIface (UShort pIdx)
{
  return MAX_INT;
}

//=======================================================================
/*
 * Returns hop-count distance from memory 'mIdx' to the Iface
 * component.
 */

int Memory::DistanceMemToIface (UShort mIdx)
{
  return (mIdx == Idx()) ? 0 : MAX_INT;
}

//=======================================================================
/*
 * Returns uni-directional latency from the processor 'pIdx'
 * to the interface of component.
 * Always MAX_DOUBLE for a memory (i.e. processor not accessible).
 */

double Memory::ULatProcToIface (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from the interface of component
 * to the processor 'pIdx'.
 * Always MAX_DOUBLE for a memory (i.e. processor not accessible).
 */

double Memory::ULatIfaceToProc (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory 'mIdx'
 * to the interface of component.
 */

double Memory::ULatMemToIface (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return (mIdx == Idx()) ? 0.0 : MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from the interface of component
 * to the memory 'mIdx'.
 */

double Memory::ULatIfaceToMem (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return (mIdx == Idx()) ? 0.0 : MAX_DOUBLE;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
