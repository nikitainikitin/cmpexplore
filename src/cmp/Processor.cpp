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

#include <iostream>
#include <cmath>

#include "Processor.hpp"
#include "CmpConfig.hpp"

using std::cout;
using std::endl;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
    
  namespace cmp {

//=======================================================================
/*
 * Constructors and destructor
 */

Processor::Processor ( UShort idx, UShort clIdx, Component * parent ) :
  Device (idx, clIdx, CTPROCESSOR, parent), l1Size_ (0), l2Size_ (0),
  l3SizeEff_(0), active_ (true), thr_ (0.0), lambda_ (0.0), type_ (-1) {}

Processor::~Processor () {}

//=======================================================================
/*
 * Returns true if component contains the processor 'idx'. Only true when
 * component IS the processor being enquired for (i.e. has the same index).
 */

bool Processor::HasProcessor (UShort idx)
{
  return (idx == Idx()) ?  true : false;
}

//=======================================================================
/*
 * Returns true if component contains the memory 'idx'.
 * Always false for a processor.
 */

bool Processor::HasMemory (UShort idx)
{
  return false;
}

//=======================================================================
/*
 * Returns hop-count distance from processor 'pIdx' to the Iface
 * component.
 */

int Processor::DistanceProcToIface (UShort pIdx)
{
  return (pIdx == Idx()) ? 0 : MAX_INT;
}

//=======================================================================
/*
 * Returns hop-count distance from memory 'mIdx' to the Iface
 * component.
 */

int Processor::DistanceMemToIface (UShort mIdx)
{
  return MAX_INT;
}

//=======================================================================
/*
 * Returns uni-directional latency from the processor 'pIdx'
 * to the interface of component.
 */

double Processor::ULatProcToIface (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return (pIdx == Idx()) ? 0.0 : MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from interface of component
 * to the processor 'pIdx'.
 */

double Processor::ULatIfaceToProc (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return (pIdx == Idx()) ? 0.0 : MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory 'mIdx'
 * to the interface of component.
 * Always MAX_DOUBLE for a processor (i.e. memory not accessible).
 */

double Processor::ULatMemToIface (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return MAX_DOUBLE;
}

//=======================================================================
/*
 * Returns uni-directional latency from interface of component
 * to the memory 'mIdx'.
 * Always MAX_DOUBLE for a processor (i.e. memory not accessible).
 */

double Processor::ULatIfaceToMem (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  return MAX_DOUBLE;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
