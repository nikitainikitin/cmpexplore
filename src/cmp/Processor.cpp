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
#include "Memory.hpp"

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
  l3SizeEff_(0), active_ (true), plwidth_ (2), smtdegree_ (1), 
  thr_ (0.0), lambda_ (0.0), type_ (-1) {}

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
/*
 * Sets probabilities of memory access to the caches and MC.
 *
 * !!! NOTICE: this function has been tested in the -tmap mode,
 * under the assumption that L3 can be off and this information
 * is fetched from the memory activity flags.
 * It is assumed that every tile has one core and one L3 slice
 * and that the core and the L3 slice have the same index.
 */

void Processor::SetMemAccessProbabilities ( model::Function * mr ) {
  SetL1AccProb(1.0 - mr->eval(L1Size()));
  SetL2AccProb(mr->eval(L1Size()) - mr->eval(L2Size()+L1Size()));

  bool L3IsOn = cmpConfig.GetMemory(Idx())->Active();

  SetL3AccProb(L3SizeEff() < E_DOUBLE || !L3IsOn ? 0.0 :
               (mr->eval(L2Size()+L1Size()) -
                mr->eval(L3SizeEff()+L2Size()+L1Size())));
  SetMMAccProb(L3SizeEff() < E_DOUBLE || !L3IsOn ?
                mr->eval(L2Size()+L1Size()) :
                mr->eval(L3SizeEff()+L2Size()+L1Size()));

  SetL1DMissRate(mr->eval(L1Size()));
  SetL1IMissRate(mr->eval(L1Size()));
  SetL2MissRate(mr->eval(L2Size()));
  SetL3MissRate(mr->eval(L3SizeEff()));

  // DEBUG
  /*cout << "L1SIZE = " << L1Size() << ", L2SIZE = " << L2Size() << ", L3SIZEEFF = " << L3SizeEff() << endl;
  cout << "L1ACC = " << 1.0 << ", L1MR = " << L1DMissRate() << endl;
  cout << "L2ACC = " << L1IMissRate() << ", L2MR = " << L2MissRate() << endl;
  cout << "L3ACC = " << L1DMissRate()*L2MissRate() << ", L3MR = " << L3MissRate() << endl;
  cout << "MMACC = " << L1DMissRate()*L2MissRate()*L3MissRate() << ", MMHR = " << mr->eval(L3SizeEff()+L2Size()+L1Size()) << endl;*/
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
