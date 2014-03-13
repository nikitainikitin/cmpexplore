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

#include "BusIc.hpp"
#include "Cluster.hpp"
#include "CmpConfig.hpp"
#include "Config.hpp"
#include "../model/BusModel.hpp"

using std::cout;
using std::endl;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  extern Config config;
  
  using namespace model;

  namespace cmp {

//=======================================================================
/*
 * Constructors and destructor
 */

BusIc::BusIc (Cluster * c, UInt subnCnt) : Interconnect (ITBUS, c, subnCnt),
  accessTime_ (1) {}

BusIc::~BusIc () {}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 */

int BusIc::DistanceCompToComp (UShort srcIdx, UShort dstIdx)
{
  //return AccessTime();
  return 1;
}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'idx' to
 * the Iface component.
 * Assume Iface component is in the top-right tile.
 */

int BusIc::DistanceCompToIface (UShort idx)
{
  //return AccessTime();
  return 1;
}

//=======================================================================
/*
 * Returns latency from component with index 'srcIdx' to
 * another component with latency 'dstIdx'.
 */

double BusIc::LatencyCompToComp (UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return AccessTime()*pSize + (dynamic ? BufDelay(subnIdx, srcIdx) : 0.0);
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to the memory
 * controller 'mcIdx'.
 * Assume routers are placed in the top-right corner of the tile.
 */

double BusIc::LatencyCompToMemCtrl (UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return AccessTime()*pSize + (dynamic ? BufDelay(subnIdx, idx) : 0.0);
}

//=======================================================================
/*
 * Returns latency from memory controller with index 'mcIdx' to
 * component with index 'idx'.
 * Assume routers are placed in the top-right corner of the tile.
 */

double BusIc::LatencyMemCtrlToComp (UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with buses" << endl;
  cout << "Warning: buffers for MCs should be added to BusIC for dynamic latency" << endl;
  return AccessTime()*pSize; // + dynamic latency in MC buffer
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to
 * the interface component.
 * Assume that the interface component is the last one in the list.
 */

double BusIc::LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return AccessTime()*pSize + (dynamic ? BufDelay(subnIdx, idx) : 0.0);
}

//=======================================================================
/*
 * Returns latency from the interface component to
 * component with index 'idx'.
 * Assume that the interface component is the last one in the list.
 */

double BusIc::LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return AccessTime()*pSize + (dynamic ? BufDelay(subnIdx, TotalCompCnt()-1) : 0.0);
}

//=======================================================================
/*
 * Follow route between components and save traffic rates into the matrix.
 */

void BusIc::MarkPathCompToComp (UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, srcIdx, dstIdx) += traffic;
}

//=======================================================================
/*
 * Follow route between a component and a memctrl
 * and save traffic rates into the matrix.
 */

void BusIc::MarkPathCompToMemCtrl (UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with buses" << endl;
  cout << "Warning: buffers for MCs should be added to BusIC for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route between a memctrl and a component
 * and save traffic rates into the matrix.
 */

void BusIc::MarkPathMemCtrlToComp (UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with buses" << endl;
  cout << "Warning: buffers for MCs should be added to BusIC for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route from component 'idx' to the interface component
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void BusIc::MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, idx, TotalCompCnt()-1) += traffic;
}

//=======================================================================
/*
 * Follow route from the interface component to the component 'idx'
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void BusIc::MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, TotalCompCnt()-1, idx) += traffic;
}

//=======================================================================
/*
 * Initialize traffic matrix: resize and set all values to zero.
 * Resize the buffer delay matrix accordingly.
 */

void BusIc::InitModel()
{
  tm_.resize(SubnCnt()*TotalCompCnt()*TotalCompCnt());
  tm_.assign(SubnCnt()*TotalCompCnt()*TotalCompCnt(), 0.0);
  bufDelays_.resize(SubnCnt()*TotalCompCnt());
}

//=======================================================================
/*
 * Run analytical modeling of the contention delays in input buffers.
 */

int BusIc::EstimateBufferDelays(bool fixNegDelays)
{
  // Create an array of bus models, one per subnetwork:
  // - if CC is not modeled, i.e. the number of subnetworks is 1,
  //   use service time as (Req + Reply)/2
  // - if CC is modeled, i.e. the number of subnetworks is 3,
  //   use Req service time for Req, Ack and Reply time for Data
  vector<BusModel*> bms;
  if (!config.SimulateCC()) {
    bms.push_back(new BusModel(TotalCompCnt(), (cmpConfig.MemReplySize()+1.0)*AccessTime()/2.0));
  }
  else {
    bms.push_back(new BusModel(TotalCompCnt(), AccessTime())); // REQ
    bms.push_back(new BusModel(TotalCompCnt(), AccessTime())); // ACK
    bms.push_back(new BusModel(TotalCompCnt(), cmpConfig.MemReplySize()*AccessTime())); // DATA
  }

  bool error = false;
  for (int s = 0; s < SubnCnt(); ++s) {
    if (bms[s]->CalcBufferDelays( &Traffic(s,0,0), &BufDelay(s,0), fixNegDelays )) {
      error = true; break;
    }
  }

  for (vector<BusModel*>::const_iterator it = bms.begin(); it != bms.end(); ++it)
    delete *it;

  return error ? 1 : 0;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
