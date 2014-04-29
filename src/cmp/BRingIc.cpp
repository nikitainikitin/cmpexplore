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

#include "BRingIc.hpp"
#include "Cluster.hpp"
#include "CmpConfig.hpp"
#include "Config.hpp"
#include "model/RouterModel.hpp"

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

BRingIc::BRingIc (Cluster * c, UInt subnCnt, double freq, double volt) :
  Interconnect (ITBRING, c, freq, volt, subnCnt),
  routerDelay_ (1), linkDelay_ (1) {}

BRingIc::~BRingIc () {}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 */

int BRingIc::DistanceCompToComp (UShort srcIdx, UShort dstIdx)
{
  /*int d = dstIdx > srcIdx ? std::min(dstIdx-srcIdx, srcIdx+TotalCompCnt()-dstIdx) :
                            std::min(srcIdx-dstIdx, dstIdx+TotalCompCnt()-srcIdx);
  DASSERT(d <= TotalCompCnt()/2);
  return d;*/
  return 1;
}

//=======================================================================
/*
 * Returns hop-count distance between component with index 'idx' and
 * the NI component.
 */

int BRingIc::DistanceCompToIface (UShort idx)
{
  /*return std::min(TotalCompCnt()-1-idx, idx+1);*/
  return 1;
}

//=======================================================================
/*
 * Returns latency from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 * NOTE: latency unit is [ns]
 */

double BRingIc::LatencyCompToComp (UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  UShort curIdx = srcIdx;
  UShort curInPort = 0; // packet is injected into the ring via component port (0)

  double latency = 0.0;

  // decide shortest routing direction: clockwise vs ccw
  bool pathCW = dstIdx > srcIdx ? signed(dstIdx)-signed(srcIdx) <= TotalCompCnt()/2.0 :
                                  signed(dstIdx)-signed(srcIdx) <= -TotalCompCnt()/2.0;

  while (curIdx != dstIdx) {
    latency += LinkDelayNs() + RouterDelayNs() +
                 (dynamic ? BufDelay(subnIdx, curIdx, curInPort) : 0.0);
    // move to next router
    if (pathCW) { // clockwise direction
      if (curIdx < TotalCompCnt()-1) { // not reached NI
        ++curIdx;
        // if not reached NI, input port is only updated upon injection
        if (!curInPort) curInPort = 1;
      }
      else { // reached NI
        curIdx = 0;
        // if reached NI:
        // - assign to VC0 (port 1) if packet is injected
        // - switch to VC1 (port 2) if packet passes through NI
        curInPort = curInPort ? 2 : 1;
      }
    }
    else { // counterclockwise direction
      if (curIdx < TotalCompCnt()-1) { // not reached NI
        curIdx = curIdx ? curIdx-1 : TotalCompCnt()-1;
        // if not reached NI, input port is only updated upon injection
        if (!curInPort) curInPort = 3;
      }
      else { // reached NI
        --curIdx;
        // if reached NI:
        // - assign to VC0 (port 3) if packet is injected
        // - switch to VC1 (port 4) if packet passes through NI
        curInPort = curInPort ? 4 : 3;
      }
    }
  }

  // at the destination router
  latency += RouterDelayNs() + (dynamic ? BufDelay(subnIdx, curIdx, curInPort) : 0.0) +
             (pSize-1.0)/Freq();

  return latency;
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to the memory
 * controller 'mcIdx'.
 */

double BRingIc::LatencyCompToMemCtrl (UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  cout << "Error: memory controllers are not fully supported in connection with rings" << endl;
  cout << "Error: buffers for MCs should be added to BRingIc for dynamic latency" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Returns latency from memory controller with index 'mcIdx' to
 * component with index 'idx'.
 */

double BRingIc::LatencyMemCtrlToComp (UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  cout << "Error: memory controllers are not fully supported in connection with rings" << endl;
  cout << "Error: buffers for MCs should be added to BRingIc for dynamic latency" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to
 * the interface component.
 * Assume that the interface component is the last one in the list.
 */

double BRingIc::LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return LatencyCompToComp(idx, TotalCompCnt()-1, pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Returns latency from the interface component to
 * component with index 'idx'.
 * Assume that the interface component is the last one in the list.
 */

double BRingIc::LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return LatencyCompToComp(TotalCompCnt()-1, idx, pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Follow route between components and save traffic rates into the matrix.
 */

void BRingIc::MarkPathCompToComp (UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx)
{
  UShort curIdx = srcIdx;
  UShort curInPort = 0; // packet is injected into the ring via component port (0)

  // decide shortest routing direction: clockwise vs ccw
  bool pathCW = dstIdx > srcIdx ? signed(dstIdx)-signed(srcIdx) <= TotalCompCnt()/2.0 :
                                  signed(dstIdx)-signed(srcIdx) <= -TotalCompCnt()/2.0;

  while (curIdx != dstIdx) {
    Traffic(subnIdx, curIdx, curInPort, pathCW ? 1 : 2) += traffic;
    // move to next router
    if (pathCW) { // clockwise direction
      if (curIdx < TotalCompCnt()-1) { // not reached NI
        ++curIdx;
        // if not reached NI, input port is only updated upon injection
        if (!curInPort) curInPort = 1;
      }
      else { // reached NI
        curIdx = 0;
        // if reached NI:
        // - assign to VC0 (port 1) if packet is injected
        // - switch to VC1 (port 2) if packet passes through NI
        curInPort = curInPort ? 2 : 1;
      }
    }
    else { // counterclockwise direction
      if (curIdx < TotalCompCnt()-1) { // not reached NI
        curIdx = curIdx ? curIdx-1 : TotalCompCnt()-1;
        // if not reached NI, input port is only updated upon injection
        if (!curInPort) curInPort = 3;
      }
      else { // reached NI
        --curIdx;
        // if reached NI:
        // - assign to VC0 (port 3) if packet is injected
        // - switch to VC1 (port 4) if packet passes through NI
        curInPort = curInPort ? 4 : 3;
      }
    }
  }

  // at the destination router
  Traffic(subnIdx, curIdx, curInPort, 0) += traffic;
}

//=======================================================================
/*
 * Follow route between a component and a memctrl
 * and save traffic rates into the matrix.
 */

void BRingIc::MarkPathCompToMemCtrl (UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with rings" << endl;
  cout << "Warning: buffers for MCs should be added to BRingIc for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route between a memctrl and a component
 * and save traffic rates into the matrix.
 */

void BRingIc::MarkPathMemCtrlToComp (UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with rings" << endl;
  cout << "Warning: buffers for MCs should be added to BRingIc for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route from component 'idx' to the interface component
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void BRingIc::MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx)
{
  MarkPathCompToComp(idx, TotalCompCnt()-1, traffic, subnIdx);
}

//=======================================================================
/*
 * Follow route from the interface component to the component 'idx'
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void BRingIc::MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx)
{
  MarkPathCompToComp(TotalCompCnt()-1, idx, traffic, subnIdx);
}

//=======================================================================
/*
 * Initialize traffic matrix: resize and set all values to zero.
 * Resize the buffer delay matrix accordingly.
 */

void BRingIc::InitModel()
{
  tm_.resize(SubnCnt()*TotalCompCnt()*PORT_PAIR_NUM);
  tm_.assign(SubnCnt()*TotalCompCnt()*PORT_PAIR_NUM, 0.0);
  bufDelays_.resize(SubnCnt()*TotalCompCnt()*IPORT_NUM);
}

//=======================================================================
/*
 * Run analytical modeling of the contention delays in input buffers.
 */

int BRingIc::EstimateBufferDelays(bool fixNegDelays)
{
  // Create an array of router models, one per subnetwork:
  // - if CC is not modeled, i.e. the number of subnetworks is 1,
  //   use service time as (Req + Reply)/2
  // - if CC is modeled, i.e. the number of subnetworks is 3,
  //   use Req service time for Req, Ack and Reply time for Data
  vector<RouterModel*> rms;
  if (!config.SimulateCC()) {
    rms.push_back(new RouterModel(IPORT_NUM, OPORT_NUM,
                                  ((cmpConfig.MemReplySize()+1.0)/2.0+RouterDelay()-1.0)/Freq()));
  }
  else {
    rms.push_back(new RouterModel(IPORT_NUM, OPORT_NUM, RouterDelay()/Freq())); // REQ
    rms.push_back(new RouterModel(IPORT_NUM, OPORT_NUM, RouterDelay()/Freq())); // ACK
    rms.push_back(new RouterModel(IPORT_NUM, OPORT_NUM,
                                  (RouterDelay()+cmpConfig.MemReplySize()-1.0)/Freq())); // DATA
  }

  double max_adj = 1.0;
  double cur_adj;

  bool error = false;

  for (int s = 0; s < SubnCnt(); ++s) {
    for (int r = 0; r < TotalCompCnt(); ++r) {
      rms[s]->CalcBufferDelays( &Traffic(s,r,0,0), &BufDelay(s,r,0), &cur_adj, fixNegDelays );
      if (cur_adj > max_adj) {
        max_adj = cur_adj;
        error = true;
      }
    }
  }

  for (vector<RouterModel*>::const_iterator it = rms.begin(); it != rms.end(); ++it)
    delete *it;

  return error ? 1 : 0;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
