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

#include "XBarIc.hpp"
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

XBarIc::XBarIc (Cluster * c, UInt subnCnt, double freq, double volt) :
  Interconnect (ITXBAR, c, freq, volt, subnCnt) {}

XBarIc::~XBarIc () {}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 */

int XBarIc::DistanceCompToComp (UShort srcIdx, UShort dstIdx)
{
  return 1;
}

//=======================================================================
/*
 * Returns hop-count distance between component with index 'idx' and
 * the NI component.
 */

int XBarIc::DistanceCompToIface (UShort idx)
{
  return 1;
}

//=======================================================================
/*
 * Returns latency from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 * Note: latency unit is [ns]
 */

double XBarIc::LatencyCompToComp (UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return DelayNs() + (dynamic ? BufDelay(subnIdx, srcIdx) : 0.0) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to the memory
 * controller 'mcIdx'.
 */

double XBarIc::LatencyCompToMemCtrl (UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  cout << "Error: memory controllers are not fully supported in connection with xbars" << endl;
  cout << "Error: buffers for MCs should be added to XBarIc for dynamic latency" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Returns latency from memory controller with index 'mcIdx' to
 * component with index 'idx'.
 */

double XBarIc::LatencyMemCtrlToComp (UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  cout << "Error: memory controllers are not fully supported in connection with xbars" << endl;
  cout << "Error: buffers for MCs should be added to XBarIc for dynamic latency" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to
 * the interface component.
 * Assume that the interface component is the last one in the list.
 * Note: latency unit is [ns]
 */

double XBarIc::LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return DelayNs() + (dynamic ? BufDelay(subnIdx, idx) : 0.0) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from the interface component to
 * component with index 'idx'.
 * Assume that the interface component is the last one in the list.
 * Note: latency unit is [ns]
 */

double XBarIc::LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return DelayNs() + (dynamic ? BufDelay(subnIdx, TotalCompCnt()-1) : 0.0) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Follow route between components and save traffic rates into the matrix.
 */

void XBarIc::MarkPathCompToComp (UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, srcIdx, dstIdx) += traffic;
}

//=======================================================================
/*
 * Follow route between a component and a memctrl
 * and save traffic rates into the matrix.
 */

void XBarIc::MarkPathCompToMemCtrl (UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with xbars" << endl;
  cout << "Warning: buffers for MCs should be added to XBarIc for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route between a memctrl and a component
 * and save traffic rates into the matrix.
 */

void XBarIc::MarkPathMemCtrlToComp (UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx)
{
  cout << "Warning: memory controllers are not fully supported in connection with xbars" << endl;
  cout << "Warning: buffers for MCs should be added to XBarIc for dynamic latency" << endl;
  cout << "Warning: MCs have to be associated with some component index" << endl;
  DASSERT(0);
}

//=======================================================================
/*
 * Follow route from component 'idx' to the interface component
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void XBarIc::MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, idx, TotalCompCnt()-1) += traffic;
}

//=======================================================================
/*
 * Follow route from the interface component to the component 'idx'
 * and save traffic rates into the matrix.
 * Assume that the interface component is the last one in the list.
 */

void XBarIc::MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx)
{
  Traffic(subnIdx, TotalCompCnt()-1, idx) += traffic;
}

//=======================================================================
/*
 * Initialize traffic matrix: resize and set all values to zero.
 * Resize the buffer delay matrix accordingly.
 */

void XBarIc::InitModel()
{
  tm_.resize(SubnCnt()*TotalCompCnt()*TotalCompCnt());
  tm_.assign(SubnCnt()*TotalCompCnt()*TotalCompCnt(), 0.0);
  bufDelays_.resize(SubnCnt()*TotalCompCnt());
}

//=======================================================================
/*
 * Run analytical modeling of the contention delays in input buffers.
 */

int XBarIc::EstimateBufferDelays(bool fixNegDelays)
{
  // Create an array of router models, one per subnetwork:
  // - if CC is not modeled, i.e. the number of subnetworks is 1,
  //   use service time as (Req + Reply)/2
  // - if CC is modeled, i.e. the number of subnetworks is 3,
  //   use Req service time for Req, Ack and Reply time for Data
  vector<RouterModel*> rms;
  if (!config.SimulateCC()) {
    rms.push_back(new RouterModel(TotalCompCnt(), TotalCompCnt(),
                                  ((cmpConfig.MemReplySize()+1.0)/2.0+Delay()-1.0)/Freq()));
  }
  else {
    rms.push_back(new RouterModel(TotalCompCnt(), TotalCompCnt(), Delay()/Freq())); // REQ
    rms.push_back(new RouterModel(TotalCompCnt(), TotalCompCnt(), Delay()/Freq())); // ACK
    rms.push_back(new RouterModel(TotalCompCnt(), TotalCompCnt(),
                                  (Delay()+cmpConfig.MemReplySize()-1.0)/Freq())); // DATA
  }

  double max_adj = 1.0;
  double cur_adj;

  bool error = false;

  for (int s = 0; s < SubnCnt(); ++s) {
    rms[s]->CalcBufferDelays( &Traffic(s,0,0), &BufDelay(s,0), &cur_adj, fixNegDelays );
    if (cur_adj > max_adj) {
      max_adj = cur_adj;
      error = true;
    }
  }

  for (vector<RouterModel*>::const_iterator it = rms.begin(); it != rms.end(); ++it)
    delete *it;

  return error ? 1 : 0;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
