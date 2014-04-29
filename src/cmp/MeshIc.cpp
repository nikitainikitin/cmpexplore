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

#include "MeshIc.hpp"
#include "Cluster.hpp"
#include "CmpConfig.hpp"
#include "MeshIcTile.hpp"
#include "RouterModel.hpp"
#include "Config.hpp"

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

MeshIc::MeshIc (Cluster * c, UInt subnCnt, double freq, double volt) :
  Interconnect (ITMESH, c, freq, volt, subnCnt), GGraph (),
  linkDelay_ (1), routerDelay_ (3) {}

MeshIc::~MeshIc () {}

//=======================================================================
/*
 * Factory method for vertex: creates Tile object.
 */

GVertex * MeshIc::CreateVertex ( int idx )
{
  return new MeshIcTile(idx, idx%ColNum(), idx/ColNum());
}

//=======================================================================
/*
 * Factory method for edge: creates Link object.
 */

GDEdge * MeshIc::CreateEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal )
{
  return new MeshIcLink(idx, static_cast<MeshIcTile*>(src),
                          static_cast<MeshIcTile*>(dst), horizontal, 0.0);
}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'srcIdx' to
 * another component with index 'dstIdx'.
 */

int MeshIc::DistanceCompToComp (UShort srcIdx, UShort dstIdx)
{
  return (abs(GetTile(srcIdx)->ColIdx()-GetTile(dstIdx)->ColIdx()) +
           abs(GetTile(srcIdx)->RowIdx()-GetTile(dstIdx)->RowIdx()));
}

//=======================================================================
/*
 * Returns hop-count distance from component with index 'idx' to
 * the Iface component.
 * Assume Iface component is in the top-right tile.
 */

int MeshIc::DistanceCompToIface (UShort idx)
{
  return ( abs(ColNum()-1-GetTile(idx)->ColIdx()) + GetTile(idx)->RowIdx() );
}

//=======================================================================
/*
 * Returns latency from component with index 'srcIdx' to
 * another component with latency 'dstIdx'.
 * Note: latency unit is [ns]
 */

double MeshIc::LatencyCompToComp (UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  return Latency(srcIdx, RDPRIMARY, dstIdx, RDPRIMARY, dynamic, subnIdx) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to the memory
 * controller 'mcIdx'.
 * Assume routers are placed in the top-right corner of the tile.
 * Note: latency unit is [ns]
 */

double MeshIc::LatencyCompToMemCtrl (UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(idx);

  UShort dstIdx = MAX_USHORT;
  RouteDir dstPort;
  
  double latency = 0.0;

  // define destination router and port
  if (mc->name == "North") {
    // go up to the controller
    dstIdx = tile->ColIdx();
    dstPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    dstIdx = (tile->RowIdx()+1)*ColNum()-1;
    dstPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    dstIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    dstPort = RDSOUTH;
//    latency += LinkDelay()+RouterDelay(); // as router is in the top-right corner of the tile
  }
  else if (mc->name == "West") {
    // go left to the controller
    dstIdx = idx-tile->ColIdx();
    dstPort = RDWEST;
//    latency += LinkDelay()+RouterDelay(); // as router is in the top-right corner of the tile
  }

  DASSERT(dstIdx != MAX_USHORT);

  return latency + Latency(idx, RDPRIMARY, dstIdx, dstPort, dynamic, subnIdx) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from memory controller with index 'mcIdx' to
 * component with index 'idx'.
 * Assume routers are placed in the top-right corner of the tile.
 * Note: latency unit is [ns]
 */

double MeshIc::LatencyMemCtrlToComp (UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(idx);

  UShort srcIdx = MAX_USHORT;
  RouteDir srcPort;

  double latency = 0.0;

  // define source router and port
  if (mc->name == "North") {
    // go up to the controller
    srcIdx = tile->ColIdx();
    srcPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    srcIdx = (tile->RowIdx()+1)*ColNum()-1;
    srcPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    srcIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    srcPort = RDSOUTH;
//    latency += LinkDelay()+RouterDelay(); // as router is in the top-right corner of the tile
  }
  else if (mc->name == "West") {
    // go left to the controller
    srcIdx = idx-tile->ColIdx();
    srcPort = RDWEST;
//    latency += LinkDelay()+RouterDelay(); // as router is in the top-right corner of the tile
  }

  DASSERT(srcIdx != MAX_USHORT);

  return latency + Latency(srcIdx, srcPort, idx, RDPRIMARY, dynamic, subnIdx) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from component with index 'idx' to
 * the interface component.
 * Assume that the interface component is the top-right tile.
 * Note: latency unit is [ns]
 */

double MeshIc::LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  UShort dstIdx = ColNum()-1;
  
  return Latency(idx, RDPRIMARY, dstIdx, RDIFACE, dynamic, subnIdx) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Returns latency from the interface component to
 * component with index 'idx'.
 * Assume that the interface component is the top-right tile.
 * Note: latency unit is [ns]
 */

double MeshIc::LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx)
{
  UShort srcIdx = ColNum()-1;
  
  return Latency(srcIdx, RDIFACE, idx, RDPRIMARY, dynamic, subnIdx) + (pSize-1.0)/Freq();
}

//=======================================================================
/*
 * Follow route between components and save traffic rates into the matrix.
 */

void MeshIc::MarkPathCompToComp (UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx)
{
  MarkPath (srcIdx, RDPRIMARY, dstIdx, RDPRIMARY, traffic, subnIdx);
}

//=======================================================================
/*
 * Follow route between a component and a memctrl
 * and save traffic rates into the matrix.
 */

void MeshIc::MarkPathCompToMemCtrl (UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx)
{
  //cout << "MarkPathCompToMemCtrl started, cIdx = " << cIdx << ", mcIdx = " << mcIdx << endl;

  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(cIdx);

  UShort dstIdx = MAX_USHORT;
  RouteDir dstPort;

  // define destination router and port
  if (mc->name == "North") {
    // go up to the controller
    dstIdx = tile->ColIdx();
    dstPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    dstIdx = (tile->RowIdx()+1)*ColNum()-1;
    dstPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    dstIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    dstPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    dstIdx = cIdx-tile->ColIdx();
    dstPort = RDWEST;
  }

  DASSERT(dstIdx != MAX_USHORT);

  MarkPath (cIdx, RDPRIMARY, dstIdx, dstPort, traffic, subnIdx);
  //cout << "MarkPathCompToMemCtrl finished" << endl;
}

//=======================================================================
/*
 * Follow route between a memctrl and a component
 * and save traffic rates into the matrix.
 */

void MeshIc::MarkPathMemCtrlToComp (UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx)
{
  //cout << "MarkPathMemCtrlToComp started, mcIdx = " << mcIdx << ", cIdx = " << cIdx << endl;

  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(cIdx);

  UShort srcIdx = MAX_USHORT;
  RouteDir srcPort;

  // define source router and port
  if (mc->name == "North") {
    // go up to the controller
    srcIdx = tile->ColIdx();
    srcPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    srcIdx = (tile->RowIdx()+1)*ColNum()-1;
    srcPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    srcIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    srcPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    srcIdx = cIdx-tile->ColIdx();
    srcPort = RDWEST;
  }

  DASSERT(srcIdx != MAX_USHORT);

  MarkPath (srcIdx, srcPort, cIdx, RDPRIMARY, traffic, subnIdx);
  //cout << "MarkPathMemCtrlToComp finished" << endl;
}

//=======================================================================
/*
 * Follow route from component 'idx' to the interface component
 * and save traffic rates into the matrix.
 */

void MeshIc::MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx)
{
  UShort dstIdx = ColNum()-1;
  MarkPath (idx, RDPRIMARY, dstIdx, RDIFACE, traffic, subnIdx);
}

//=======================================================================
/*
 * Follow route from the interface component to the component 'idx'
 * and save traffic rates into the matrix.
 */

void MeshIc::MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx)
{
  UShort srcIdx = ColNum()-1;
  MarkPath (srcIdx, RDIFACE, idx, RDPRIMARY, traffic, subnIdx);
}

//=======================================================================
/*
 * Follows the route from srcPort of srcRouter to dstPort of dstRouter
 * and returns the latency between two ports. Assume XY routing.
 * TODO: merge with MarkPath() to have only one path traversal method
 * parametrizable by functors (like traffic filling and latency calculation).
 * Note: latency unit is [ns]
 */

double MeshIc::Latency (UShort srcRouter, RouteDir srcPort,
                        UShort dstRouter, RouteDir dstPort, bool dynamic, UShort subnIdx)
{
  MeshIcTile * curTile = GetTile(srcRouter);
  MeshIcTile * dstTile = GetTile(dstRouter);

  RouteDir curInPort = srcPort;
  
  double latency = 0.0;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    /*RouteDir curOutPort = (curTile->ColIdx() > dstTile->ColIdx()) ?
                            RDWEST : RDEAST;*/
    latency += LinkDelayNs() + RouterDelayNs() +
                 (dynamic ? BufDelay(subnIdx, curTile->Idx(), curInPort) : 0.0);
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    /*RouteDir curOutPort = (curTile->RowIdx() > dstTile->RowIdx()) ?
                            RDNORTH : RDSOUTH;*/
    latency += LinkDelayNs() + RouterDelayNs() +
                 (dynamic ? BufDelay(subnIdx, curTile->Idx(), curInPort) : 0.0);
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  latency += RouterDelayNs() + (dynamic ? BufDelay(subnIdx, curTile->Idx(), curInPort) : 0.0);
  
  return latency;
}

//=======================================================================
/*
 * Follows the route from srcPort of srcRouter to dstPort of dstRouter
 * and fills in the traffic data into the matrix. Assume XY routing.
 */

void MeshIc::MarkPath (UShort srcRouter, RouteDir srcPort,
                       UShort dstRouter, RouteDir dstPort, double traffic, UShort subnIdx)
{
  MeshIcTile * curTile = GetTile(srcRouter);
  MeshIcTile * dstTile = GetTile(dstRouter);

  RouteDir curInPort = srcPort;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    RouteDir curOutPort = (curTile->ColIdx() > dstTile->ColIdx()) ?
                            RDWEST : RDEAST;
    Traffic(subnIdx, curTile->Idx(), curInPort, curOutPort) += traffic;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    RouteDir curOutPort = (curTile->RowIdx() > dstTile->RowIdx()) ?
                            RDNORTH : RDSOUTH;
    Traffic(subnIdx, curTile->Idx(), curInPort, curOutPort) += traffic;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  Traffic(subnIdx, curTile->Idx(), curInPort, dstPort) += traffic;
}

//=======================================================================
/*
 * Initialize traffic matrix: resize and set all values to zero.
 * Resize the buffer delay matrix accordingly.
 */

void MeshIc::InitModel()
{
  tm_.resize(SubnCnt()*TCnt()*PORT_PAIR_NUM);
  tm_.assign(SubnCnt()*TCnt()*PORT_PAIR_NUM, 0.0);
  bufDelays_.resize(SubnCnt()*TCnt()*PORT_NUM);
}

//=======================================================================
/*
 * Run analytical modeling of the contention delays in input buffers.
 */

int MeshIc::EstimateBufferDelays(bool fixNegDelays)
{
  // Create an array of router models, one per subnetwork:
  // - if CC is not modeled, i.e. the number of subnetworks is 1,
  //   use service time as (Req + Reply)/2
  // - if CC is modeled, i.e. the number of subnetworks is 3,
  //   use Req service time for Req, Ack and Reply time for Data
  vector<RouterModel*> rms;
  if (!config.SimulateCC()) {
    rms.push_back(new RouterModel(PORT_NUM, PORT_NUM,
                                  ((cmpConfig.MemReplySize()+1.0)/2.0+RouterDelay()-1.0)/Freq()));
  }
  else {
    rms.push_back(new RouterModel(PORT_NUM, PORT_NUM, RouterDelay()/Freq())); // REQ
    rms.push_back(new RouterModel(PORT_NUM, PORT_NUM, RouterDelay()/Freq())); // ACK
    rms.push_back(new RouterModel(PORT_NUM, PORT_NUM,
                                  (RouterDelay()+cmpConfig.MemReplySize()-1.0)/Freq())); // DATA
  }

  double max_adj = 1.0;
  double cur_adj;

  bool error = false;

  for (int s = 0; s < SubnCnt(); ++s) {
    for (int r = 0; r < TCnt(); ++r) {
      //cout << " ========== Traffic matrix for router " << r << " ===========" << endl;
      rms[s]->CalcBufferDelays(
            &Traffic(s,r,RDNORTH,RDNORTH), &BufDelay(s,r,RDNORTH), &cur_adj, fixNegDelays );
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
/*
 *
 */

bool MeshIc::PortOnPathCompToComp(UShort srcIdx, UShort dstIdx, UShort rIdx, UShort pIdx)
{
  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
    return true;

  return false;
}

//=======================================================================
/*
 *
 */

bool MeshIc::PortOnPathCompToMemCtrl(UShort srcIdx, UShort mcIdx, UShort rIdx, UShort pIdx)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(srcIdx);

  UShort dstIdx = MAX_USHORT;
  RouteDir dstPort;

  // define destination router and port
  if (mc->name == "North") {
    // go up to the controller
    dstIdx = tile->ColIdx();
    dstPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    dstIdx = (tile->RowIdx()+1)*ColNum()-1;
    dstPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    dstIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    dstPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    dstIdx = srcIdx-tile->ColIdx();
    dstPort = RDWEST;
  }

  DASSERT(dstIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
    return true;

  return false;
}

//=======================================================================
/*
 *
 */

bool MeshIc::PortOnPathMemCtrlToComp(UShort mcIdx, UShort dstIdx, UShort rIdx, UShort pIdx)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(dstIdx);

  UShort srcIdx = MAX_USHORT;
  RouteDir srcPort;

  // define source router and port
  if (mc->name == "North") {
    // go up to the controller
    srcIdx = tile->ColIdx();
    srcPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    srcIdx = (tile->RowIdx()+1)*ColNum()-1;
    srcPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    srcIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    srcPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    srcIdx = dstIdx-tile->ColIdx();
    srcPort = RDWEST;
  }

  DASSERT(srcIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = srcPort;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
      return true;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == rIdx && curInPort == pIdx)
    return true;

  return false;
}

//=======================================================================
/*
 *
 */

void MeshIc::DumpLatencyEqCompToComp(UShort srcIdx, UShort dstIdx,
                                     double prob, std::ostringstream& os)
{
  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  os << "+" << prob << "*" << RouterDelay()
     << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
}

//=======================================================================
/*
 *
 */

void MeshIc::DumpLatencyEqCompToMemCtrl(UShort srcIdx, UShort mcIdx,
                                        double prob, std::ostringstream& os)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(srcIdx);

  UShort dstIdx = MAX_USHORT;
  RouteDir dstPort;

  // define destination router and port
  if (mc->name == "North") {
    // go up to the controller
    dstIdx = tile->ColIdx();
    dstPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    dstIdx = (tile->RowIdx()+1)*ColNum()-1;
    dstPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    dstIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    dstPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    dstIdx = srcIdx-tile->ColIdx();
    dstPort = RDWEST;
  }

  DASSERT(dstIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  os << "+" << prob << "*" << RouterDelay()
     << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
}

//=======================================================================
/*
 *
 */

void MeshIc::DumpLatencyEqMemCtrlToComp(UShort mcIdx, UShort dstIdx,
                                        double prob, std::ostringstream& os)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(dstIdx);

  UShort srcIdx = MAX_USHORT;
  RouteDir srcPort;

  // define source router and port
  if (mc->name == "North") {
    // go up to the controller
    srcIdx = tile->ColIdx();
    srcPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    srcIdx = (tile->RowIdx()+1)*ColNum()-1;
    srcPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    srcIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    srcPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    srcIdx = dstIdx-tile->ColIdx();
    srcPort = RDWEST;
  }

  DASSERT(srcIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = srcPort;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    os << "+" << prob << "*" << (LinkDelay() + RouterDelay())
       << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  os << "+" << prob << "*" << RouterDelay()
     << "+" << prob << "*wm_" << curTile->Idx() << "_" << curInPort;
}

//=======================================================================
/*
 *
 */

bool MeshIc::PathCompToCompFollowsPorts(UShort srcIdx, UShort dstIdx,
                                        UShort r, UShort pi, UShort po)
{
  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    RouteDir curOutPort = (curTile->ColIdx() > dstTile->ColIdx()) ?
                            RDWEST : RDEAST;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    RouteDir curOutPort = (curTile->RowIdx() > dstTile->RowIdx()) ?
                            RDNORTH : RDSOUTH;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == r && curInPort == pi && RDPRIMARY == po)
    return true;

  return false;
}

//=======================================================================
/*
 *
 */

bool MeshIc::PathCompToMemCtrlFollowsPorts(UShort srcIdx, UShort mcIdx,
                                           UShort r, UShort pi, UShort po)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(srcIdx);

  UShort dstIdx = MAX_USHORT;
  RouteDir dstPort;

  // define destination router and port
  if (mc->name == "North") {
    // go up to the controller
    dstIdx = tile->ColIdx();
    dstPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    dstIdx = (tile->RowIdx()+1)*ColNum()-1;
    dstPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    dstIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    dstPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    dstIdx = srcIdx-tile->ColIdx();
    dstPort = RDWEST;
  }

  DASSERT(dstIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = RDPRIMARY;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    RouteDir curOutPort = (curTile->ColIdx() > dstTile->ColIdx()) ?
                            RDWEST : RDEAST;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    RouteDir curOutPort = (curTile->RowIdx() > dstTile->RowIdx()) ?
                            RDNORTH : RDSOUTH;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == r && curInPort == pi && dstPort == po)
    return true;

  return false;
}

//=======================================================================
/*
 *
 */

bool MeshIc::PathMemCtrlToCompFollowsPorts(UShort mcIdx, UShort dstIdx,
                                           UShort r, UShort pi, UShort po)
{
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);

  MeshIcTile * tile = GetTile(dstIdx);

  UShort srcIdx = MAX_USHORT;
  RouteDir srcPort;

  // define source router and port
  if (mc->name == "North") {
    // go up to the controller
    srcIdx = tile->ColIdx();
    srcPort = RDNORTH;
  }
  else if (mc->name == "East") {
    // go right to the controller
    srcIdx = (tile->RowIdx()+1)*ColNum()-1;
    srcPort = RDEAST;
  }
  else if (mc->name == "South") {
    // go down to the controller
    srcIdx = (RowNum()-1)*ColNum()+tile->ColIdx();
    srcPort = RDSOUTH;
  }
  else if (mc->name == "West") {
    // go left to the controller
    srcIdx = dstIdx-tile->ColIdx();
    srcPort = RDWEST;
  }

  DASSERT(srcIdx != MAX_USHORT);

  MeshIcTile * curTile = GetTile(srcIdx);
  MeshIcTile * dstTile = GetTile(dstIdx);

  RouteDir curInPort = srcPort;

  // go X-direction
  while (curTile->ColIdx() != dstTile->ColIdx()) {
    RouteDir curOutPort = (curTile->ColIdx() > dstTile->ColIdx()) ?
                            RDWEST : RDEAST;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->ColIdx() > dstTile->ColIdx()) {
      curTile = curTile->WestOut()->Dst();
      curInPort = RDEAST;
    }
    else {
      curTile = curTile->EastOut()->Dst();
      curInPort = RDWEST;
    }
  }

  // go Y-direction
  while (curTile->RowIdx() != dstTile->RowIdx()) {
    RouteDir curOutPort = (curTile->RowIdx() > dstTile->RowIdx()) ?
                            RDNORTH : RDSOUTH;
    if (curTile->Component()->ClIdx() == r && curInPort == pi && curOutPort == po)
      return true;

    // move to next tile
    if (curTile->RowIdx() > dstTile->RowIdx()) {
      curTile = curTile->NorthOut()->Dst();
      curInPort = RDSOUTH;
    }
    else {
      curTile = curTile->SouthOut()->Dst();
      curInPort = RDNORTH;
    }
  }

  // mark last tile
  if (curTile->Component()->ClIdx() == r && curInPort == pi && RDPRIMARY == po)
    return true;

  return false;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
