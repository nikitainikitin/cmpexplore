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
#include <string>
#include <vector>
#include <fstream>
#include <numeric>
#include <iomanip>

#include "PowerModel.hpp"
#include "Component.hpp"
#include "CmpConfig.hpp"
#include "Cluster.hpp"
#include "Processor.hpp"
#include "Memory.hpp"
#include "Interconnect.hpp"
#include "MeshIc.hpp"
#include "BusIc.hpp"
#include "URingIc.hpp"
#include "BRingIc.hpp"
#include "XBarIc.hpp"
#include "RouterDefs.hpp"
#include "Config.hpp"

using namespace std;

namespace cmpex {
  
  extern cmp::CmpConfig cmpConfig;
  extern Config config;

  using namespace cmp;

  namespace power {

//=======================================================================
/*
 * Constructors and destructor
 */

PowerModel::PowerModel() {}

PowerModel::~PowerModel() {}

vector<double> PowerModel::corePower_;
vector<double> PowerModel::L1Power_;
vector<double> PowerModel::L2Power_;
vector<double> PowerModel::L3Power_;
vector<double> PowerModel::MCPower_;
vector<double> PowerModel::MeshRouterPower_;
vector<double> PowerModel::MeshLinkPower_;

//=======================================================================
/*
 * Calculates power of cmp configuration.
 */

double PowerModel::GetTotalPower(Component * cmp)
{
  corePower_.assign(cmpConfig.ProcCnt(), 0.0);
  L1Power_.assign(cmpConfig.ProcCnt(), 0.0);
  L2Power_.assign(cmpConfig.ProcCnt(), 0.0);
  L3Power_.assign(cmpConfig.MemCnt(), 0.0);
  MCPower_.assign(cmpConfig.MemCtrlCnt(), 0.0);
  MeshRouterPower_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkPower_.assign(cmpConfig.ProcCnt()*4, 0.0);

  double power = 0.0;

  // Processor leakage and dynamic power
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    // core
    corePower_[p] = proc->Active() ?
                      proc->Epi() * VScalDynPowerProc(proc->Volt()) * proc->Thr() +
                      proc->Pleak() * VScalLeakPowerProc(proc->Volt()) : 0.0;

    // l1
    L1Power_[p] = proc->Active() ?
                    proc->L1Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L1AccessProbability() +
                    proc->L1Pleak() * VScalLeakPowerProc(proc->Volt()) : 0.0;

    // l2
    L2Power_[p] = proc->Active() ?
                    proc->L2Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L2AccessProbability() +
                    proc->L2Pleak() * VScalLeakPowerProc(proc->Volt()) : 0.0;
  }

  double core_power = 0.0;
  core_power += accumulate(corePower_.begin(), corePower_.end(), 0.0);
  core_power += accumulate(L1Power_.begin(), L1Power_.end(), 0.0);
  core_power += accumulate(L2Power_.begin(), L2Power_.end(), 0.0);
  power += core_power;

  //cout << "Core power = " << core_power << "W; ";

  // L3 leakage and dynamic
  double l3PowerTot = 0.0;
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);

    L3Power_[m] = mem->Eacc() * VScalDynPowerUncore(cmpConfig.UVolt()) * mem->Lambda() +
                  mem->Pleak() * VScalLeakPowerUncore(cmpConfig.UVolt());
  }

  double l3Power = 0.0;
  l3Power += accumulate(L3Power_.begin(), L3Power_.end(), 0.0);
  power += l3Power;

  //cout << "L3 power = " << l3Power << "W; ";

  // MC leakage and dynamic
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    CmpConfig::MemCtrl * memCtrl = cmpConfig.GetMemCtrl(mc);

    MCPower_[mc] = memCtrl->eacc * VScalDynPowerMc(cmpConfig.UVolt()) * memCtrl->lambda +
                   memCtrl->pleak * VScalLeakPowerMc(cmpConfig.UVolt());
  }

  double mcPower = 0.0;
  mcPower += accumulate(MCPower_.begin(), MCPower_.end(), 0.0);
  power += mcPower;

  //cout << "MC power = " << mcPower << "W; ";

  double icPower = GetIcPower(cmp);
  power += icPower;

  //cout << "IC power = " << icPower << "W; ";
  //cout << "Total power = " << power << "W" << endl;

  if (config.DumpPTsimPower()) DumpPTsimPower(cmp);

  return power;
}

//=======================================================================
/*
 * Calculates power of cmp interconnect.
 * Assumption: two-level or flat IC.
 */

double PowerModel::GetIcPower(Component * cmp)
{
  double power = 0.0;

  // top level
  Cluster * clCmp = static_cast<Cluster*>(cmp);
  power += GetFlatIcPower(clCmp->Ic());

  // bottom level
  if (!cmpConfig.FlatMeshIc()) {
    for (int c = 0; c < clCmp->CompCnt(); ++c) {
      Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));
      power += GetFlatIcPower(comp->Ic());
    }
  }

  return power;
}

//=======================================================================
/*
 * Calculates power of the flat interconnect.
 */

double PowerModel::GetFlatIcPower(Interconnect * ic)
{
  switch(ic->Type()) {
  case ITMESH: return MeshPower(static_cast<MeshIc*>(ic));
  case ITBUS: return BusPower(static_cast<BusIc*>(ic));
  case ITURING: return URingPower(static_cast<URingIc*>(ic));
  case ITBRING: return BRingPower(static_cast<BRingIc*>(ic));
  case ITXBAR: return XBarPower(static_cast<XBarIc*>(ic));
  }
}

//=======================================================================
/*
 * Calculates power of the mesh interconnect.
 * Assume all routers are 5x5, because of the connections to MCs.
 */

double PowerModel::MeshPower(MeshIc * ic)
{
  double power = 0.0;

  // calculate link length, assume square clusters
  double linkLen = sqrt(config.MaxArea()/(ic->ColNum()*ic->RowNum()));
  //cout << "Link length = " << linkLen << " mm" << endl;

  // routers
  for (int s = 0; s < ic->SubnCnt(); ++s) { // for every subnetwork
    // CCP support - number of flits depends on the subnetwork type
    double flitCnt = (1.0+cmpConfig.MemReplySize())/2;
    if (config.SimulateCC()) { // CCP is on
      flitCnt = (s == int(MSGDATA)) ? cmpConfig.MemReplySize() : 1;
    }

    for (int r = 0; r < ic->TCnt(); ++r) {
      // calculate total traffic on all ports
      double traffic = 0.0;
      for (int i = 0; i < MeshIc::PORT_NUM; ++i) {
        double iTraffic = 0.0;
        for (int o = 0; o < MeshIc::PORT_NUM; ++o) {
          iTraffic += ic->Traffic(s, r, RouteDir(i), RouteDir(o))*flitCnt;
        }
        traffic += iTraffic;
        // add power of the link connected to this input port
        if (RouteDir(i) != RDPRIMARY) {
          //cout << "r = " << r << ", traffic to input port " << i << " = " << iTraffic << endl;
          double link_power = ( LinkEpf(config.Tech(), config.LinkWidth())*iTraffic*VScalDynPowerUncore(cmpConfig.UVolt())
                     + LinkPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt()) ) * linkLen;
          power += link_power;
          //cout << "r = " << r << ", power of link at input port " << i << " = " << link_power << endl;
          MeshLinkPower_[r*4+i] += link_power;
        }
      }

      // Assume all routers are 5x5, because of the connections to MCs.
      int portNum = 5;
      /*bool atEdgeColumn = (r%ic->ColNum() == 0 || r%ic->ColNum() == ic->ColNum()-1);
      bool atEdgeRow = (r/ic->ColNum() == 0 || r/ic->ColNum() == ic->RowNum()-1);
      if (atEdgeColumn && atEdgeRow) { // 3x3 router
        portNum = 3;
      }
      else if (atEdgeColumn || atEdgeRow) { // 4x4 router
        portNum = 4;
      }
      else {
        portNum = 5;
      }*/

      double router_power = RouterEpf(config.Tech(), portNum, config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
               + RouterPleak(config.Tech(), portNum, config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
      power += router_power;
      MeshRouterPower_[r] += router_power;
    }
  }

  return power;
}

//=======================================================================
/*
 * Calculates power of the bus interconnect.
 * Bus is modeled as a link with size of the cluster and a set of buffers.
 */

double PowerModel::BusPower(BusIc * ic)
{
  double power = 0.0;

  // calculate bus length, assume square clusters
  Cluster * clCmp = static_cast<Cluster*>(ic->Parent()->Parent());
  MeshIc * mesh = static_cast<MeshIc*>(clCmp->Ic());
  double busLen = sqrt(config.MaxArea()/(mesh->ColNum()*mesh->RowNum()));
  //cout << "Bus length = " << busLen << " mm" << endl;

  // calculate bus traffic
  for (int s = 0; s < ic->SubnCnt(); ++s) { // for every subnetwork
    // CCP support - number of flits depends on the subnetwork type
    double flitCnt = (1.0+cmpConfig.MemReplySize())/2;
    if (config.SimulateCC()) { // CCP is on
      flitCnt = (s == int(MSGDATA)) ? cmpConfig.MemReplySize() : 1;
    }

    double traffic = 0.0;
    for (int i = 0; i < ic->TotalCompCnt(); ++i) {
      double iTraffic = 0.0;
      for (int o = 0; o < ic->TotalCompCnt(); ++o) {
        iTraffic += ic->Traffic(s, i, o)*flitCnt;
      }
      traffic += iTraffic;
      // add power of the buffer connected to this input port
      //cout << "r = " << r << ", traffic to input port " << i << " = " << iTraffic << endl;
      power += BufferEpf(config.Tech(), config.LinkWidth())*iTraffic*VScalDynPowerUncore(cmpConfig.UVolt())
                 + BufferPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
    }

    // add link power
    power += ( LinkEpf(config.Tech(), config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
               + LinkPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt()) ) * busLen;
  }

  return power;
}

//=======================================================================
/*
 * Calculates power of the uring interconnect.
 * URing is modeled as a set of links and 2x2 routers.
 */

double PowerModel::URingPower(URingIc * ic)
{
  double power = 0.0;

  // calculate ring length, assume square clusters
  Cluster * clCmp = static_cast<Cluster*>(ic->Parent()->Parent());
  MeshIc * mesh = static_cast<MeshIc*>(clCmp->Ic());
  double ringLen = sqrt(config.MaxArea()/(mesh->ColNum()*mesh->RowNum()))*2;
  //cout << "Ring length = " << ringLen << " mm" << endl;
  double linkLen = ringLen/ic->TotalCompCnt();

  // routers
  for (int s = 0; s < ic->SubnCnt(); ++s) { // for every subnetwork
    // CCP support - number of flits depends on the subnetwork type
    double flitCnt = (1.0+cmpConfig.MemReplySize())/2;
    if (config.SimulateCC()) { // CCP is on
      flitCnt = (s == int(MSGDATA)) ? cmpConfig.MemReplySize() : 1;
    }

    for (int r = 0; r < ic->TotalCompCnt(); ++r) {
      // calculate total traffic on all ports
      double traffic = 0.0;
      for (int i = 0; i < URingIc::IPORT_NUM; ++i) {
        double iTraffic = 0.0;
        for (int o = 0; o < URingIc::OPORT_NUM; ++o) {
          iTraffic += ic->Traffic(s, r, i, o)*flitCnt;
        }
        traffic += iTraffic;
        // add power of the link connected to this input port
        if (i != 0) { // if not a component port
          //cout << "r = " << r << ", traffic to input port " << i << " = " << iTraffic << endl;
          power += ( LinkEpf(config.Tech(), config.LinkWidth())*iTraffic*VScalDynPowerUncore(cmpConfig.UVolt())
                     + LinkPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt()) ) * linkLen;
        }
      }

      const int portNum = 2; // power of 2x2 routers (with 2 VCs)
      power += RouterEpf(config.Tech(), portNum, config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
               + RouterPleak(config.Tech(), portNum, config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
    }
  }

  return power;
}

//=======================================================================
/*
 * Calculates power of the bring interconnect.
 * BRing is modeled as a set of links and 3x3 routers.
 */

double PowerModel::BRingPower(BRingIc * ic)
{
  double power = 0.0;

  // calculate ring length, assume square clusters
  Cluster * clCmp = static_cast<Cluster*>(ic->Parent()->Parent());
  MeshIc * mesh = static_cast<MeshIc*>(clCmp->Ic());
  double ringLen = sqrt(config.MaxArea()/(mesh->ColNum()*mesh->RowNum()))*2;
  //cout << "Ring length = " << ringLen << " mm" << endl;
  double linkLen = ringLen/ic->TotalCompCnt();

  // routers
  for (int s = 0; s < ic->SubnCnt(); ++s) { // for every subnetwork
    // CCP support - number of flits depends on the subnetwork type
    double flitCnt = (1.0+cmpConfig.MemReplySize())/2;
    if (config.SimulateCC()) { // CCP is on
      flitCnt = (s == int(MSGDATA)) ? cmpConfig.MemReplySize() : 1;
    }

    for (int r = 0; r < ic->TotalCompCnt(); ++r) {
      // calculate total traffic on all ports
      double traffic = 0.0;
      for (int i = 0; i < BRingIc::IPORT_NUM; ++i) {
        double iTraffic = 0.0;
        for (int o = 0; o < BRingIc::OPORT_NUM; ++o) {
          iTraffic += ic->Traffic(s, r, i, o)*flitCnt;
        }
        traffic += iTraffic;
        // add power of the link connected to this input port
        if (i != 0) { // if not a component port
          //cout << "r = " << r << ", traffic to input port " << i << " = " << iTraffic << endl;
          power += ( LinkEpf(config.Tech(), config.LinkWidth())*iTraffic*VScalDynPowerUncore(cmpConfig.UVolt())
                     + LinkPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt()) ) * linkLen;
        }
      }

      const int portNum = 3; // power of 3x3 routers (with 2 VCs)
      power += RouterEpf(config.Tech(), portNum, config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
               + RouterPleak(config.Tech(), portNum, config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
    }
  }

  return power;
}

//=======================================================================
/*
 * Calculates power of the xbar interconnect.
 * XBar is modeled as a single router.
 */

double PowerModel::XBarPower(XBarIc * ic)
{
  double power = 0.0;

  int portNum = ic->TotalCompCnt();

  static bool POWER_WARNING_ISSUED = false;

  if (portNum > 10) {
    if (!POWER_WARNING_ISSUED) {
      cout << endl << "-W- PowerModel: XBars with max 10 ports are supported (got " << portNum << ")" << endl;
      cout << "-W- PowerModel: Using power model of a 10x10 crossbar" << endl;
      cout << "-W- PowerModel: Similar warnings will not be issued further" << endl;
      POWER_WARNING_ISSUED = true;
    }
    portNum = 10;
  }

  for (int s = 0; s < ic->SubnCnt(); ++s) { // for every subnetwork
    // CCP support - number of flits depends on the subnetwork type
    double flitCnt = (1.0+cmpConfig.MemReplySize())/2;
    if (config.SimulateCC()) { // CCP is on
      flitCnt = (s == int(MSGDATA)) ? cmpConfig.MemReplySize() : 1;
    }

    // calculate total traffic on all ports
    double traffic = 0.0;
    for (int i = 0; i < ic->TotalCompCnt(); ++i) {
      double iTraffic = 0.0;
      for (int o = 0; o < ic->TotalCompCnt(); ++o) {
        iTraffic += ic->Traffic(s, i, o)*flitCnt;
      }
      traffic += iTraffic;
    }

    power += RouterEpf(config.Tech(), portNum, config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
             + RouterPleak(config.Tech(), portNum, config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
  }

  return power;
}

//=======================================================================
/*
 * Writed out PTsim power file using the values from temporary buffers.
 * Assumed a flat mesh interconnect.
 * Ignores power of local busses (between cores and caches).
 */

double PowerModel::DumpPTsimPower(Component * cmp)
{
  const string fname = "./ptsim_power.txt";

  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  cout << "-I- Writing PTsim power file " << fname << endl;

  ofstream out(fname.c_str());

  out << "# This is the power data for PTsim dumped by CMPexplore." << endl;
  out << "# The first line contains the power values for MCs." << endl;
  out << "# The following lines contain the power values for the components of every tile:" << endl;
  out << "# LinkW  RTR  L3  LinkN  L2  Core  L1D  L1I (one line per component)." << endl << endl;

  out << setiosflags(ios::fixed) << setprecision(3);

  // MCs
  bool first = true;

  for (vector<double>::const_iterator it = MCPower_.begin(); it != MCPower_.end(); ++it) {
    if (first) first = false;
    else out << ' ';
    out << (*it);
  }
  out << endl;

  for (int tile_id = 0; tile_id < corePower_.size(); ++tile_id) {
    // LinkW - bi-directional link power
    double link_power = 0.0;
    if (tile_id%mic->ColNum() != 0) {
      link_power = MeshLinkPower_[tile_id*int(RDWEST)] + MeshLinkPower_[(tile_id-1)*int(RDEAST)];
    }
    else { // west links for tiles in the left column are those to MC, approximate traffic by doubling
      link_power = 2.0*MeshLinkPower_[tile_id*int(RDWEST)];
    }
    out << link_power;

    // RTR
    out << '\t' << MeshRouterPower_[tile_id*int(RDWEST)];

    // L3
    out << '\t' << L3Power_[tile_id];

    // LinkN - bi-directional link power
    link_power = 0.0;
    if (tile_id >= mic->ColNum()) { // no north links for tiles in the upper row
      link_power = MeshLinkPower_[tile_id*int(RDNORTH)] +
                   MeshLinkPower_[(tile_id-mic->ColNum())*int(RDSOUTH)];
    }
    out << '\t' << link_power;

    // L2
    out << '\t' << L2Power_[tile_id];

    // Core
    out << '\t' << corePower_[tile_id];

    // L1D
    out << '\t' << L1Power_[tile_id];

    // L1I - approximate as power of data L1
    out << '\t' << L1Power_[tile_id];

    out << endl;
  }

  out.close();
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalDynPower ( double volt )
{
  return volt*volt/(0.8*0.8);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalLeakPower ( double volt )
{
  return volt/0.8;
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalDynPowerProc ( double volt )
{
  double vnom = cmpConfig.ProcVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalLeakPowerProc ( double volt )
{
  double vnom = cmpConfig.ProcVoltNom();
  return volt/vnom;
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalDynPowerUncore ( double volt )
{
  double vnom = cmpConfig.UVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalLeakPowerUncore ( double volt )
{
  double vnom = cmpConfig.UVoltNom();
  return volt/vnom;
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalDynPowerMc ( double volt )
{
  double vnom = cmpConfig.McVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Nominal voltage is assumed to be fixed (0.8 V).
 */

double PowerModel::VScalLeakPowerMc ( double volt )
{
  double vnom = cmpConfig.McVoltNom();
  return volt/vnom;
}

//=======================================================================

  } // namespace power

} //namespace cmpex
