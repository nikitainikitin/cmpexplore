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
#include "PTsim.hpp"

using namespace std;

namespace cmpex {
  
  extern cmp::CmpConfig cmpConfig;
  extern cmp::CmpBuilder cmpBuilder;
  //extern temperature::PTsim ptsim;
  extern Config config;

  namespace temperature {
    class PTsim;
  }
  
  using namespace cmp;

  using namespace temperature;

  namespace power {

//=======================================================================
/*
 * Constructors and destructor
 */

PowerModel::PowerModel() {}

PowerModel::~PowerModel() {}

vector<double> PowerModel::corePower_;
vector<double> PowerModel::L1Power_;
vector<double> PowerModel::L1IPower_;
vector<double> PowerModel::L1DPower_;
vector<double> PowerModel::L2Power_;
vector<double> PowerModel::L3Power_;
vector<double> PowerModel::MCPower_;
vector<double> PowerModel::MeshRouterPower_;
vector<double> PowerModel::MeshLinkPower_;
vector<double> PowerModel::MeshLinkNPower_;
vector<double> PowerModel::MeshLinkWPower_;

    
    /*    vector<double> temperature::PTsim::coreTemp_;
vector<double> PTsim::L1DTemp_;
vector<double> PTsim::L1ITemp_;
vector<double> PTsim::L2Temp_;
vector<double> PTsim::L3Temp_;
vector<double> PTsim::MCTemp_;
vector<double> PTsim::MeshRouterTemp_;
vector<double> PTsim::MeshLinkTemp_; // 4 links per router
    */
//=======================================================================
/*
 * Calculates power of cmp configuration.
 */

double PowerModel::GetTotalPower(Component * cmp)
{
  corePower_.assign(cmpConfig.ProcCnt(), 0.0);
  L1Power_.assign(cmpConfig.ProcCnt(), 0.0);
  L1IPower_.assign(cmpConfig.ProcCnt(), 0.0);
  L1DPower_.assign(cmpConfig.ProcCnt(), 0.0);
  L2Power_.assign(cmpConfig.ProcCnt(), 0.0);
  L3Power_.assign(cmpConfig.MemCnt(), 0.0);
  MCPower_.assign(cmpConfig.MemCtrlCnt(), 0.0);
  MeshRouterPower_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkPower_.assign(cmpConfig.ProcCnt()*4, 0.0);
  MeshLinkNPower_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkWPower_.assign(cmpConfig.ProcCnt(), 0.0);

  double power = 0.0;

  double coretemp = 45.0;
  double l1itemp = 45.0;
  double l1dtemp = 45.0;
  double l2temp = 45.0;
  double l3temp = 45.0;
  double mctemp = 45.0;

  bool warmupdone = PTsim::WarmupDone();

  // Processor leakage and dynamic power
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    // core
    /*if (proc->Active() && !proc->Sleep()) {
      corePower_[p] = proc->Pidle() * VScalDynPowerProc(proc->Volt()) * FScalPowerProc(proc->Freq()) +
	proc->Epi() * VScalDynPowerProc(proc->Volt()) * proc->Thr() +
	proc->CorePleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
      L1Power_[p] = proc->L1Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L1AccessProbability() +
	proc->L1PleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
      L2Power_[p] = proc->L2Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L2AccessProbability() +
	proc->L2PleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
    }
    else if (proc->Active() && proc->Sleep()) {
      corePower_[p] = proc->CorePgPleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
      L1Power_[p] = proc->L1PgPleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
      L2Power_[p] = proc->L2PgPleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());
    }
    else {
      corePower_[p] = 0.0;
      L1Power_[p] = 0.0;
      L2Power_[p] = 0.0;
      }*/

    if (warmupdone) {
      coretemp = PTsim::CoreTemp(p); 
      l1itemp = PTsim::L1ITemp(p); 
      l1dtemp = PTsim::L1DTemp(p); 
      l2temp = PTsim::L2Temp(p); 
    }
    //cout << " Core["<<p<<"] Temperature = " << coretemp << endl;
    //cout << " L1I["<<p<<"] Temperature = " << l1itemp << endl;
    //cout << " L1D["<<p<<"] Temperature = " << l1dtemp << endl;
    //cout << " L2["<<p<<"] Temperature = " << l2temp << endl;

    
    corePower_[p] = proc->Active() ?
      proc->Pidle() * VScalDynPowerProc(proc->Volt()) * FScalPowerProc(proc->Freq()) +
      proc->Epi() * VScalDynPowerProc(proc->Volt()) * proc->Thr() +
      proc->CorePleakOfTemp(273.15+coretemp) * VScalLeakPowerProc(proc->Volt()) : 
      proc->CorePgPleakOfTemp(273.15+coretemp) * VScalLeakPowerProc(proc->Volt());

    // l1
    /*L1Power_[p] = proc->Active() ?
      proc->L1Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L1AccessProbability() +
      proc->L1PleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt()) : 
      proc->L1PgPleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt());*/

    // l1I
    double l1imr = 0.0; // FIXME: miss rate in L1I is assumed zero
    L1IPower_[p] = proc->Active() ?
      (proc->L1IEa() + l1imr*proc->L1IEm()) * VScalDynPowerProc(proc->Volt()) * proc->Thr() +
      proc->L1IPleakOfTemp(273.15+l1itemp) * VScalLeakPowerProc(proc->Volt()) : 
      proc->L1IPgPleakOfTemp(273.15+l1itemp) * VScalLeakPowerProc(proc->Volt());

    // l1D
    double l1dwr = 0.333; // FIXME
    double l1drd = 0.667; // FIXME
    double l1dmr = proc->L2AccessProbability();
    L1DPower_[p] = proc->Active() ?
      (proc->L1DErda()*l1drd + proc->L1DEwra()*l1dwr + 
       l1dmr * (proc->L1DErdm()*l1drd + proc->L1DEwrm()*l1dwr) ) * 
      VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L1AccessProbability() +
      proc->L1DPleakOfTemp(273.15+l1dtemp) * VScalLeakPowerProc(proc->Volt()) : 
      proc->L1DPgPleakOfTemp(273.15+l1dtemp) * VScalLeakPowerProc(proc->Volt());

    /*
    // l2
    L2Power_[p] = proc->Active() ?
      proc->L2Eacc() * VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L2AccessProbability() +
      proc->L2PleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt()) : 
      proc->L2PgPleakOfTemp(300.0) * VScalLeakPowerProc(proc->Volt()); */

    // l2
    double l2wr = 0.333; // FIXME
    double l2rd = 0.667; // FIXME
    double l2mr = proc->L3AccessProbability();
    L2Power_[p] = proc->Active() ?
      (proc->L2Erda()*l2rd + proc->L2Ewra()*l2wr + 
       l2mr * (proc->L2Erdm()*l2rd + proc->L2Ewrm()*l2wr) ) * 
      VScalDynPowerProc(proc->Volt()) * proc->Lambda()*proc->L2AccessProbability() +
      proc->L2PleakOfTemp(273.15+l2temp) * VScalLeakPowerProc(proc->Volt()) : 
      proc->L2PgPleakOfTemp(273.15+l2temp) * VScalLeakPowerProc(proc->Volt());

  }

  double core_power = 0.0;
  core_power += accumulate(corePower_.begin(), corePower_.end(), 0.0);
  //core_power += accumulate(L1Power_.begin(), L1Power_.end(), 0.0);
  core_power += accumulate(L1IPower_.begin(), L1IPower_.end(), 0.0);
  core_power += accumulate(L1DPower_.begin(), L1DPower_.end(), 0.0);
  core_power += accumulate(L2Power_.begin(), L2Power_.end(), 0.0);
  power += core_power;

  //cout << "Core power = " << core_power << "W; ";

  // L3 leakage and dynamic
  double l3PowerTot = 0.0;
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);

    /*L3Power_[m] = mem->Active() ? mem->Eacc() * VScalDynPowerUncore(cmpConfig.UVolt()) * mem->Lambda() +
      mem->PleakOfTemp(300.0) * VScalLeakPowerUncore(cmpConfig.UVolt()) :
      mem->PgPleakOfTemp(300.0) * VScalLeakPowerUncore(cmpConfig.UVolt());*/

    if (warmupdone) {
      l3temp = PTsim::L3Temp(m);    
    }
    //cout << " L3["<<m<<"] Temperature = " << l3temp << endl;

    double l3wr = 0.333; // FIXME
    double l3rd = 0.667; // FIXME
    double l3mr = 0.0; // FIXME
    L3Power_[m] = mem->Active() ?
      (mem->Erda()*l3rd + mem->Ewra()*l3wr + 
       l3mr * (mem->Erdm()*l3rd + mem->Ewrm()*l3wr) ) * 
      VScalDynPowerUncore(cmpConfig.UVolt()) * mem->Lambda() +
      mem->PleakOfTemp(273.15+l3temp) * VScalLeakPowerUncore(cmpConfig.UVolt()) :
      mem->PgPleakOfTemp(273.15+l3temp) * VScalLeakPowerUncore(cmpConfig.UVolt());
  }

  double l3Power = 0.0;
  l3Power += accumulate(L3Power_.begin(), L3Power_.end(), 0.0);
  power += l3Power;

  //cout << "L3 power = " << l3Power << "W; ";

  // MC leakage and dynamic
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    CmpConfig::MemCtrl * memCtrl = cmpConfig.GetMemCtrl(mc);

    /*MCPower_[mc] = memCtrl->active ? memCtrl->eacc * VScalDynPowerMc(cmpConfig.UVolt()) * memCtrl->lambda +
      McPleakOfTemp(300.0) * VScalLeakPowerMc(cmpConfig.UVolt()) :
      McPgPleakOfTemp(300.0) * VScalLeakPowerMc(cmpConfig.UVolt());*/

    if (warmupdone) {
      mctemp = PTsim::MCTemp(mc);    
    }
    //cout << " MC["<<mc<<"] Temperature = " << mctemp << endl;


    MCPower_[mc] = memCtrl->active ? 
      memCtrl->pidle * VScalDynPowerMc(cmpConfig.UVolt()) * FScalPowerMc(cmpConfig.McFreq()) +
      memCtrl->eacc * VScalDynPowerMc(cmpConfig.UVolt()) * memCtrl->lambda +
      McPleakOfTemp(273.15+mctemp) * VScalLeakPowerMc(cmpConfig.UVolt()) :
      McPgPleakOfTemp(273.15+mctemp) * VScalLeakPowerMc(cmpConfig.UVolt());
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
  bool warmupdone = PTsim::WarmupDone();
  double rtrtemp = 45.0;
  double linktemp = 45.0;
  //double linkntemp = 45.0;
  //double linkwtemp = 45.0;

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
          //double link_power = ( LinkEpf(config.Tech(), config.LinkWidth())*iTraffic*VScalDynPowerUncore(cmpConfig.UVolt())
	  //         + LinkPleak(config.Tech(), config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt()) ) * linkLen;
	  if (warmupdone) {
	    if ((i == int(RDEAST)) || (i == int(RDWEST))) {
	      linktemp =  PTsim::MeshLinkWTemp(r);
	      //cout << "DEBUG LINK W TEMP = " << linktemp << endl;
	    }
	    else if ((i == int(RDNORTH)) || (i == int(RDSOUTH))) {
	      linktemp =  PTsim::MeshLinkNTemp(r);
	      //cout << "DEBUG LINK N TEMP = " << linktemp << endl;
	    }
	    else {
	      //cout << "DEBUG LINK TEMP ERROR";
	    }
	  }
	  double link_power = ic->Active(r) ? ic->LinkEpf()/4 * iTraffic * VScalDynPowerUncore(cmpConfig.UVolt())
	    + ic->LinkPleakOfTemp(273.15+linktemp)/4 * VScalLeakPowerUncore(cmpConfig.UVolt()) :
	    ic->LinkPgPleakOfTemp(273.15+linktemp)/4 * VScalLeakPowerUncore(cmpConfig.UVolt());
          power += link_power;
          //cout << "r = " << r << ", power of link at input port " << i << " = " << link_power << endl;
          MeshLinkPower_[r*4+i] += link_power;
	  if ((i == int(RDEAST)) || (i == int(RDWEST))) {
	    MeshLinkWPower_[r] += link_power;
	  }
	  else if ((i == int(RDNORTH)) || (i == int(RDSOUTH))) {
	    MeshLinkNPower_[r] += link_power;
	  }
	}
      }

      // Assume all routers are 5x5, because of the connections to MCs.
      //int portNum = 5;
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
    if (warmupdone) {
      rtrtemp = PTsim::MeshRouterTemp(r);    
    }
    //cout << " RTR["<<r<<"] Temperature = " << rtrtemp << endl;

    /*double link_power = ic->Active(r) ? ic->LinkEpf() * traffic * VScalDynPowerUncore(cmpConfig.UVolt())
	+ ic->LinkPleakOfTemp(273.15+linktemp) * VScalLeakPowerUncore(cmpConfig.UVolt()) :
	ic->LinkPgPleakOfTemp(273.15+linktemp) * VScalLeakPowerUncore(cmpConfig.UVolt());*/

      double router_power = ic->Active(r) ? ic->RouterEpf() * traffic * VScalDynPowerUncore(cmpConfig.UVolt())
	+ ic->RouterPleakOfTemp(273.15+rtrtemp) * VScalLeakPowerUncore(cmpConfig.UVolt()) :
	+ ic->RouterPgPleakOfTemp(273.15+rtrtemp) * VScalLeakPowerUncore(cmpConfig.UVolt());

      //double router_power = RouterEpf(config.Tech(), portNum, config.LinkWidth())*traffic*VScalDynPowerUncore(cmpConfig.UVolt())
      //     + RouterPleak(config.Tech(), portNum, config.LinkWidth())*VScalLeakPowerUncore(cmpConfig.UVolt());
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
 * Writes out PTsim power file using the values from temporary buffers.
 * Assumes a flat mesh interconnect.
 * Ignores power of local busses (between cores and caches).
 */

void PowerModel::DumpPTsimPower(Component * cmp)
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
    /*if (tile_id%mic->ColNum() != 0) {
      link_power = MeshLinkPower_[tile_id*4+int(RDWEST)] + MeshLinkPower_[(tile_id-1)*4+int(RDEAST)];
    }
    else { // west links for tiles in the left column are those to MC, approximate traffic by doubling
      link_power = 2.0*MeshLinkPower_[tile_id*4+int(RDWEST)];
      }*/
    link_power = MeshLinkWPower_[tile_id];
    out << link_power;

    // RTR
    out << '\t' << MeshRouterPower_[tile_id];

    // L3
    out << '\t' << L3Power_[tile_id];

    // LinkN - bi-directional link power
    /*link_power = 0.0;
    if (tile_id >= mic->ColNum()) { // no north links for tiles in the upper row
      link_power = MeshLinkPower_[tile_id*4+int(RDNORTH)] +
                   MeshLinkPower_[(tile_id-mic->ColNum())*4+int(RDSOUTH)];
		   }*/
    link_power = MeshLinkNPower_[tile_id];

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
 * Creates PTsim power vector using the values from temporary buffers.
 * Assumes a flat mesh interconnect.
 * Ignores power of local busses (between cores and caches).
 */

void PowerModel::CreatePTsimPowerVector(Component * cmp, vector<double>& power_vec)
{
  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  int vec_len = MCPower_.size() + corePower_.size()*8;
  power_vec.assign(vec_len, 0.0);

  int cnt = 0;

  // MCs
  for (vector<double>::const_iterator it = MCPower_.begin(); it != MCPower_.end(); ++it) {
    power_vec[cnt++] = *it;
  }

  for (int tile_id = 0; tile_id < corePower_.size(); ++tile_id) {
    // LinkW - bi-directional link power
    double link_power = 0.0;
    /*if (tile_id%mic->ColNum() != 0) {
      link_power = MeshLinkPower_[tile_id*4+int(RDWEST)] + MeshLinkPower_[(tile_id-1)*4+int(RDEAST)];
    }
    else { // west links for tiles in the left column are those to MC, approximate traffic by doubling
      link_power = 2.0*MeshLinkPower_[tile_id*4+int(RDWEST)];
      }*/
    link_power = MeshLinkWPower_[tile_id];
    power_vec[cnt++] = link_power;

    // RTR
    power_vec[cnt++] = MeshRouterPower_[tile_id];

    // L3
    power_vec[cnt++] = L3Power_[tile_id];

    // LinkN - bi-directional link power
    link_power = 0.0;
    /*if (tile_id >= mic->ColNum()) { // no north links for tiles in the upper row
      link_power = MeshLinkPower_[tile_id*4+int(RDNORTH)] +
                   MeshLinkPower_[(tile_id-mic->ColNum())*4+int(RDSOUTH)];
		   }*/
    link_power = MeshLinkNPower_[tile_id];
    power_vec[cnt++] = link_power;

    // L2
    power_vec[cnt++] = L2Power_[tile_id];

    // Core
    power_vec[cnt++] = corePower_[tile_id];

    // L1D
    power_vec[cnt++] = L1DPower_[tile_id];

    // L1I - approximate as power of data L1
    power_vec[cnt++] = L1IPower_[tile_id];
  }
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
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalDynPowerProc ( double volt )
{
  double vnom = cmpConfig.ProcVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalLeakPowerProc ( double volt )
{
  double vnom = cmpConfig.ProcVoltNom();
  return volt/vnom;
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalDynPowerUncore ( double volt )
{
  double vnom = cmpConfig.UVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalLeakPowerUncore ( double volt )
{
  double vnom = cmpConfig.UVoltNom();
  return volt/vnom;
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for dynamic power.
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalDynPowerMc ( double volt )
{
  double vnom = cmpConfig.McVoltNom();
  return volt*volt/(vnom*vnom);
}

//=======================================================================
/*
 * Returns voltage scaling coefficient for leakage power.
 * Power values have been obtained with McPAT at nominal voltage (1.0 V).
 */

double PowerModel::VScalLeakPowerMc ( double volt )
{
  double vnom = cmpConfig.McVoltNom();
  return volt/vnom;
}

//=======================================================================
/*
 * Returns frequency scaling coefficient for dynamic power.
 * Frequency values have been obtained with McPAT at nominal voltage (1.0 V)
 * and maximum frequency (3.5 GHz).
 */

double PowerModel::FScalPowerProc ( double freq )
{
  double fnom = cmpConfig.ProcFreqMax();
  return freq/fnom;
}

double PowerModel::FScalPowerMc ( double freq )
{
  double fnom = cmpConfig.McFreqMax();
  return freq/fnom;
}

//=======================================================================
/*
 * Returns minimum voltage for a given frequency.
 * Parameters obtained with McPAT at nominal voltage (1.0 V)
 * and maximum frequency (3.5 GHz).
 */

double PowerModel::VoltAtFreqProc ( double freq )
{
  double fmin = cmpConfig.ProcMinVoltFreq();
  double fmax = cmpConfig.ProcFreqMax();
  double vmin = cmpConfig.ProcVoltMin();
  double vmax = cmpConfig.ProcVoltMax();
  return freq > fmin ?  vmin+(vmax-vmin)/(fmax-fmin)*(freq-fmin) : vmin;
}



//=======================================================================
/*
 * Returns leakage of memory controoler.
 */
double PowerModel::McPleakOfTemp ( double tmp ) {
  return cmpBuilder.McLeakageOfTemp(tmp);
}

//=======================================================================
/*
 * Returns leakage of memory controoler in power gating state
 */
double PowerModel::McPgPleakOfTemp ( double tmp ) {
  return cmpBuilder.McPgLeakageOfTemp(tmp);
}


  } // namespace power

} //namespace cmpex
