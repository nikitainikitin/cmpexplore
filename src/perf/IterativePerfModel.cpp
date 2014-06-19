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
#include <fstream>
#include <sstream>

#include <cstdlib>
#include <unistd.h>

#include "IterativePerfModel.hpp"
#include "Component.hpp"
#include "CmpConfig.hpp"
#include "Config.hpp"
#include "Cluster.hpp"
#include "Processor.hpp"
#include "Memory.hpp"
#include "Interconnect.hpp"
#include <Timer.hpp>
#include "Debug.hpp"
#include "StatConfig.hpp"

using std::cout;
using std::endl;
using std::ofstream;
using std::ostringstream;
using std::fabs;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  extern Config config;
  extern Debug debug;

  using namespace cmp;
  using stat::StatMetrics;

  namespace perf {

  const double thrEps = 0.02; // precision in throughput
  const double latEps = 0.02; // precision in latency

//#define ALWAYS_BISECTION
//#define STATIC_LAT_ONLY

//=======================================================================
/*
 * Constructors and destructor
 */

IterativePerfModel::IterativePerfModel () {}

IterativePerfModel::~IterativePerfModel () {}

//=======================================================================
/*
 * Main method to estimate CMP performance.
 * Uses either fixed-point or bisection.
 */

StatMetrics * IterativePerfModel::Run ()
{
  // sanity check: there should be at least one actively running core
  bool active_core_exists = false;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    if (proc->Active() && proc->Freq() > E_DOUBLE) active_core_exists = true;
  }
  if (!active_core_exists) {
    //cout << "-W- There are no actively runnning cores, throughput is 0" << endl;
    InitModels(); // for the power model to work
    return new StatMetrics(0.0, MAX_DOUBLE);
  }

  //return RunBisectionFp();
  return RunSubgradientFp();
}

//=======================================================================
/*
 * Estimate CMP performance: try FP; if not possible, run subgradient.
 */

StatMetrics * IterativePerfModel::RunSubgradientFp ()
{
  StatMetrics * sm = RunFixedPoint();

  if (sm->Throughput() < E_DOUBLE) {
    DEBUG(1, "FixedPoint failed, running Subgradient" << endl);
    delete sm;
    sm = RunSubgradient();
  }
  else {
    DEBUG(1, "FixedPoint succeeded" << endl);
  }

  return sm;
}

//=======================================================================
/*
 * Estimate CMP performance using fixed-point iteration.
 * Units: latency [ns], throughput [instr. per ns], traffic [req. per ns]
 */

StatMetrics * IterativePerfModel::RunFixedPoint(double statThr, double statInj)
{
  double prevThr = 0.0;
  double systemThr = 0.0;
  double systemLat = 0.0;
  double systemTr = 0.0;
  double systemInj = 0.0; // injection rate to IC
  int iter = 0;
  bool cont = false;

  do {
    prevThr = systemThr;

    systemThr = 0.0;
    systemLat = 0.0;
    systemTr = 0.0;
    systemInj = 0.0;
    cont = false;

    // Optimize performance by precalculating L3 to Mc latencies
    if (config.SimulateCC()) PrecalcL3ToMcLatencies(iter ? true : false);

    // calculate traffic rates
    vector<double> procLat;
    vector<double> procThr;
    vector<double> procRate;
    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Processor * proc = cmpConfig.GetProcessor(p);

      double lat = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalculateProcLatency(p, iter ? true : false) : 0.0;
      DEBUG(2, "P" << p << ": avg lat = " << lat << " cycles" << endl);
      systemLat += lat;
      procLat.push_back(lat);

      double thr = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalcThr(proc->Ipc()*proc->Freq(), proc->Mpi(), lat) : 0.0;
      proc->Thr(thr);
      DEBUG(2, "P" << p << ": thr = " << thr << endl);
      systemThr += thr;
      procThr.push_back(thr);

      double totalTraffic = thr*proc->Mpi();
      proc->Lambda(totalTraffic);
      DEBUG(2, "P" << p << ": total memory traffic = " << totalTraffic << endl);
      systemTr += totalTraffic;
      systemInj += totalTraffic*(proc->L3AccessProbability()+proc->MainMemAccessProbability());
      procRate.push_back(totalTraffic);
    }

    DEBUG(1, "Iter " << iter << ": system throughput = " << systemThr << endl);
    DEBUG(1, "Iter " << iter << ": avg lat = " << systemLat/cmpConfig.ProcCnt() << endl);
//    cout << "Iter " << iter << ": system throughput = " << systemThr << endl;
//    cout << "Iter " << iter << ": avg lat = " << systemLat/cmpConfig.ProcCnt() << endl;

    // init models
    InitModels();

    // mark paths
    MarkPaths(procRate);

    // run models
    if(EstimateDelays())
      return new StatMetrics(0.0, MAX_DOUBLE);

    ++iter;

  } while (std::fabs(prevThr - systemThr) >= thrEps*systemThr);

  StatMetrics * sc = new StatMetrics(systemThr, systemLat/cmpConfig.ProcCnt(), systemInj);
  sc->StatTraffic(statInj);
  sc->StatThroughput(statThr);
  return sc;
}

//=======================================================================
/*
 * Estimate CMP performance using subgradient method.
 * Units: latency [ns], throughput [instr. per ns], traffic [req. per ns]
 */

StatMetrics * IterativePerfModel::RunSubgradient ()
{
  double systemThr = 0.0;
  double systemTr = 0.0;
  double systemLat = 0.0;
  double systemInj = 0.0;

  // init models
  InitModels();

  vector<double> procRateCur(cmpConfig.ProcCnt(), E_DOUBLE);
  vector<double> step(cmpConfig.ProcCnt(), 0.125);
  vector<int> dir(cmpConfig.ProcCnt(), 1);
  int grad_iter = 0;
  double prevGap = 0;
  double totalGap = 0;

  // boolean property if dimension has changed its gradient direction
  vector<int> changedDir(cmpConfig.ProcCnt(), 0);

  double precision;

  do {
    prevGap = totalGap;
    totalGap = 0;
    systemThr = 0.0;
    systemLat = 0.0;
    systemTr = 0.0;
    precision = 0.0;

    // Optimize performance by precalculating L3 to Mc latencies
    if (config.SimulateCC()) PrecalcL3ToMcLatencies(true);

    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Processor * proc = cmpConfig.GetProcessor(p);

      double lat = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalculateProcLatency(p, true) : 0.0;
      systemLat += lat;

      double thr = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalcThr(proc->Ipc()*proc->Freq(), proc->Mpi(), lat) : 0.0;
      proc->Thr(thr);
      systemThr += thr;

      double totalTraffic = thr*proc->Mpi();
      proc->Lambda(totalTraffic);
      systemTr += totalTraffic;

      double lat_star = 1.0/procRateCur[p] - 1.0/(proc->Mpi()*proc->Ipc()*proc->Freq());

      double gap = lat - lat_star;
      totalGap += fabs(gap);
      precision += std::min(lat, lat_star)*latEps; // 2% of current latency

      if (gap < 0) {
        if (dir[p] == -1) {
          //step[p] /= 2;
          dir[p] = 1;
          changedDir[p] = 1;
        }
        procRateCur[p] += step[p];
      }
      else {
        if (dir[p] == 1) {
          //step[p] /= 2;
          dir[p] = -1;
          changedDir[p] = 1;
        }
        procRateCur[p] -= step[p];
      }
    }

    // check if step has to be decreased (i.e. all dimensions changed directions)
    bool decrease = true;
    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Processor * proc = cmpConfig.GetProcessor(p);
      if (proc->Active() && proc->Freq() > E_DOUBLE && !changedDir[p]) {
        decrease = false; break;
      }
    }

    if (decrease) {
      changedDir.assign(cmpConfig.ProcCnt(), 0);
      step.assign(cmpConfig.ProcCnt(), step[0]/2);
      //cout << "Changed step to " << step[0] << endl;
    }

    DEBUG(1, "Iter " << grad_iter << ": system throughput = " << systemThr);
    DEBUG(1, ", avg lat = " << systemLat/cmpConfig.ProcCnt());
    DEBUG(1, ", GAP = " << totalGap << endl);

    // init models
    InitModels();

    // mark paths
    MarkPaths(procRateCur);

    // run models
    EstimateDelays(true);

    ++grad_iter;

    if (grad_iter == 2001)
      DEBUG(1, "Subgradient hasn't converged in 2000 iters, totalGap = "
            << totalGap << ", precision (2%) = " << precision << endl);

    if (grad_iter > 10000) {
      cout << "-W- Subgradient hasn't converged in 10000 iters, totalGap = "
            << totalGap << ", precision (2%) = " << precision << endl;
      break;
    }

  } while (totalGap > precision);

  if (grad_iter > 2001 && grad_iter <= 10000)
    DEBUG(1, "Subgradient took " << grad_iter << " iters to converge" << endl);

  //if (grad_iter > 50)
  //  cout << "Gradient took " << grad_iter << " iterations" << endl;

  StatMetrics * scs = new StatMetrics(systemThr, systemLat/cmpConfig.ProcCnt(), systemInj);
  return scs;
}

//=======================================================================
/*
 * Estimate CMP performance using bisection or fixed-point iteration.
 * Units: latency [ns], throughput [instr. per ns], traffic [req. per ns]
 */

StatMetrics * IterativePerfModel::RunBisectionFp ()
{
  if (fabs(cmpConfig.McFreq() - cmpConfig.UFreq()) > E_DOUBLE ) {
    cout << "Bisection is obsolete with MC BW, check subgradient instead" << endl;
    exit(1);
  }

  double prevThr = 0.0;
  double systemThr = 0.0;
  double systemTr = 0.0;
  double systemLat = 0.0;
  double systemInj = 0.0;
  int iter = 0;
  bool modelError = false;

  // ==================== 1. check if models can be run with static latency ================
  prevThr = systemThr;

  systemThr = 0.0;
  systemLat = 0.0;

  // init models
  InitModels();

  // Optimize performance by precalculating L3 to Mc latencies
  if (config.SimulateCC()) PrecalcL3ToMcLatencies(false);

  // calculate traffic rates
  vector<double> procLat;
  vector<double> procThr;
  vector<double> procRate;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    double lat = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                   CalculateProcLatency(p, false) : 0.0;
    DEBUG(2, "P" << p << ": avg lat = " << lat << " cycles" << endl);
    systemLat += lat;
    procLat.push_back(lat);

    double thr = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                   CalcThr(proc->Ipc()*proc->Freq(), proc->Mpi(), lat) : 0.0;
    proc->Thr(thr);
    DEBUG(2, "P" << p << ": thr = " << thr << endl);
    systemThr += thr;
    procThr.push_back(thr);

    double totalTraffic = thr*proc->Mpi();
    proc->Lambda(totalTraffic);
    DEBUG(2, "P" << p << ": total memory traffic = " << totalTraffic << endl);
    systemTr += totalTraffic;
    systemInj += totalTraffic*(proc->L3AccessProbability()+proc->MainMemAccessProbability());
    procRate.push_back(totalTraffic);
  }

#ifdef STATIC_LAT_ONLY
  StatMetrics * scs = new StatMetrics(systemThr, systemLat/cmpConfig.ProcCnt(), systemInj);
  scs->StatTraffic(systemInj);
  scs->StatThroughput(systemThr);
  return scs;
#endif

  DEBUG(1, "Iter " << iter << ": system throughput = " << systemThr << endl);
  DEBUG(1, "Iter " << iter << ": avg lat = " << systemLat/cmpConfig.ProcCnt() << endl);
  DEBUG(1, "Iter " << iter << ": avg traffic = " << systemTr/cmpConfig.ProcCnt() << endl);
  //cout << 4.0*systemInj << '\t'; // REMOVE: static rate
  double statThr = systemThr;
  double statInj = systemInj;

  // mark paths
  MarkPaths(procRate);

  // run models
  modelError = EstimateDelays();

#ifndef ALWAYS_BISECTION
  if (!modelError) // fixed point will proceed successfully
    return RunFixedPoint(statThr, statInj);
#endif

  //cout << "Bisection invoked" << endl;

  // ==================== 2. find adjustment bounds ================

  DEBUG(1, "ModelError encountered with static latency -> invoke fixLambda flow" << endl);

  vector<double> procRateMax(procRate.begin(), procRate.end());
  vector<double> procRateMin(procRate.size(), 0.0);
  vector<double> procRateAvg(procRate.size(), 0.0);
  int bisect_cnt = 0;

  double prevLat;

  double delta_throughput = 0.0;

  double prevGap = 0.0;
  double curGap = 0.0;

  do {

    prevLat = systemLat;
    prevThr = systemThr;

    for (int p = 0; p < procRateAvg.size(); ++p) {
      procRateAvg[p] = (procRateMax[p] + procRateMin[p]) / 2.0;
      /*cout << "Min = " << procRateMin[p] << ", Max = " << procRateMax[p]
           << ", Avg = " << procRateAvg[p] << endl;*/
    }

    modelError = false;

    // init models
    InitModels();

    // mark paths
    MarkPaths(procRateAvg);

    // run models
    modelError = EstimateDelays(true);

    // if model error happens, then the average rate is too big - set it as maximum
    if (modelError) {
      DEBUG(1, "Iter " << bisect_cnt << ": Model error with this traffic rate, continue..." << endl);
      procRateMax.assign(procRateAvg.begin(), procRateAvg.end());
      bisect_cnt++;
      prevThr = 0.0;
      continue;
    }

    systemLat = 0.0;
    systemThr = 0.0;
    systemTr = 0.0;
    systemInj = 0.0;
    double Fsys = 0.0;

    // Optimize performance by precalculating L3 to Mc latencies
    if (config.SimulateCC()) PrecalcL3ToMcLatencies(true);

    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Processor * proc = cmpConfig.GetProcessor(p);

      double lat = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalculateProcLatency(p, true) : 0.0;
      double lat_star = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                          1.0/procRateAvg[p] - 1.0/(proc->Mpi()*proc->Ipc()*proc->Freq()) : 0.0;

      double lat_avg = lat;
      DEBUG(2, "P" << p << ": avg lat = " << lat_avg << " cycles" << endl);
      systemLat += lat_avg;

      double thr = (proc->Active() && proc->Freq() > E_DOUBLE) ?
                     CalcThr(proc->Ipc()*proc->Freq(), proc->Mpi(), lat_avg): 0.0;
      proc->Thr(thr);
      DEBUG(2, "P" << p << ": thr = " << thr << endl);
      systemThr += thr;

      double totalTraffic = thr*proc->Mpi();
      proc->Lambda(totalTraffic);
      DEBUG(2, "P" << p << ": total memory traffic = " << totalTraffic << endl);
      systemTr += totalTraffic;
      systemInj += totalTraffic*(proc->L3AccessProbability()+proc->MainMemAccessProbability());

      double F = lat - lat_star;
      Fsys += F;

      //cout << "Max-Min rate for core " << p << " = " << (procRateMax[p]-procRateMin[p]) << endl;
      //if (fabs(F) > 1) cout << "F[" << p << "] = " << F << ' ';

      if (F < 0) procRateMin[p] = procRateAvg[p];
      else procRateMax[p] = procRateAvg[p];
    }

    DEBUG(1, "Iter " << bisect_cnt << ": Avg thr = " << systemThr << endl);
    DEBUG(1, "Iter " << bisect_cnt << ": Avg latency = " << systemLat/cmpConfig.ProcCnt() << endl);
    DEBUG(1, "Iter " << bisect_cnt << ": Avg traffic = " << systemTr/cmpConfig.ProcCnt()
         << ", func gap = " << std::fabs(Fsys) << endl);
    DEBUG(1, "Iter " << bisect_cnt << ": DeltaThr = " << delta_throughput << endl);

    curGap = std::fabs(Fsys);
    bisect_cnt++;

  } while (bisect_cnt == 1 || prevThr < E_DOUBLE || bisect_cnt <= 20);


  double newThr = systemThr;
  double newLat = systemLat;

  StatMetrics * sc = new StatMetrics(newThr, newLat/cmpConfig.ProcCnt(), systemInj);
  sc->StatTraffic(statInj);
  sc->StatThroughput(statThr);
  return sc;
}

//=======================================================================
/*
 * Procedure to mark paths in two-level hierarchical CMP.
 */

void IterativePerfModel::MarkPaths (const vector<double>& procRates)
{
  config.SimulateCC() ? MarkPathsCC(procRates) : MarkPathsNoCC(procRates);
}

//=======================================================================
/*
 * Procedure to mark paths in two-level hierarchical CMP.
 * Should be used when cache coherence protocol is not considered
 * and all ICs have only one physical subnetwork.
 * Accumulates traffic rates of all flows per router port,
 * assuming XY-routing algorithm. This defines the total arrival rate
 * \lambda per port, that is required for analytical latency model.
 */

void IterativePerfModel::MarkPathsNoCC (const vector<double>& procRates)
{
  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());

  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    // handle L3 memories
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      Memory * mem = cmpConfig.GetMemory(m);

      double trafficToMemMiss = procRates[p]*proc->MainMemAccessProbability()*proc->L3ProbDistr()[m];//pa[m];
      double trafficToMem = procRates[p]*proc->L3AccessProbability()*proc->L3ProbDistr()[m];//pa[m];
      double trafficToMemAccess = trafficToMem + trafficToMemMiss;

      // in clusters
      if (!cmpConfig.FlatMeshIc()) {
        for (int c = 0; c < clCmp->CompCnt(); ++c) {
          Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));

          if (comp->ProcOwner(p) && comp->MemOwner(m)) {
            comp->Ic()->MarkPathCompToComp(
                proc->ClIdx(), mem->ClIdx(), trafficToMem, UShort(MSGNOCC));
            comp->Ic()->MarkPathCompToComp(
                mem->ClIdx(), proc->ClIdx(), trafficToMem, UShort(MSGNOCC));
          }
          else if (comp->ProcOwner(p) && !comp->MemOwner(m)) {
            comp->Ic()->MarkPathCompToIface(
                proc->ClIdx(), trafficToMem, UShort(MSGNOCC));
            comp->Ic()->MarkPathIfaceToComp(
                proc->ClIdx(), trafficToMem, UShort(MSGNOCC));
          }
          else if (!comp->ProcOwner(p) && comp->MemOwner(m)) {
            comp->Ic()->MarkPathCompToIface(
                mem->ClIdx(), trafficToMem, UShort(MSGNOCC));
            comp->Ic()->MarkPathIfaceToComp(
                mem->ClIdx(), trafficToMem, UShort(MSGNOCC));
          }
        }
      }

      // global mesh
      if (clCmp->ProcOwner(p) != clCmp->MemOwner(m)) {
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->ProcOwner(p)->ClIdx(), clCmp->MemOwner(m)->ClIdx(), trafficToMem, UShort(MSGNOCC));
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->MemOwner(m)->ClIdx(), clCmp->ProcOwner(p)->ClIdx(), trafficToMem, UShort(MSGNOCC));
      }

      // L3
      mem->Lambda(mem->Lambda()+trafficToMem);
      mem->LambdaMiss(mem->LambdaMiss()+trafficToMemMiss);
      mem->LambdaAccess(mem->LambdaAccess()+trafficToMemAccess);
    } // L3 memories

    // handle memory controllers
    double trafficToMC =
      procRates[p]*proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt();

    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      // in clusters
      if (!cmpConfig.FlatMeshIc()) {
        for (int c = 0; c < clCmp->CompCnt(); ++c) {
          Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));
          if (comp->ProcOwner(p)) {
            // we're in the cluster, so MC is attached to a top-level mesh,
            // hence we have to mark path to iface (not MC!)
            comp->Ic()->MarkPathCompToIface(
                proc->ClIdx(), trafficToMC, UShort(MSGNOCC));
            comp->Ic()->MarkPathIfaceToComp(
                proc->ClIdx(), trafficToMC, UShort(MSGNOCC));
          }
        }
      }

      // global mesh
      clCmp->Ic()->MarkPathCompToMemCtrl(
          clCmp->ProcOwner(p)->ClIdx(), mc, trafficToMC, UShort(MSGNOCC));
      clCmp->Ic()->MarkPathMemCtrlToComp(
          mc, clCmp->ProcOwner(p)->ClIdx(), trafficToMC, UShort(MSGNOCC));

      // MC
      cmpConfig.GetMemCtrl(mc)->lambda = cmpConfig.GetMemCtrl(mc)->lambda+trafficToMC;
    } // memory controllers
  }
}

//=======================================================================
/*
 * Procedure to mark paths in two-level hierarchical CMP,
 * cache coherence protocol is considered
 * and all ICs have three physical subnetworks.
 * Accumulates traffic rates of all flows per router port,
 * assuming XY-routing algorithm. This defines the total arrival rate
 * \lambda per port, that is required for analytical latency model.
 */

void IterativePerfModel::MarkPathsCC (const vector<double>& procRates)
{
  const double evictProb = 0.5;

  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());

  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);

    double trafficMemToMc = 0.0; // total traffic from this L3 to any MC

    // mark paths from Procs to L3s
    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Processor * proc = cmpConfig.GetProcessor(p);

      double trafficProcToMem = procRates[p]*
          (proc->L3AccessProbability()+proc->MainMemAccessProbability())*proc->L3ProbDistr()[m];

      trafficMemToMc += procRates[p]*
          proc->MainMemAccessProbability()*proc->L3ProbDistr()[m]/cmpConfig.MemCtrlCnt();

      // in clusters
      if (!cmpConfig.FlatMeshIc()) {
        for (int c = 0; c < clCmp->CompCnt(); ++c) {
          Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));

          if (comp->ProcOwner(p) && comp->MemOwner(m)) { // intra-cluster communication
            comp->Ic()->MarkPathCompToComp(
                proc->ClIdx(), mem->ClIdx(), trafficProcToMem, UShort(MSGREQ));
            comp->Ic()->MarkPathCompToComp(
                mem->ClIdx(), proc->ClIdx(), trafficProcToMem, UShort(MSGACK));
            comp->Ic()->MarkPathCompToComp(
                proc->ClIdx(), mem->ClIdx(), trafficProcToMem*evictProb, UShort(MSGDATA));
            comp->Ic()->MarkPathCompToComp(
                mem->ClIdx(), proc->ClIdx(), trafficProcToMem, UShort(MSGDATA));
          }
          else if (comp->ProcOwner(p) && !comp->MemOwner(m)) {
            comp->Ic()->MarkPathCompToIface(
                proc->ClIdx(), trafficProcToMem, UShort(MSGREQ));
            comp->Ic()->MarkPathIfaceToComp(
                proc->ClIdx(), trafficProcToMem, UShort(MSGACK));
            comp->Ic()->MarkPathCompToIface(
                proc->ClIdx(), trafficProcToMem*evictProb, UShort(MSGDATA));
            comp->Ic()->MarkPathIfaceToComp(
                proc->ClIdx(), trafficProcToMem, UShort(MSGDATA));
          }
          else if (!comp->ProcOwner(p) && comp->MemOwner(m)) {
            comp->Ic()->MarkPathIfaceToComp(
                mem->ClIdx(), trafficProcToMem, UShort(MSGREQ));
            comp->Ic()->MarkPathCompToIface(
                mem->ClIdx(), trafficProcToMem, UShort(MSGACK));
            comp->Ic()->MarkPathIfaceToComp(
                mem->ClIdx(), trafficProcToMem*evictProb, UShort(MSGDATA));
            comp->Ic()->MarkPathCompToIface(
                mem->ClIdx(), trafficProcToMem, UShort(MSGDATA));
          }
        }
      }

      // global mesh
      if (clCmp->ProcOwner(p) != clCmp->MemOwner(m)) { // if not in the same cluster
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->ProcOwner(p)->ClIdx(), clCmp->MemOwner(m)->ClIdx(), trafficProcToMem, UShort(MSGREQ));
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->MemOwner(m)->ClIdx(), clCmp->ProcOwner(p)->ClIdx(), trafficProcToMem, UShort(MSGACK));
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->ProcOwner(p)->ClIdx(), clCmp->MemOwner(m)->ClIdx(), trafficProcToMem*evictProb, UShort(MSGDATA));
        clCmp->Ic()->MarkPathCompToComp(
            clCmp->MemOwner(m)->ClIdx(), clCmp->ProcOwner(p)->ClIdx(), trafficProcToMem, UShort(MSGDATA));
      }

      // L3
      mem->Lambda(mem->Lambda()+trafficProcToMem);
    }

    // mark paths from L3s to MCs (i.e. from Procs to MCs via L3s)
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      // in clusters
      if (!cmpConfig.FlatMeshIc()) {
        for (int c = 0; c < clCmp->CompCnt(); ++c) {
          Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));
          if (comp->MemOwner(m)) {
            // we're in the cluster, so MC is attached to a top-level mesh,
            // hence we have to mark path to iface (not MC!)
            comp->Ic()->MarkPathCompToIface(
                mem->ClIdx(), trafficMemToMc, UShort(MSGREQ));
            comp->Ic()->MarkPathIfaceToComp(
                mem->ClIdx(), trafficMemToMc, UShort(MSGACK));
            comp->Ic()->MarkPathCompToIface(
                mem->ClIdx(), trafficMemToMc*evictProb, UShort(MSGDATA));
            comp->Ic()->MarkPathIfaceToComp(
                mem->ClIdx(), trafficMemToMc, UShort(MSGDATA));
          }
        }
      }

      // global mesh
      clCmp->Ic()->MarkPathCompToMemCtrl(
          clCmp->MemOwner(m)->ClIdx(), mc, trafficMemToMc, UShort(MSGREQ));
      clCmp->Ic()->MarkPathMemCtrlToComp(
          mc, clCmp->MemOwner(m)->ClIdx(), trafficMemToMc, UShort(MSGACK));
      clCmp->Ic()->MarkPathCompToMemCtrl(
          clCmp->MemOwner(m)->ClIdx(), mc, trafficMemToMc*evictProb, UShort(MSGDATA));
      clCmp->Ic()->MarkPathMemCtrlToComp(
          mc, clCmp->MemOwner(m)->ClIdx(), trafficMemToMc, UShort(MSGDATA));

      // MC
      cmpConfig.GetMemCtrl(mc)->lambda = cmpConfig.GetMemCtrl(mc)->lambda+trafficMemToMc;
      // L3
      mem->Lambda(mem->Lambda()+trafficMemToMc);
    } // memory controllers
  }
}

//=======================================================================
/*
 * Perform initialization of analytical models for ICs, caches and MCs.
 */

void IterativePerfModel::InitModels ()
{
  // ICs
  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());
  clCmp->Ic()->InitModel();

  if (!cmpConfig.FlatMeshIc()) {
    for (int c = 0; c < clCmp->CompCnt(); ++c) {
      Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));
      comp->Ic()->InitModel();
    }
  }

  // caches
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    cmpConfig.GetMemory(m)->Lambda(0.0);
    cmpConfig.GetMemory(m)->LambdaMiss(0.0);
    cmpConfig.GetMemory(m)->LambdaAccess(0.0);
    cmpConfig.GetMemory(m)->BufDelay(0.0);
  }

  // MCs
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    cmpConfig.GetMemCtrl(mc)->lambda = 0.0;
    cmpConfig.GetMemCtrl(mc)->bufDelay = 0.0;
  }
}


//=======================================================================
/*
 * Run analytical models and estimate contention delays in ICs, caches and MCs.
 */

bool IterativePerfModel::EstimateDelays(bool fixNegDelays)
{
  bool error = false;

  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());

  // ICs
  if (!cmpConfig.FlatMeshIc()) {
    for (int c = 0; c < clCmp->CompCnt(); ++c) {
      Cluster * comp = static_cast<Cluster*>(clCmp->GetComponent(c));
      if (comp->Ic()->EstimateBufferDelays(fixNegDelays))
        error = true;
    }
  }

  if (clCmp->Ic()->EstimateBufferDelays(fixNegDelays))
    error = true;

  // caches
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);

    // Pollaczek-Khinchin formula for an M/D/1 server.
    // Memory service time is assumed to be 1 cycle;
    // should be changed if Fmem != Fnoc
    //double delay = mem->Lambda()/(2.0*(1.0-mem->Lambda())); // [uncore cycles]
    double sTime = 1.0/cmpConfig.UFreq(); // [ns]
    DASSERT(mem->Lambda()*sTime < 1.0);
    double delay = mem->Lambda()*sTime*sTime/(2.0*(1.0-mem->Lambda()*sTime));

    if (delay < -E_DOUBLE) {
      if (fixNegDelays) {
        delay = 1.0e6;
      }
      else {
        error = true;
      }
    }

    if (delay < 0)
      cout << "Mem " << m << ": lambda=" << mem->Lambda() << ", delay = " << delay << endl;

    mem->BufDelay(delay);
  }

  // MCs
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    CmpConfig::MemCtrl * memCtrl = cmpConfig.GetMemCtrl(mc);

    // Pollaczek-Khinchin formula for an M/D/1 server.
    //double sTime = cmpConfig.UFreq()/cmpConfig.McFreq(); // [uncore cycles]
    double sTime = 1.0/cmpConfig.McFreq(); // [ns]
    DASSERT(memCtrl->lambda*sTime < 1.0);
    double delay = memCtrl->lambda*sTime*sTime/(2.0*(1.0-memCtrl->lambda*sTime));

    if (delay < -E_DOUBLE) {
      if (fixNegDelays) {
        delay = 1.0e6;
      }
      else {
        error = true;
      }
    }

//    if (delay < 0) {
//      cout << "Mc " << mc << ": lambda=" << memCtrl->lambda << ", sTime = " << sTime <<
//              ", delay = " << delay << endl;
    //cout << "MC-" << mc << " utilization = " << memCtrl->lambda*sTime << endl;

    memCtrl->bufDelay = delay;
  }

  return error;
}

//=======================================================================

  } // namespace perf

} // namespace cmpex
