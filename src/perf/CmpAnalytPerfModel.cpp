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
#include <algorithm>

#include "CmpAnalytPerfModel.hpp"
#include "Component.hpp"
#include "CmpConfig.hpp"
#include "Config.hpp"
#include "Cluster.hpp"
#include "Processor.hpp"
#include "Memory.hpp"
#include "Interconnect.hpp"
#include "Debug.hpp"

using std::cout;
using std::endl;
using std::max;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  extern Config config;
  extern Debug debug;

  using namespace cmp;

  namespace perf {

//=======================================================================
/*
 * Constructors and destructor
 */

CmpAnalytPerfModel::CmpAnalytPerfModel () {}

CmpAnalytPerfModel::~CmpAnalytPerfModel () {}

//=======================================================================
/*
 * Calculates average latency for processor 'idx'.
 * NOTE: latency unit is [ns]
 */

double CmpAnalytPerfModel::CalculateProcLatency (
    UShort idx, bool dynamic)
{
  Processor * proc = cmpConfig.GetProcessor(idx);
  if (!proc->Active()) // approximate cores that are off by a very high latency
    return MAX_DOUBLE;

  return config.SimulateCC() ? CalculateProcLatencyCC(idx, dynamic) :
                                  CalculateProcLatencyNoCC(idx, dynamic);
}

//=======================================================================
/*
 * Calculates average latency for processor 'idx'.
 * Cache coherence is not considered.
 * Accesses to L3 and MC are done independently and
 * only Request and Reply transactions are considered.
 * NOTE: latency unit is [ns]
 */

double CmpAnalytPerfModel::CalculateProcLatencyNoCC (
    UShort idx, bool dynamic)
{
  Processor * proc = cmpConfig.GetProcessor(idx);

  // L1 latency [ns]
  double l1Latency = proc->L1Lat() / proc->Freq();

  // L2 latency [ns]
  double l2Latency = proc->L2Lat() / proc->Freq();

  // L3 latency [ns]
  double l3Latency = 0.0;
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    if (idx == 0)
      DEBUG(3, "pa[" << m << "] = " << proc->L3ProbDistr()[m]
               << ", latency = " << LatencyProcToMem(idx, m, dynamic) << endl);
    l3Latency += proc->L3ProbDistr()[m]*LatencyProcToMem(idx, m, dynamic);
  }

  // Main memory latency [ns]
  double mainMemLatency = 0.0;
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    if (idx == 0)
      DEBUG(3, "prob[" << mc << "] = " << (1.0/cmpConfig.MemCtrlCnt())
             << ", latency = " << LatencyProcToMemCtrl(idx, mc, dynamic) << endl);
    mainMemLatency += (1.0/cmpConfig.MemCtrlCnt()) *
                       LatencyProcToMemCtrl(idx, mc, dynamic);
  }

  const double L1AccProb = proc->L1AccessProbability();
  const double L2AccProb = proc->L2AccessProbability();
  const double L3AccProb = proc->L3AccessProbability();
  const double MMAccProb = proc->MainMemAccessProbability();

  DEBUG(3, "P" << idx << ": p(L2) = " << L2AccProb
         << ", l2Latency = " << l2Latency
         << ", p(L3) = " << L3AccProb
         << ", l3Latency = " << l3Latency
         << ", p(MM) = " << MMAccProb
         << ", MMLatency = " << mainMemLatency << endl);

  return ( ( proc->OoO() ? 0.0 : L1AccProb*l1Latency +
             L2AccProb*l2Latency ) +
          L3AccProb*l3Latency +
          MMAccProb*mainMemLatency);
}

//=======================================================================
/*
 * Calculates average latency for processor 'idx' with cache coherence.
 * NOTE: latency unit is [ns]
 */

double CmpAnalytPerfModel::CalculateProcLatencyCC (
    UShort idx, bool dynamic)
{
  Processor * proc = cmpConfig.GetProcessor(idx);

  // L1 latency [ns]
  double l1Latency = proc->L1Lat() / proc->Freq();

  // L2 latency [ns]
  double l2Latency = proc->L2Lat() / proc->Freq();

  // L3 latency - L3 hit [ns]
  double l3Latency = 0.0;
  double mainMemLatency = 0.0;
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);
    const double dirLat = mem->Latency()/cmpConfig.UFreq()/3.0; // directory latency (approx.) [ns]

    Component * parent = cmpConfig.GetProcessor(idx)->Parent();
    DASSERT(parent);
    DASSERT(parent->Type() == CTCLUSTER);
    Cluster * cl = static_cast<Cluster*>(parent);

    // find lowest cluster that contains both, proc and mem
    while (!(cl->ProcOwner(idx)) || !(cl->MemOwner(m))) {
      cl = cl->ClParent();
      DASSERT(cl);
    }

    // precalulate latencies between Proc and L3 [ns]
    double latProcToL3Req = cl->ULatProcToMem(idx, m, dynamic, 1, UShort(MSGREQ));
    double latL3ToProcAck = cl->ULatMemToProc(m, idx, dynamic, 1, UShort(MSGACK));
    double latL3ToProcData = cl->ULatMemToProc(
          m, idx, dynamic, cmpConfig.MemReplySize(), UShort(MSGDATA));

    //cout << "latProcToL3Req = " << latProcToL3Req
    //     << ", latL3ToProcAck = " << latL3ToProcAck
    //     << ", latL3ToProcData = " << latL3ToProcData << endl;

    // L3 hit [ns]
    double l3HitLatency = latProcToL3Req + max(
          dirLat + (dynamic ? mem->BufDelay() : 0.0) + latL3ToProcAck,
          mem->Latency()/cmpConfig.UFreq() + (dynamic ? mem->BufDelay() : 0.0) + latL3ToProcData);

    if (idx == 0)
      DEBUG(3, "pa[" << m << "] = " << proc->L3ProbDistr()[m]
               << ", latency = " << l3HitLatency << endl);
    l3Latency += proc->L3ProbDistr()[m]*l3HitLatency;

    // L3 miss (MC latency) [ns]
    const int numValuesInSubn = cmpConfig.MemCnt()*cmpConfig.MemCtrlCnt();
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      // See L3 Miss transaction in 2012-07-05_Memory_Flows_Implementation.pptx
      double l3MissLatency = latProcToL3Req + dirLat + (dynamic ? mem->BufDelay() : 0.0) +
          L3ToMcLat()[UShort(MSGREQ)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] +
          cmpConfig.GetMemCtrl(mc)->latency + (dynamic ? cmpConfig.GetMemCtrl(mc)->bufDelay : 0.0) +
          max(L3ToMcLat()[UShort(MSGACK)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] +
              dirLat + (dynamic ? mem->BufDelay() : 0.0) + latL3ToProcAck,
              L3ToMcLat()[UShort(MSGDATA)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] +
              (dynamic ? mem->BufDelay() : 0.0) + latL3ToProcData);

      /*if (idx == 0)
        DEBUG(3, "prob[" << mc << "] = " << (1.0/cmpConfig.MemCtrlCnt())
               << ", latency = " << LatencyProcToMemCtrl(idx, mc, dynamic) << endl);*/
      mainMemLatency += proc->L3ProbDistr()[m] * l3MissLatency / cmpConfig.MemCtrlCnt();
    }
  }

  const double L1AccProb = proc->L1AccessProbability();
  const double L2AccProb = proc->L2AccessProbability();
  const double L3AccProb = proc->L3AccessProbability();
  const double MMAccProb = proc->MainMemAccessProbability();

  DEBUG(3, "P" << idx << ": p(L2) = " << L2AccProb
         << ", l2Latency = " << l2Latency
         << ", p(L3) = " << L3AccProb
         << ", l3Latency = " << l3Latency
         << ", p(MM) = " << MMAccProb
         << ", MMLatency = " << mainMemLatency << endl);

  return ( ( proc->OoO() ? 0.0 : L1AccProb*l1Latency +
             L2AccProb*l2Latency ) +
          L3AccProb*l3Latency +
          MMAccProb*mainMemLatency);
}

//=======================================================================
/*
 * Returns latency for processor 'pIdx' to access the memory controller
 * with index 'mcIdx'. No CC is considered.
 */

double CmpAnalytPerfModel::LatencyProcToMemCtrl (
    UShort pIdx, UShort mcIdx, bool dynamic)
{
  Component * parent = cmpConfig.GetProcessor(pIdx)->Parent();
  DASSERT(parent);

  // find top component
  while (parent->Parent()) {
    parent = parent->Parent();
  }

  DASSERT(parent->Type() == CTCLUSTER);
  Cluster * cl = static_cast<Cluster*>(parent);

  return cl->FLatProcToMemCtrl(pIdx, mcIdx, dynamic);
}

//=======================================================================
/*
 * Returns latency for processor 'pIdx' to access the memory with index 'mIdx'.
 * No CC is considered.
 */

double CmpAnalytPerfModel::LatencyProcToMem (UShort pIdx, UShort mIdx, bool dynamic)
{
  Component * parent = cmpConfig.GetProcessor(pIdx)->Parent();
  DASSERT(parent);
  DASSERT(parent->Type() == CTCLUSTER);
  Cluster * cl = static_cast<Cluster*>(parent);

  // find lowest cluster that contains both, proc and mem
  while (!(cl->ProcOwner(pIdx)) || !(cl->MemOwner(mIdx))) {
    cl = cl->ClParent();
    DASSERT(cl);
  }

  return cl->FLatProcToMem(pIdx, mIdx, dynamic);
}

//=======================================================================
/*
 * Precalculate latencies from L3s to MCs (only for cache coherence).
 */

void CmpAnalytPerfModel::PrecalcL3ToMcLatencies(bool dynamic)
{
  // Access: l3ToMcLat_[subnIdx*numValuesInSubn + l3Idx*cmpConfig.MemCtrlCnt()+mcIdx];
  const int numValuesInSubn = cmpConfig.MemCnt()*cmpConfig.MemCtrlCnt();
  l3ToMcLat_.resize(UShort(NUMMSG)*numValuesInSubn);

  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Component * parent = cmpConfig.GetMemory(m)->Parent();
    DASSERT(parent);

    // find top component
    while (parent->Parent()) {
      parent = parent->Parent();
    }

    DASSERT(parent->Type() == CTCLUSTER);
    Cluster * cl = static_cast<Cluster*>(parent);

    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      l3ToMcLat_[UShort(MSGREQ)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] =
          cl->ULatMemToMemCtrl (m, mc, dynamic, 1, UShort(MSGREQ));
      l3ToMcLat_[UShort(MSGACK)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] =
          cl->ULatMemCtrlToMem (mc, m, dynamic, 1, UShort(MSGACK));
      l3ToMcLat_[UShort(MSGDATA)*numValuesInSubn+m*cmpConfig.MemCtrlCnt()+mc] =
          cl->ULatMemCtrlToMem (mc, m, dynamic, cmpConfig.MemReplySize(), UShort(MSGDATA));
    }
  }
}

//=======================================================================
/*
 * Print: this is a stub function for CmpAnalytPerfModel.
 */

void CmpAnalytPerfModel::Print () const
{

}

//=======================================================================

  } // namespace perf

} // namespace cmpex
