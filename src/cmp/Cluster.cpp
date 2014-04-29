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

#include "Cluster.hpp"
#include "CmpConfig.hpp"
#include "Interconnect.hpp"
#include "Memory.hpp"

#include <iostream>

using std::cout;
using std::endl;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  
  namespace cmp {

//=======================================================================
/*
 * Constructors and destructor
 */

Cluster::Cluster ( UShort idx, UShort clIdx, Component * parent ) :
  Component (idx, clIdx, CTCLUSTER, parent) {}

Cluster::~Cluster ()
{
  for (CCIter it = components_.begin(); it != components_.end(); ++it) {
    delete *it;
  }
  delete ic_;
}

//=======================================================================
/*
 * Returns true if component contains the processor 'idx'.
 */

bool Cluster::HasProcessor (UShort idx)
{
  return (ProcOwner(idx)) ? true : false;
}

//=======================================================================
/*
 * Returns true if component contains the memory 'idx'.
 */

bool Cluster::HasMemory (UShort idx)
{
  return (MemOwner(idx)) ? true : false;
}

//=======================================================================
/*
 * Returns hop-count distance from processor 'pIdx' to the Iface
 * component.
 */

int Cluster::DistanceProcToIface (UShort pIdx)
{
  return Ic()->DistanceCompToIface(ProcOwner(pIdx)->ClIdx());
}

//=======================================================================
/*
 * Returns hop-count distance from memory 'mIdx' to the Iface
 * component.
 */

int Cluster::DistanceMemToIface (UShort mIdx)
{
  return Ic()->DistanceCompToIface(MemOwner(mIdx)->ClIdx());
}

//=======================================================================
/*
 * Returns uni-directional latency from the processor 'pIdx'
 * to the interface of cluster.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatProcToIface (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx)) // if processor is not in cluster
    return MAX_DOUBLE;
  
  DASSERT(Parent()); // TODO: for top-component there's no Iface
  
  return ProcOwner(pIdx)->ULatProcToIface(pIdx, dynamic, pSize, subnIdx) +
         cmpConfig.NiDelayNs() +                 // NI delay
         Ic()->LatencyCompToIface(ProcOwner(pIdx)->ClIdx(), pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the interface of cluster
 * to the processor 'pIdx'.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatIfaceToProc (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx)) // if processor is not in cluster
    return MAX_DOUBLE;
  
  DASSERT(Parent()); // TODO: for top-component there's no Iface
  
  return Ic()->LatencyIfaceToComp(ProcOwner(pIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         cmpConfig.NiDelayNs() +                 // NI delay
         ProcOwner(pIdx)->ULatIfaceToProc(pIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory 'mIdx'
 * to the interface of component.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatMemToIface (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!MemOwner(mIdx)) // if memory is not in cluster
    return MAX_DOUBLE;
  
  DASSERT(Parent()); // TODO: for top-component there's no Iface
  
  return MemOwner(mIdx)->ULatMemToIface(mIdx, dynamic, pSize, subnIdx) +
         cmpConfig.NiDelayNs() +                 // NI delay
         Ic()->LatencyCompToIface(MemOwner(mIdx)->ClIdx(), pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the interface of component
 * to the memory 'mIdx'.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatIfaceToMem (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!MemOwner(mIdx)) // if memory is not in cluster
    return MAX_DOUBLE;
  
  DASSERT(Parent()); // TODO: for top-component there's no Iface
  
  return Ic()->LatencyIfaceToComp(MemOwner(mIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         cmpConfig.NiDelayNs() +                 // NI delay
         MemOwner(mIdx)->ULatIfaceToMem(mIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the processor 'pIdx' to access
 * the memory with index 'mIdx', if both are in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatProcToMem (UShort pIdx, UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx) || !MemOwner(mIdx)) // if not in cluster
    return MAX_DOUBLE;
  
  return ProcOwner(pIdx)->ULatProcToIface(pIdx, dynamic, pSize, subnIdx) +
         Ic()->LatencyCompToComp(
             ProcOwner(pIdx)->ClIdx(), MemOwner(mIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         MemOwner(mIdx)->ULatIfaceToMem(mIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory 'mIdx' to access
 * the processor with index 'pIdx', if both are in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatMemToProc (UShort mIdx, UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx) || !MemOwner(mIdx)) // if not in cluster
    return MAX_DOUBLE;

  return MemOwner(mIdx)->ULatMemToIface(mIdx, dynamic, pSize, subnIdx) +
         Ic()->LatencyCompToComp(
             MemOwner(mIdx)->ClIdx(), ProcOwner(pIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         ProcOwner(pIdx)->ULatIfaceToProc(pIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Returns full latency from the processor 'pIdx' to access
 * the memory with index 'mIdx', if both are in cluster;
 * otw returns MAX_DOUBLE.
 * Full latency is the sum of request, access and reply latencies.
 * !! NOTE: This function only works when cache coherence is not considered.
 * NOTE: latency unit is [ns]
 */

double Cluster::FLatProcToMem (UShort pIdx, UShort mIdx, bool dynamic)
{
  if (!ProcOwner(pIdx) || !MemOwner(mIdx)) // if not in cluster
    return MAX_DOUBLE;

  return ULatProcToMem(pIdx, mIdx, dynamic, 1, UShort(MSGNOCC)) +    // request
         cmpConfig.GetMemory(mIdx)->Latency() / cmpConfig.UFreq() +  // access (static)
         ULatMemToProc(mIdx, pIdx, dynamic, cmpConfig.MemReplySize(), UShort(MSGNOCC)) + // reply
         (dynamic ? cmpConfig.GetMemory(mIdx)->BufDelay() : 0.0); // access (dynamic)
}

//=======================================================================
/*
 * Returns uni-directional latency from the processor 'pIdx' to access
 * the memory controller with index 'mcIdx', if processor is in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatProcToMemCtrl (UShort pIdx, UShort mcIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx)) // if not in cluster
    return MAX_DOUBLE;
  
  if (Parent()) // if not top cluster
    return ClParent()->ULatProcToMemCtrl(pIdx, mcIdx, dynamic, pSize, subnIdx);
  
  return ProcOwner(pIdx)->ULatProcToIface(pIdx, dynamic, pSize, subnIdx) +
         Ic()->LatencyCompToMemCtrl(ProcOwner(pIdx)->ClIdx(), mcIdx, pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory controller 'mcIdx' to access
 * the processor with index 'pIdx', if processor is in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatMemCtrlToProc (UShort mcIdx, UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!ProcOwner(pIdx)) // if not in cluster
    return MAX_DOUBLE;
  
  if (Parent()) // if not top cluster
    return ClParent()->ULatMemCtrlToProc(mcIdx, pIdx, dynamic, pSize, subnIdx);
  
  return Ic()->LatencyMemCtrlToComp(mcIdx, ProcOwner(pIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         ProcOwner(pIdx)->ULatIfaceToProc(pIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Returns full latency from the processor 'pIdx' to access
 * the memory controller with index 'mcIdx', if processor is in cluster;
 * otw returns MAX_DOUBLE.
 * Full latency is the sum of request, access and reply latencies.
 * !! NOTE: This function only works when cache coherence is not considered.
 * NOTE: latency unit is [ns]
 */

double Cluster::FLatProcToMemCtrl (UShort pIdx, UShort mcIdx, bool dynamic)
{
  if (!ProcOwner(pIdx)) // if not in cluster
    return MAX_DOUBLE;
  
  if (Parent()) // if not top cluster
    return ClParent()->FLatProcToMemCtrl(pIdx, mcIdx, dynamic);
  
  const CmpConfig::MemCtrl * mc = cmpConfig.GetMemCtrl(mcIdx);
  
  return ULatProcToMemCtrl(pIdx, mcIdx, dynamic, 1, UShort(MSGNOCC)) +    // request
         mc->latency +                                // access (static)
         (dynamic ? mc->bufDelay : 0.0) + // access (dynamic)
         ULatMemCtrlToProc(mcIdx, pIdx, dynamic, cmpConfig.MemReplySize(), UShort(MSGNOCC)); // reply
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory 'mIdx' to access
 * the memory controller with index 'mcIdx', if memory is in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatMemToMemCtrl (UShort mIdx, UShort mcIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!MemOwner(mIdx)) // if not in cluster
    return MAX_DOUBLE;

  if (Parent()) // if not top cluster
    return ClParent()->ULatMemToMemCtrl(mIdx, mcIdx, dynamic, pSize, subnIdx);

  return MemOwner(mIdx)->ULatMemToIface(mIdx, dynamic, pSize, subnIdx) +
         Ic()->LatencyCompToMemCtrl(MemOwner(mIdx)->ClIdx(), mcIdx, pSize, dynamic, subnIdx);
}

//=======================================================================
/*
 * Returns uni-directional latency from the memory controller 'mcIdx' to access
 * the memory with index 'mIdx', if memory is in cluster;
 * otw returns MAX_DOUBLE.
 * NOTE: latency unit is [ns]
 */

double Cluster::ULatMemCtrlToMem (UShort mcIdx, UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx)
{
  if (!MemOwner(mIdx)) // if not in cluster
    return MAX_DOUBLE;

  if (Parent()) // if not top cluster
    return ClParent()->ULatMemCtrlToMem(mcIdx, mIdx, dynamic, pSize, subnIdx);

  return Ic()->LatencyMemCtrlToComp(mcIdx, MemOwner(mIdx)->ClIdx(), pSize, dynamic, subnIdx) +
         MemOwner(mIdx)->ULatIfaceToMem(mIdx, dynamic, pSize, subnIdx);
}

//=======================================================================
/*
 * Initializes ownership arrays using configuration data.
 */

void Cluster::InitOwnerships ()
{
  procOwners_.assign(cmpConfig.ProcCnt(), static_cast<Component*>(0x0));
  memOwners_.assign(cmpConfig.MemCnt(), static_cast<Component*>(0x0));
}

//=======================================================================
/*
 * Returns hop-count distance from processor 'pIdx' to memory 'mIdx'.
 * Assure that cluster contains both, p and m.
 * TODO: add special method.
 */

int Cluster::DistanceProcToMem (UShort pIdx, UShort mIdx)
{
  if (!ProcOwner(pIdx) || !MemOwner(mIdx))
    return ClParent()->DistanceProcToMem(pIdx, mIdx);
  
  return ProcOwner(pIdx)->DistanceProcToIface(pIdx) +
         Ic()->DistanceCompToComp(
             ProcOwner(pIdx)->ClIdx(), MemOwner(mIdx)->ClIdx()) +
         MemOwner(mIdx)->DistanceMemToIface(mIdx);
}

//=======================================================================
/*
 * Print method.
 */

void Cluster::Print () const
{
  cout << "Printing cluster " << Idx() << endl;
  
  cout << "ProcOwnership vector:";
  for (int i = 0; i < cmpConfig.ProcCnt(); ++i) {
    cout << ' ' << ProcOwner(i);
  }
  cout << endl;
  
  cout << "MemOwnership vector:";
  for (int i = 0; i < cmpConfig.MemCnt(); ++i) {
    cout << ' ' << MemOwner(i);
  }
  cout << endl;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
