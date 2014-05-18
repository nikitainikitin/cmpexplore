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
#include <limits>
#include <vector>

#include "CmpConfig.hpp"
#include "Defs.hpp"
#include "Memory.hpp"
#include "Processor.hpp"
#include "Cluster.hpp"
#include "CmpBuilder.hpp"
#include "Component.hpp"
#include "arch/ArchConfig.hpp"
#include "MeshIc.hpp"

using std::cout;
using std::endl;
using std::vector;
using std::string;

namespace cmpex {

  using arch::ArchConfig;

  namespace cmp {

//=======================================================================
/*
 * Constructors and destructor
 */

CmpConfig::CmpConfig ( void ) :
  cmp_ (0), unitLen_ (1.0e-3), memDensity_ (10),
  uFreq_ (1.6), uFreqMax_ (3.5), uVolt_(0.8), uVoltMax_(1.0), uVoltMin_(0.5), uVoltNom_(1.0),
  mcFreq_ (0.8), mcFreqMax_ (0.8), mcVolt_(0.8), mcVoltMax_(1.0), mcVoltMin_(0.5), mcVoltNom_(1.0),
  procFreqMax_ (3.5), procVoltMax_ (1.0), procVoltMin_(0.5), procVoltNom_(1.0),
  memReplySize_(3),
  niDelay_ (0), L3ClusterSize_(0), totalL3Size_ (0.0), subnCnt_ (1),
  wlFile_(""), flatMeshIc_ (true), wlIdx_(-1) {}

CmpConfig::~CmpConfig () {
  if (cmp_) delete cmp_;

  for (int i = 0; i < workloads_.size(); ++i)
    delete workloads_[i];
}

//=======================================================================
/*
 * Calculate total available size of L3 memory
 */

void CmpConfig::CalcTotalL3Size ()
{
  double sz = 0.0;
  
  for (int m = 0; m < MemCnt(); ++m) {
    if (GetMemory(m)->MemType() == MTL3) {
      sz += GetMemory(m)->Size();
    }
  }
  
  //if (sz < EMIN_DOUBLE)
  //  cout << "-W-: L3 memory size = " << sz << "!" << endl;

  totalL3Size_ = sz;
}

//=======================================================================
/*
 * Creates cmp from the specified file.
 */

int CmpConfig::CreateCmp (const std::string& fname)
{
  Component * cmp = CmpBuilder::ReadFromFile(fname);
  DASSERT(cmp);
  SetCmp(cmp);
  InitComponentOwnership();
  //InitL3ToMcMapping();
  InitRuntimeProperties();
  InitProcL3ProbDistr();

  return 0;
}

//=======================================================================
/*
 * Creates cmp from the specified config.
 */

int CmpConfig::CreateCmp (ArchConfig& ac)
{
  Component * cmp = CmpBuilder::ReadFromArchConfig(ac);
  DASSERT(cmp);
  SetCmp(cmp);
  InitComponentOwnership();
  //InitL3ToMcMapping();
  InitRuntimeProperties();
  InitProcL3ProbDistr();

  return 0;
}

//=======================================================================
/*
 * Converts architectural file to simulator-compatible
 * workload + architecture files
 */

void CmpConfig::WriteWorkloadArchFromArch (const std::string& fname)
{
  CmpBuilder::WriteWorkloadArchFromArch(fname);
}

//=======================================================================
/*
 * The procedure sets ownership for composite components.
 */

void CmpConfig::InitComponentOwnership ()
{
  for (int c = 0; c < ClusterCnt(); ++c) {
    GetCluster(c)->InitOwnerships();
  }
  
  // !!! It is assumed that all parents of Devices are Clusters !!!
  
  for (int p = 0; p < ProcCnt(); ++p) {
    Component * current = GetProcessor(p);
    while (current->Parent()) {
      Cluster * parent = static_cast<Cluster*>(current->Parent());
      parent->SetProcOwnership(p, current);
      current = parent;
    }
  }

  for (int m = 0; m < MemCnt(); ++m) {
    Component * current = GetMemory(m);
    while (current->Parent()) {
      Cluster * parent = static_cast<Cluster*>(current->Parent());
      parent->SetMemOwnership(m, current);
      current = parent;
    }
  }
}

//=======================================================================
/*
 * The procedure sets mapping from L3 to MC.
 * Every L3 is associated to the closest MC
 * (selecting randomly among those having the same distance).
 *
 * !! NOTE: Assume two-level hierarchical Cmp with top-level mesh
 * and 4 preplaced MCs at the periphery.
 */

void CmpConfig::InitL3ToMcMapping()
{
  Cluster * clCmp = static_cast<Cluster*>(Cmp());
  MeshIc * mesh = static_cast<MeshIc*>(clCmp->Ic());

  for (int m = 0; m < MemCnt(); ++m) {
    Memory * mem = GetMemory(m);

    int memTileColIdx = mesh->GetTile(clCmp->MemOwner(m)->ClIdx())->ColIdx();
    int memTileRowIdx = mesh->GetTile(clCmp->MemOwner(m)->ClIdx())->RowIdx();

    int distToNorth = memTileRowIdx + 1;
    int distToWest = memTileColIdx + 1;
    int distToSouth = mesh->RowNum()-memTileRowIdx;
    int distToEast = mesh->ColNum()-memTileColIdx;

    // list of closest MCs
    vector<string> closestMcs;

    int dist = distToNorth;
    closestMcs.push_back("North");

    if (distToWest < dist) {
      dist = distToWest;
      closestMcs.clear();
      closestMcs.push_back("West");
    }
    else if (distToWest == dist) {
      closestMcs.push_back("West");
    }

    if (distToSouth < dist) {
      dist = distToSouth;
      closestMcs.clear();
      closestMcs.push_back("South");
    }
    else if (distToSouth == dist) {
      closestMcs.push_back("South");
    }

    if (distToEast < dist) {
      dist = distToEast;
      closestMcs.clear();
      closestMcs.push_back("East");
    }
    else if (distToEast == dist) {
      closestMcs.push_back("East");
    }

    // randomly select MC among the closest ones
    string mcName = closestMcs[RandInt(closestMcs.size())];

    for (int mc = 0; mc < MemCtrlCnt(); ++mc) {
      if (GetMemCtrl(mc)->name == mcName) {
        mem->McIdx(mc);
        break;
      }
    }
  }
}

//=======================================================================
/*
 * Initialize global runtime properties for CmpConfig.
 */

void CmpConfig::InitRuntimeProperties()
{
  // 1. --------------- Init flatMeshIc property -----------------
  Cluster * clCmp = static_cast<Cluster*>(Cmp());

  bool flatMeshIc = true;
  for (int c = 0; c < clCmp->CompCnt(); ++c) {
    if (clCmp->GetComponent(c)->Type() == CTCLUSTER)
      flatMeshIc = false;
    break;
  }

  FlatMeshIc(flatMeshIc);
}

//=======================================================================
/*
 * Initializes probabilities of processors to access instances of
 * distributed L3 caches, once the config is built.
 */

void CmpConfig::InitProcL3ProbDistr()
{
  if (!L3ClusterSize()) {
    // This is the default scenario when every core can access ALL L3 slices on chip.
    // The code below calculates the probability of accessing every L3 slice
    // depending on the distance to this slice.
    for (int p = 0; p < ProcCnt(); ++p) {
      Processor * proc = GetProcessor(p);
      Cluster * cl = static_cast<Cluster*>(proc->Parent());

      proc->L3ProbDistr().reserve(MemCnt());

      double sum = 0.0;
      for (int m = 0; m < MemCnt(); ++m) {
        double dist = cl->DistanceProcToMem(p, m);
        proc->L3ProbDistr()[m] = GetMemory(m)->Size() / (dist*dist*dist);
        sum += proc->L3ProbDistr()[m];
      }

      // normalize
      for (int m = 0; m < MemCnt(); ++m) {
        proc->L3ProbDistr()[m] /= sum;
      }
    }
  }
  else {
    // In this scenario we assume that every core can only access L3 slices
    // within a predefined region, called an L3 cluster.
    // The concept of L3 cluster is different from cmp::Cluster.
    // The latter is essentially one physical tile, while L3 cluster
    // is a logical a group of neighboring tiles sharing the L3 slices.
    // E.g. for a 64-tile CMP and L3ClusterSize=4, the chip will be
    // logically split into 64/4 = 16 L3 clusters of size 2x2 and
    // every core will only access L3 slices within the cluster.
    // Uniform probability within the cluster is assumed.

    // NOTE: the code below has been designed and tested assuming a mesh of tiles
    // architecture, so that every tile contains one core and one L3 slice.
    // For simplicity of implementation,
    // in this mode only square clusters are supported for now.
    // The allowed sizes are: 1, 4, 16 and 64.

    // sanity checks
    Cluster * clCmp = static_cast<Cluster*>(Cmp());
    MeshIc * mesh = static_cast<MeshIc*>(clCmp->Ic());
    if (mesh->TCnt() < L3ClusterSize()) {
      cout << "-E- L3ClusterSize can not exceed the mesh dimensions. Exiting..." << endl;
      exit(1);
    }
    else if (!(L3ClusterSize() == 1 || L3ClusterSize() == 4 ||
               L3ClusterSize() == 16 || L3ClusterSize() == 64)) {
      cout << "-E- L3ClusterSize can only take values of 1, 4, 16 or 64. Exiting..." << endl;
      exit(1);
    }

    double prob = 1.0/L3ClusterSize();
    int l3ClusterWidth = int(sqrt(L3ClusterSize())+E_DOUBLE);
    //cout << "prob = " << prob << endl;
    //cout << "l3CLusterwidth = " << l3ClusterWidth << endl;

    // calculate access probabilities for every processor
    for (int p = 0; p < ProcCnt(); ++p) {
      //cout << "-------> Proc = " << p << endl;
      Processor * proc = GetProcessor(p);
      Cluster * cl = static_cast<Cluster*>(proc->Parent());

      proc->L3ProbDistr().assign(MemCnt(), 0.0);

      // L3 cluster coordinates for this processor
      int minCol = p%mesh->ColNum() - (p%mesh->ColNum())%l3ClusterWidth;
      int maxCol = minCol + l3ClusterWidth - 1;
      int minRow = p/mesh->ColNum() - (p/mesh->ColNum())%l3ClusterWidth;
      int maxRow = minRow + l3ClusterWidth - 1;
      /*cout << "minCol = " << minCol << endl;
      cout << "maxCol = " << maxCol << endl;
      cout << "minRow = " << minRow << endl;
      cout << "maxRow = " << maxRow << endl;*/

      for (int m = 0; m < MemCnt(); ++m) {
        int memCol = m%mesh->ColNum();
        int memRow = m/mesh->ColNum();
        if (memCol >= minCol && memCol <= maxCol && memRow >= minRow && memRow <= maxRow) {
          //cout << "mem " << m << " is active" << endl;
          proc->L3ProbDistr()[m] = prob;
        }
      }
    }
  }

}

//=======================================================================
/*
 * Perform cleanup of CmpConfig containers.
 */

void CmpConfig::Cleanup()
{
  memCtrl_.clear();
  processors_.clear();
  memories_.clear();
  clusters_.clear();
  if (cmp_) {
    delete cmp_;
    SetCmp(0);
  }

  // Note that workloads are not cleaned here for speedup purpose:
  // if next cmp has the same wlFile, then workloads will be kept.
  // See CmpBuilder::ReadParameter().
}

//=======================================================================

  } // namespace cmp

} //namespace cmpex
