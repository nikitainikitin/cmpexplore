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

#include "ParamIterator.hpp"
#include "ArchPlanner.hpp"
#include "Config.hpp"
#include "RouterDefs.hpp"

using namespace std;

namespace cmpex {

  extern Config config;

  namespace arch {

//=======================================================================
/*
 * ParamIterator: constructor and destructor
 */

ParamIterator::ParamIterator(ArchPlanner * ap, const string& name) :
  ap_ (ap), name_ (name) {}

ParamIterator::~ParamIterator() {}

//=======================================================================
/*
 * GMeshDimIter: constructor and destructor
 */

GMeshDimIter::GMeshDimIter(ArchPlanner * ap) : ParamIterator(ap, "GMeshDimIter"){}

GMeshDimIter::~GMeshDimIter() {}

//=======================================================================
/*
 * GMeshDimIter initialization.
 */

bool GMeshDimIter::Init()
{
  Ap()->GMeshDimXIter(0);
  Ap()->GMeshDimYIter(0);

  if (config.GMeshDimXVec().empty() | config.GMeshDimYVec().empty())
    return false;
  
  return true;
}

//=======================================================================
/*
 * GMeshDimIter advance.
 */

void GMeshDimIter::Advance()
{
  Ap()->GMeshDimYIter(Ap()->GMeshDimYIter()+1); // advance y
  
  if (Ap()->GMeshDimYIter() == config.GMeshDimYVec().size()) { // advance x
    Ap()->GMeshDimXIter(Ap()->GMeshDimXIter()+1);
    Ap()->GMeshDimYIter(0);
  }

  const double AR = 2.01; // .01 to avoid double division errors

  // ASPECT RATIO HARDCODED!!!
  double x = config.GMeshDimXVec()[Ap()->GMeshDimXIter()];
  double y = config.GMeshDimYVec()[Ap()->GMeshDimYIter()];
  if (int(x) && int(y) && (x/y > AR || y/x > AR || int(y) > int(x)))
    Advance();
}

//=======================================================================
/*
 * GMeshDimIter check for valid value.
 */

bool GMeshDimIter::Valid()
{
  if (Ap()->GMeshDimXIter() >= config.GMeshDimXVec().size())
    return false;

  return true;
}

//=======================================================================
/*
 * ProcSetIter: constructor and destructor
 */

ProcSetIter::ProcSetIter(ArchPlanner * ap) : ParamIterator(ap, "ProcSetIter"),
  minProcArea_ (MAX_DOUBLE), valid_ (true)
{
  for (int p = 0; p < config.ProcCnt(); ++p) {
    // HACK: assume that the first L1 and L2 sizes are the smallest ones,
    double l1Size = config.ProcL1Size(p, 0); // hence [0] idx,
    double l2Size = config.ProcL2Size(p, 0); // the same here
    double procArea = config.ProcArea(p) + (l1Size+l2Size)*config.MemDensity();
    if (minProcArea_ > procArea) {
      minProcArea_ = procArea;
      minProcAreaIdx_ = p;
    }
  }
}

ProcSetIter::~ProcSetIter() {}

//=======================================================================
/*
 * ProcSetIter initialization.
 */

bool ProcSetIter::Init()
{
  Ap()->ProcIter().assign(config.ProcCnt(), 0);
  valid_ = true;

  // a config should include at least one processor:
  // select one with the smallest area
  Ap()->ProcIter()[MinProcAreaIdx()] = 1;

  // return true if at least 1 processor fits the cluster area
  return (MinProcArea() < ProcClusterArea()) ? true : false;
}

//=======================================================================
/*
 * ProcSetIter advance.
 */

void ProcSetIter::Advance()
{
  vector<int>& procIter = Ap()->ProcIter();
  UInt maxProcPerCluster = static_cast<UInt>(ProcClusterArea()/MinProcArea());

  ++procIter[0];
  if (ProcClusterAreaViolated() || procIter[0] > maxProcPerCluster) {
    procIter[0] = 0;
    if (config.ProcCnt() == 1) {
      Invalidate();
    }
    else {
      for (int p = 1; p < config.ProcCnt(); ++p) {
        ++procIter[p];
        if (ProcClusterAreaViolated() || procIter[p] > maxProcPerCluster) {
          procIter[p] = 0;
          if (p == config.ProcCnt()-1) // iterator becomes invalid at last proc
            Invalidate();
        }
        else {
          break;
        }
      }
    }
  }
}

//=======================================================================
/*
 * ProcSetIter check for valid value.
 */

bool ProcSetIter::Valid()
{
  return valid_ && !ProcClusterAreaViolated();
}

//=======================================================================
/*
 * Calculate the cluster area.
 */

double ProcSetIter::ClusterArea() const
{
  UInt clusterCnt = config.GMeshDimXVec()[Ap()->GMeshDimXIter()] *
                    config.GMeshDimYVec()[Ap()->GMeshDimYIter()];
  return config.MaxArea() / clusterCnt;
}

//=======================================================================
/*
 * Calculate the cluster area for processors.
 */

double ProcSetIter::ProcClusterArea() const
{
  return ClusterArea()*config.MaxProcAreaPerCluster() -
      RouterArea(config.Tech(), config.LinkWidth());
}

//=======================================================================
/*
 * Check violation of cluster processor area constraint.
 */

bool ProcSetIter::ProcClusterAreaViolated() const
{
  double area = 0.0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    double l1Size = config.ProcL1Size(p, Ap()->ProcL1SizeIter()[p]);
    double l2Size = config.ProcL2Size(p, Ap()->ProcL2SizeIter()[p]);
    double procArea = config.ProcArea(p) + (l1Size+l2Size)*config.MemDensity();
    area += procArea*Ap()->ProcIter()[p];
  }

  return area > ProcClusterArea();
}

//=======================================================================
/*
 * ProcL1SizeIter: constructor and destructor
 */

ProcL1SizeIter::ProcL1SizeIter(ArchPlanner * ap) : ParamIterator(ap, "ProcL1SizeIter"),
  valid_ (true) {}

ProcL1SizeIter::~ProcL1SizeIter() {}

//=======================================================================
/*
 * ProcL1SizeIter initialization.
 */

bool ProcL1SizeIter::Init()
{
  Ap()->ProcL1SizeIter().assign(config.ProcCnt(), 0);
  valid_ = true;

  return true;
}

//=======================================================================
/*
 * ProcL1SizeIter advance.
 */

void ProcL1SizeIter::Advance()
{
  vector<int>& procL1SizeIter = Ap()->ProcL1SizeIter();

  // it only makes sense to advance iterator for processor of type p
  // if there are processors of this type in cluster
  for (int p = 0; p < config.ProcCnt(); ++p) {
    if (Ap()->ProcIter()[p] == 0) {
      if (p == config.ProcCnt()-1) // iterator becomes invalid at last proc
        Invalidate();
      continue;
    }

    ++procL1SizeIter[p];
    if (procL1SizeIter[p] >= config.ProcL1SizeCnt(p)) {
      procL1SizeIter[p] = 0;
      if (p == config.ProcCnt()-1) // iterator becomes invalid at last proc
        Invalidate();
    }
    else {
      break;
    }
  }
}

//=======================================================================
/*
 * ProcL1SizeIter check for valid value.
 */

bool ProcL1SizeIter::Valid()
{
  return valid_;
}

//=======================================================================
/*
 * ProcL2SizeIter: constructor and destructor
 */

ProcL2SizeIter::ProcL2SizeIter(ArchPlanner * ap) : ParamIterator(ap, "ProcL2SizeIter"),
  valid_ (true) {}

ProcL2SizeIter::~ProcL2SizeIter() {}

//=======================================================================
/*
 * ProcL2SizeIter initialization.
 */

bool ProcL2SizeIter::Init()
{
  Ap()->ProcL2SizeIter().assign(config.ProcCnt(), 0);
  valid_ = true;

  return true;
}

//=======================================================================
/*
 * ProcL2SizeIter advance.
 */

void ProcL2SizeIter::Advance()
{
  vector<int>& procL2SizeIter = Ap()->ProcL2SizeIter();

  // it only makes sense to advance iterator for processor of type p
  // if there are processors of this type in cluster
  for (int p = 0; p < config.ProcCnt(); ++p) {
    if (Ap()->ProcIter()[p] == 0) {
      if (p == config.ProcCnt()-1) // iterator becomes invalid at last proc
        Invalidate();
      continue;
    }

    ++procL2SizeIter[p];
    if (procL2SizeIter[p] >= config.ProcL2SizeCnt(p)) {
      procL2SizeIter[p] = 0;
      if (p == config.ProcCnt()-1) // iterator becomes invalid at last proc
        Invalidate();
    }
    else {
      break;
    }
  }
}

//=======================================================================
/*
 * ProcL2SizeIter check for valid value.
 */

bool ProcL2SizeIter::Valid()
{
  return valid_;
}

//=======================================================================
/*
 * L3SetIter: constructor and destructor
 */

L3SetIter::L3SetIter(ArchPlanner * ap) : ParamIterator(ap, "L3SetIter"){}

L3SetIter::~L3SetIter() {}

//=======================================================================
/*
 * L3SetIter initialization.
 */

bool L3SetIter::Init()
{
  Ap()->NumL3Iter(0);

  return (config.NumL3PerCluster().empty()) ? false : true;
}

//=======================================================================
/*
 * L3SetIter advance.
 */

void L3SetIter::Advance()
{
  Ap()->NumL3Iter((Ap()->NumL3Iter()+1));
}

//=======================================================================
/*
 * L3SetIter check for valid value.
 */

bool L3SetIter::Valid()
{
  if (Ap()->NumL3Iter() >= config.NumL3PerCluster().size())
    return false;

  return true;
}

//=======================================================================
/*
 * ClusterIcType: constructor and destructor
 */

ClusterIcTypeIter::ClusterIcTypeIter(ArchPlanner * ap) : ParamIterator(ap, "ClusterIcTypeIter"){}

ClusterIcTypeIter::~ClusterIcTypeIter() {}

//=======================================================================
/*
 * ClusterIcType initialization.
 */

bool ClusterIcTypeIter::Init()
{
  Ap()->ClusterIcTypeIter(0);

  return (config.ClusterIcType().empty()) ? false : true;
}

//=======================================================================
/*
 * ClusterIcType advance.
 */

void ClusterIcTypeIter::Advance()
{
  Ap()->ClusterIcTypeIter((Ap()->ClusterIcTypeIter()+1));
}

//=======================================================================
/*
 * ClusterIcType check for valid value.
 */

bool ClusterIcTypeIter::Valid()
{
  if (Ap()->ClusterIcTypeIter() >= config.ClusterIcType().size())
    return false;

  return true;
}

//=======================================================================

  } // namespace arch

} //namespace cmpex
