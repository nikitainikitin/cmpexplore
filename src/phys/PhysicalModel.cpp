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

#include "PhysicalModel.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include "cmp/Processor.hpp"
#include "cmp/Memory.hpp"
#include "cmp/Interconnect.hpp"
#include "cmp/Cluster.hpp"
#include "cmp/MeshIc.hpp"
#include "arch/ArchConfig.hpp"
#include "Util.hpp"
#include "RouterDefs.hpp"

using namespace std;

namespace cmpex {
  
  extern Config config;
  extern cmp::CmpConfig cmpConfig;

  using namespace cmp;
  using namespace arch;

  namespace phys {

//=======================================================================
/*
 * Constructors and destructor
 */

PhysicalModel::PhysicalModel() {}

PhysicalModel::~PhysicalModel() {}

//=======================================================================
/*
 * Create current config from ArchPlanner and calculate its cost.
 */

bool PhysicalModel::IsRoutable(arch::ArchConfig *pAc)
{
  // 1. Calcualte routable cluster area
  double clusterArea = config.MaxArea()/(pAc->GMeshDimX()*pAc->GMeshDimY());

  // Calculate processor area
  double procArea = 0.0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    procArea += pAc->ProcCnt()[p]*config.ProcArea(p);
  }

  double routableArea = clusterArea - procArea;

  // 2. Get routability requirement
  double routerArea = RouterArea(config.Tech(), config.LinkWidth());
  double reqArea = 2.0*sqrt(clusterArea*routerArea) - routerArea;

  // 2a. Local IC
  double wirePitch = 1.0e-4; // 100 nm
  double localIcReqArea = config.LinkWidth()*wirePitch*sqrt(clusterArea);

  reqArea += localIcReqArea;

  //cout << "RoutableArea = " << routableArea << ", ReqArea = " << reqArea << endl;

  return routableArea > reqArea;
}

//=======================================================================
/*
 * Returns the difference between the routable area and
 * routability requirement. Negative value means unroutable design.
 */

double PhysicalModel::GetRoutabilitySlack(arch::ArchConfig *pAc)
{
  return GetRoutableArea(pAc) - GetRoutableAreaReq(pAc);
}

//=======================================================================
/*
 * Calculates the total routable area on chip.
 */

double PhysicalModel::GetRoutableArea(arch::ArchConfig *pAc)
{
  // Calculate processor area per cluster
  double procArea = 0.0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    procArea += pAc->ProcCnt()[p]*config.ProcArea(p);
  }

  double totalProcArea = procArea*pAc->GMeshDimX()*pAc->GMeshDimY();

  return max(0.0, config.MaxArea() - totalProcArea);
}

//=======================================================================
/*
 * Calculates the area required for routability on chip.
 */

double PhysicalModel::GetRoutableAreaReq(arch::ArchConfig *pAc)
{
  double clusterArea = config.MaxArea()/(pAc->GMeshDimX()*pAc->GMeshDimY());
  double routerArea = RouterArea(config.Tech(), config.LinkWidth());
  double reqArea = 2.0*sqrt(clusterArea*routerArea) - routerArea;
  return reqArea*pAc->GMeshDimX()*pAc->GMeshDimY();
}

//=======================================================================
/*
 * Calculates area of a given configuration.
 */

double PhysicalModel::GetCmpArea(cmp::Component * cmp)
{
  double procAreaTotal = 0.0;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    procAreaTotal += proc->Area() +
        (proc->L1Size()+proc->L2Size())*cmpConfig.MemDensity();
  }

  double memAreaTotal = 0.0;
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);

    memAreaTotal += mem->Size()*cmpConfig.MemDensity();
  }

  Cluster * clCmp = static_cast<Cluster*>(cmp);
  if (clCmp->Ic()->Type() != ITMESH) {
    cout << "-E- Area calculation only works for 2-level IC with top mesh" << endl;
    exit(1);
  }

  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  double totalArea = procAreaTotal + memAreaTotal +
      RouterArea(config.Tech(), config.LinkWidth())*mic->TCnt();

  return totalArea;
}

//=======================================================================

  } // namespace phys

} //namespace cmpex
