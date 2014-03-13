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
#include <fstream>
#include <iomanip>

#include "ExplEngine.hpp"
#include "ExplConf.hpp"
#include "Transform.hpp"
#include "Config.hpp"
#include "arch/ArchConfig.hpp"
#include "cmp/CmpConfig.hpp"
#include "arch/ArchPlanner.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "phys/PhysicalModel.hpp"
#include "power/PowerModel.hpp"
#include "RouterDefs.hpp"

using namespace std;

namespace cmpex {

  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern stat::Statistics stats;

  using namespace cmp;
  using namespace arch;
  using namespace stat;
  using namespace perf;
  using namespace phys;
  using namespace power;

  namespace explore {

//=======================================================================
/*
 * Constructors and destructor
 */

ExplEngine::ExplEngine() {
  // create transformations
  AddTransform(new TrNextIC());
  AddTransform(new TrPrevIC());
  AddTransform(new TrIncX());
  AddTransform(new TrDecX());
  AddTransform(new TrIncY());
  AddTransform(new TrDecY());
  AddTransform(new TrIncXDecY());
  AddTransform(new TrDecXIncY());

  // P1
  AddTransform(new TrIncP1());
  AddTransform(new TrDecP1());
  AddTransform(new TrIncL11());
  AddTransform(new TrDecL11());
  AddTransform(new TrIncL21());
  AddTransform(new TrDecL21());

  AddTransform(new TrDecP1IncX());
  AddTransform(new TrDecP1IncY());
  AddTransform(new TrIncP1DecX());
  AddTransform(new TrIncP1DecY());

  AddTransform(new TrReclIncP1DecX());
  AddTransform(new TrReclIncP1DecY());
  AddTransform(new TrReclDecP1IncX());
  AddTransform(new TrReclDecP1IncY());

  // P2
  AddTransform(new TrIncP2());
  AddTransform(new TrDecP2());
  AddTransform(new TrIncL12());
  AddTransform(new TrDecL12());
  AddTransform(new TrIncL22());
  AddTransform(new TrDecL22());

  AddTransform(new TrIncP2DecX());
  AddTransform(new TrIncP2DecY());
  AddTransform(new TrDecP2IncX());
  AddTransform(new TrDecP2IncY());

  AddTransform(new TrReclIncP2DecX());
  AddTransform(new TrReclIncP2DecY());
  AddTransform(new TrReclDecP2IncX());
  AddTransform(new TrReclDecP2IncY());

  AddTransform(new TrIncP1DecP2());
  AddTransform(new TrIncP2DecP1());

  // P3
  AddTransform(new TrIncP3());
  AddTransform(new TrDecP3());
  AddTransform(new TrIncL13());
  AddTransform(new TrDecL13());
  AddTransform(new TrIncL23());
  AddTransform(new TrDecL23());

  AddTransform(new TrIncP3DecX());
  AddTransform(new TrIncP3DecY());
  AddTransform(new TrIncXDecP3());
  AddTransform(new TrIncYDecP3());

  AddTransform(new TrReclIncP3DecX());
  AddTransform(new TrReclIncP3DecY());
  AddTransform(new TrReclIncXDecP3());
  AddTransform(new TrReclIncYDecP3());

  AddTransform(new TrIncP3DecP1());
  AddTransform(new TrIncP3DecP2());
  AddTransform(new TrIncP1DecP3());
  AddTransform(new TrIncP2DecP3());
}

ExplEngine::~ExplEngine() {
  for (TrIter it = transforms_.begin(); it != transforms_.end(); ++it)
    delete *it;
}

//=======================================================================
/*
 * Calculates area overhead of the current config represented by ArchPlanner.
 * Returns nonpositive value if there's no overhead.
 */

double ExplEngine::GetAreaOverhead(arch::ArchPlanner& ap) const
{
  UInt x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
  UInt y = config.GMeshDimYVec()[ap.GMeshDimYIter()];

  double procAreaPerCluster = 0.0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    double l1Size = config.ProcL1Size(p, ap.ProcL1SizeIter()[p]);
    double l2Size = config.ProcL2Size(p, ap.ProcL2SizeIter()[p]);
    double procArea = config.ProcArea(p) +(l1Size+l2Size)*config.MemDensity();
    procAreaPerCluster += procArea*ap.ProcIter()[p];
  }
  double totalArea = procAreaPerCluster + RouterArea(config.Tech(), config.LinkWidth());
  //cout << "EE: ProcArea/cluster = " << procAreaPerCluster << endl;
  //cout << "EE: RouterArea = " << RouterArea(config.Tech(), config.LinkWidth()) << endl;
  totalArea *= x*y;

  return totalArea - config.MaxArea();
}

//=======================================================================
/*
 * Calculates area overhead of the current config represented by ArchPlanner.
 * Returns nonpositive value if there's no overhead.
 */

ExplConf* ExplEngine::GetCurConfigCost(arch::ArchPlanner& ap, double lambda) const
{
  ArchConfig * pAc = ap.GenerateCurrentArchConfig();

  /*ofstream out(string("debug.cmp").c_str());
  out << pAc->ToString();
  out.close();*/

  cmpConfig.CreateCmp(*pAc);

  StatConfig * sc = new StatConfig(pAc);

  // *************** HACK: avoid configs with no L3 ********************
  if (config.SimulateCC() && !cmpConfig.MemCnt()) {
    //cout << "Skipping 0-memory" << endl;
    ExplConf * conf = new ExplConf(ap, config.MaxArea(), 0.1, 10000, 0.001);
    cmpConfig.Cleanup();
    return conf;
  }
  // *************** HACK: avoid configs with no L3 ********************

  double AR = 2.01;
  double x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
  double y = config.GMeshDimYVec()[ap.GMeshDimYIter()];
  double arPen = max(x/y, y/x);
  arPen = max(0.0, arPen-AR);

  double areaPen = max(0.0, GetAreaOverhead(ap));
  double powerPen = 0.0;

  // for every workload
  for (int wlIdx = 0; wlIdx < cmpConfig.GetWlCnt(); ++wlIdx) {
    cmpConfig.SetWlIdx(wlIdx);

    // Performance
    IterativePerfModel m;
    StatMetrics * pSm = m.Run();

    // Power
    double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
    pSm->Power(power);
    powerPen += max(0.0, power - config.MaxPower());

    sc->AddMetrics(pSm);
  }

  // use this for only constraining aggregate power, not max power per workload
  // (overwrites powerPen calculated above)
  //powerPen = max(0.0, sc->AggPower() - config.MaxPower());

  // cost including penalty
  double cost = sc->AggThroughput() / (1.0 + lambda*areaPen/config.MaxArea()) /
      (1.0 + lambda*arPen/AR) / (1.0 + lambda*powerPen/config.MaxPower());

  ExplConf * conf = new ExplConf(ap, pAc->Area(), sc->AggThroughput(), sc->AggPower(), cost);

  if (areaPen <= E_DOUBLE && powerPen <= E_DOUBLE && arPen <= E_DOUBLE && x >= y) {
    stats.AddConfig(sc);
  }
  else {
    delete sc;
  }

  cmpConfig.Cleanup();

  return conf;
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
