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
#include <vector>
#include <map>

#include "RbEngine.hpp"
#include "Config.hpp"
#include "arch/ArchConfig.hpp"
#include "cmp/CmpConfig.hpp"
#include <Timer.hpp>
#include "arch/ArchPlanner.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "PhysicalModel.hpp"
#include "RouterDefs.hpp"
#include "power/PowerModel.hpp"

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

RbEngine::RbEngine() {}

RbEngine::~RbEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void RbEngine::Explore() const
{
  const bool GRAPH_MODE_PRINT = false;
  const bool P3 = (config.ProcCnt() == 3);
  const double POWER_CONSTR = config.MaxPower();

  // number of local iterations
  double lIterCnt = config.GMeshDimXVec().size() + config.GMeshDimYVec().size();
  for (int ptype = 0; ptype < config.ProcCnt(); ++ptype) {
    lIterCnt += config.ProcL1SizeCnt(ptype);
    lIterCnt += config.ProcL2SizeCnt(ptype);
  }

  ArchPlanner ap;
  double bestCost = 0.0;

  Timer timer;
  timer.Start();

  double no_impr_limit = lIterCnt*lIterCnt*config.SEffort();
  double last_impr = 0;
  int iter = 0;

  do {

    // 1. Setup Ap
    ap.GMeshDimXIter(int(RandUDouble()*config.GMeshDimXVec().size()));
    ap.GMeshDimYIter(int(RandUDouble()*config.GMeshDimYVec().size()));

    // precisely calculate distribution of processors
    double x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
    double y = config.GMeshDimYVec()[ap.GMeshDimYIter()];
    double clusterArea = config.MaxArea()/(x*y);
    int p1 = int(RandUDouble()*int(clusterArea/config.ProcArea(0)));
    double leftArea = clusterArea - p1*config.ProcArea(0);
    int p2 = int(RandUDouble()*int(leftArea/config.ProcArea(1)));

    ap.ProcIter()[0] = p1;
    ap.ProcL1SizeIter()[0] = int(RandUDouble()*config.ProcL1SizeCnt(0));
    ap.ProcL2SizeIter()[0] = int(RandUDouble()*config.ProcL2SizeCnt(0));
    ap.ProcIter()[1] = p2;
    ap.ProcL1SizeIter()[1] = int(RandUDouble()*config.ProcL1SizeCnt(1));
    ap.ProcL2SizeIter()[1] = int(RandUDouble()*config.ProcL2SizeCnt(1));

    if (P3) {
      leftArea = leftArea - p2*config.ProcArea(1);
      int p3 = int(RandUDouble()*int(leftArea/config.ProcArea(2)));
      ap.ProcIter()[2] = p3;
      ap.ProcL1SizeIter()[2] = int(RandUDouble()*config.ProcL1SizeCnt(2));
      ap.ProcL2SizeIter()[2] = int(RandUDouble()*config.ProcL2SizeCnt(2));
    }

    if (P3 && ap.ProcIter()[0] + ap.ProcIter()[1] + ap.ProcIter()[2] == 0 ||
        !P3 && ap.ProcIter()[0] + ap.ProcIter()[1] == 0)
      continue;

    ap.ClusterIcTypeIter(int(RandUDouble()*config.ClusterIcType().size()));

    double AR = 2.01;

    // 2. Estimate cost
    if (x/y < AR && y/x < AR && GetAreaOverhead(ap) <= 0.0) {
      ArchConfig * pAc = ap.GenerateCurrentArchConfig();

      cmpConfig.CreateCmp(*pAc);

      StatConfig * sc = new StatConfig(pAc);

      // *************** HACK: avoid configs with no L3 ********************
      /*if (config.SimulateCC() && !cmpConfig.MemCnt()) {
        //cout << "Skipping 0-memory" << endl;
        delete sc;
        continue;
      }*/
      // *************** HACK: avoid configs with no L3 ********************

      // for every workload
      for (int wlIdx = 0; wlIdx < cmpConfig.GetWlCnt(); ++wlIdx) {
        cmpConfig.SetWlIdx(wlIdx);

        // Performance
        IterativePerfModel m;
        StatMetrics * pSm = m.RunBisectionFp();

        // Power
        double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
        pSm->Power(power);
        sc->AddMetrics(pSm);
      }

      //if (x >= y && sc->AggPower() <= POWER_CONSTR) {
      //  stats.AddConfig(sc);
      //}

      if (bestCost < sc->AggThroughput() && x >= y && sc->AggPower() <= POWER_CONSTR) {
        bestCost = sc->AggThroughput();
        last_impr = iter;
        if (GRAPH_MODE_PRINT) {
          cout << timer.Current() << '\t' << bestCost << '\t' << sc->AggPower() << endl;
        }
        else {
          cout << "(Time " << timer.Current() << ", iter " << iter << ") ";
          cout << "x=" << config.GMeshDimXVec()[ap.GMeshDimXIter()]
               << "; y=" << config.GMeshDimYVec()[ap.GMeshDimYIter()]
               << "; " << ( ap.ClusterIcTypeIter() == 0 ? "bus" :
                              ap.ClusterIcTypeIter() == 1 ? "uring" : "bring" )
               << "; P1=" << ap.ProcIter()[0]
               << "; L1=" << config.ProcL1Size(0, ap.ProcL1SizeIter()[0])
               << "; L2=" << config.ProcL2Size(0, ap.ProcL2SizeIter()[0])
               << "; P2=" << ap.ProcIter()[1]
               << "; L1=" << config.ProcL1Size(1, ap.ProcL1SizeIter()[1])
               << "; L2=" << config.ProcL2Size(1, ap.ProcL2SizeIter()[1]);
          if (config.ProcCnt() == 3)
               cout << "; P3=" << ap.ProcIter()[2]
                    << "; L1=" << config.ProcL1Size(2, ap.ProcL1SizeIter()[2])
                    << "; L2=" << config.ProcL2Size(0, ap.ProcL2SizeIter()[2]);
          cout << ";  Thr=" << bestCost << "; Pow=" << sc->AggPower() << endl;
        }
      }

      cmpConfig.Cleanup();
    }

    ++iter;
  } while (iter - last_impr < no_impr_limit);


  cout << "Finished search (no improvement during the last "
       << no_impr_limit << " iterations)" << endl;

  /*cout << "Pool of configurations:" << endl;
  stats.ReportAllConfigs();
  stats.DumpConfigs();
  system("make -C ./test/");*/
}

//=======================================================================
/*
 * Calculates area overhead for current Ap configuration.
 */

double RbEngine::GetAreaOverhead(ArchPlanner& ap) const
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
  double totalArea = procAreaPerCluster*x*y;
  totalArea += RouterArea(config.Tech(), config.LinkWidth());

  return totalArea - config.MaxArea();
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
