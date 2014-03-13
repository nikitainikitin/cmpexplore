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
#include <ctime>

#include "HcEngine.hpp"
#include "Config.hpp"
#include "arch/ArchConfig.hpp"
#include "cmp/CmpConfig.hpp"
#include <Timer.hpp>
#include "arch/ArchPlanner.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"

using namespace std;

namespace cmpex {
  
  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern stat::Statistics stats;

  using namespace cmp;
  using namespace arch;
  using namespace stat;
  using namespace perf;

  namespace explore {

//=======================================================================
/*
 * Constructors and destructor
 */

HcEngine::HcEngine() {}

HcEngine::~HcEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void HcEngine::Explore() const
{
  ArchPlanner ap;
  ArchConfig *pAc = 0, *bestConfig = 0, *globalConfig = 0;
  double curCost, bestCost, globalCost;

  Timer timer;
  timer.Start();

  int maxProcPerCluster = int(floor(config.MaxArea()/config.ProcArea(0)));

  double x, y;
  double AR = 2.01;

  for (int it = 0; it < 1000000; ++it) { // random starts

    // 1. Initialization
    ap.GMeshDimXIter(int(RandUDouble()*config.GMeshDimXVec().size()));
    ap.GMeshDimYIter(int(RandUDouble()*config.GMeshDimYVec().size()));
    ap.ProcIter()[0] = int(RandUDouble()*5)+1;
    ap.ProcL1SizeIter()[0] = int(RandUDouble()*config.ProcL1SizeCnt(0));
    ap.ProcL2SizeIter()[0] = int(RandUDouble()*config.ProcL2SizeCnt(0));
    ap.ProcIter()[1] = int(RandUDouble()*5)+1;
    ap.ProcL1SizeIter()[1] = int(RandUDouble()*config.ProcL1SizeCnt(1));
    ap.ProcL2SizeIter()[1] = int(RandUDouble()*config.ProcL2SizeCnt(1));

    pAc = ap.GenerateCurrentArchConfig();

    x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
    y = config.GMeshDimYVec()[ap.GMeshDimYIter()];

    if (GetAreaOverhead(ap) > 0 || x/y > AR || y/x > AR || int(y) > int(x)) {
      delete pAc;
      continue;
    }

    cout << "================= Random start " << it << " ==================" << endl;

    cmpConfig.CreateCmp(*pAc);
    IterativePerfModel m;
    StatMetrics * pSm = m.RunBisectionFp();
    curCost = bestCost = pSm->Throughput();
    delete pSm;
    cout << "Initial Thr = " << curCost << endl;
    cmpConfig.Cleanup();
    bestConfig = pAc;
    if (!globalConfig) {
      globalCost = bestCost;
      globalConfig = bestConfig;
    }
    if (bestCost > globalCost) {
      globalCost = bestCost;
      globalConfig = bestConfig;
    }

    // 2. Main HC loop

    int last_impr_iter = 0;

    for (int iter = 0; iter < 1000000; ++iter) { // outer cooling loop

        if (iter - last_impr_iter > 200) break;

        // 2.a. Generate new solution
        // find parameter to be adjusted: 0-x, 1-y,
        //                                2-#cores[0], 3-L1[0], 4-L2[0]
        //                                5-#cores[0], 6-L1[0], 7-L2[0]
        int param = int(8*RandUDouble());
        // parameter previous value
        int prev;

        switch(param) {
        case 0: // adjust x
          prev = ap.GMeshDimXIter();
          do {
            if (ap.GMeshDimXIter() == 0) {
              ap.GMeshDimXIter(1);
            }
            else if (ap.GMeshDimXIter() == config.GMeshDimXVec().size()-1) {
              ap.GMeshDimXIter(config.GMeshDimXVec().size()-2);
            }
            else {
              int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
              ap.GMeshDimXIter(ap.GMeshDimXIter()+incr);
            }
            x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
            y = config.GMeshDimYVec()[ap.GMeshDimYIter()];
          } while (x/y > AR || y/x > AR || int(y) > int(x));
          break;
        case 1: // adjust y
          prev = ap.GMeshDimYIter();
          do {
            if (ap.GMeshDimYIter() == 0) {
              ap.GMeshDimYIter(1);
            }
            else if (ap.GMeshDimYIter() == config.GMeshDimYVec().size()-1) {
              ap.GMeshDimYIter(config.GMeshDimYVec().size()-2);
            }
            else {
              int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
              ap.GMeshDimYIter(ap.GMeshDimYIter()+incr);
            }
            x = config.GMeshDimXVec()[ap.GMeshDimXIter()];
            y = config.GMeshDimYVec()[ap.GMeshDimYIter()];
          } while (x/y > AR || y/x > AR || int(y) > int(x));
          break;
        case 2: // adjust # proc per cluster
          prev = ap.ProcIter()[0];
          if (ap.ProcIter()[0] == 1) {
            ap.ProcIter()[0] = 2;
          }
          else if (ap.ProcIter()[0] == maxProcPerCluster) {
            ap.ProcIter()[0] = maxProcPerCluster-1;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcIter()[0] = ap.ProcIter()[0]+incr;
          }
          break;
        case 3: // adjust L1 size
          prev = ap.ProcL1SizeIter()[0];
          if (ap.ProcL1SizeIter()[0] == 0) {
            ap.ProcL1SizeIter()[0] = 1;
          }
          else if (ap.ProcL1SizeIter()[0] == config.ProcL1SizeCnt(0)-1) {
            ap.ProcL1SizeIter()[0] = config.ProcL1SizeCnt(0)-2;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcL1SizeIter()[0] = ap.ProcL1SizeIter()[0]+incr;
          }
          break;
        case 4: // adjust L2 size
          prev = ap.ProcL2SizeIter()[0];
          if (ap.ProcL2SizeIter()[0] == 0) {
            ap.ProcL2SizeIter()[0] = 1;
          }
          else if (ap.ProcL2SizeIter()[0] == config.ProcL2SizeCnt(0)-1) {
            ap.ProcL2SizeIter()[0] = config.ProcL2SizeCnt(0)-2;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcL2SizeIter()[0] = ap.ProcL2SizeIter()[0]+incr;
          }
          break;
        case 5: // adjust # proc per cluster
          prev = ap.ProcIter()[1];
          if (ap.ProcIter()[1] == 0) {
            ap.ProcIter()[1] = 1;
          }
          else if (ap.ProcIter()[1] == maxProcPerCluster) {
            ap.ProcIter()[1] = maxProcPerCluster-1;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcIter()[1] = ap.ProcIter()[1]+incr;
          }
          break;
        case 6: // adjust L1 size
          prev = ap.ProcL1SizeIter()[1];
          if (ap.ProcL1SizeIter()[1] == 0) {
            ap.ProcL1SizeIter()[1] = 1;
          }
          else if (ap.ProcL1SizeIter()[1] == config.ProcL1SizeCnt(1)-1) {
            ap.ProcL1SizeIter()[1] = config.ProcL1SizeCnt(1)-2;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcL1SizeIter()[1] = ap.ProcL1SizeIter()[1]+incr;
          }
          break;
        case 7: // adjust L2 size
          prev = ap.ProcL2SizeIter()[1];
          if (ap.ProcL2SizeIter()[1] == 0) {
            ap.ProcL2SizeIter()[1] = 1;
          }
          else if (ap.ProcL2SizeIter()[1] == config.ProcL2SizeCnt(1)-1) {
            ap.ProcL2SizeIter()[1] = config.ProcL2SizeCnt(1)-2;
          }
          else {
            int incr = 2*int(2*RandUDouble())-1; // returns either -1 or +1
            ap.ProcL2SizeIter()[1] = ap.ProcL2SizeIter()[1]+incr;
          }
          break;
        }

        // 2b. Estimate cost of new solution
        pAc = ap.GenerateCurrentArchConfig();
        cmpConfig.CreateCmp(*pAc);
        IterativePerfModel m;
        StatMetrics * pSm = m.RunBisectionFp();
        double curCost = pSm->Throughput();
        if (curCost > bestCost && GetAreaOverhead(ap) <= 0) {
          delete bestConfig;
          bestCost = curCost;
          bestConfig = pAc;
          last_impr_iter = iter;
        }
        else { // reject, restore old solution
          switch(param) {
          case 0: ap.GMeshDimXIter(prev); break;
          case 1: ap.GMeshDimYIter(prev); break;
          case 2: ap.ProcIter()[0] = prev; break;
          case 3: ap.ProcL1SizeIter()[0] = prev; break;
          case 4: ap.ProcL2SizeIter()[0] = prev; break;
          case 5: ap.ProcIter()[1] = prev; break;
          case 6: ap.ProcL1SizeIter()[1] = prev; break;
          case 7: ap.ProcL2SizeIter()[1] = prev; break;
          }
          delete pAc;
        }

        delete pSm;
        cmpConfig.Cleanup();

        // 2d. update best cost
        if (bestCost > globalCost) {
          globalCost = bestCost;
          //delete globalConfig;
          globalConfig = bestConfig;
          cout << "HC found better solution with cost " << globalCost
               << " (time = " << timer.Current() << ")" << endl;
          for (ArchConfig::PCIter it = globalConfig->Params().begin();
               it != globalConfig->Params().end(); ++it)
            cout << it->first << '=' << it->second << "; ";

          cout << "\tThr = " << setw(7) << globalCost << endl;
        }
      }
  }
}

//=======================================================================
/*
 * Calculates area overhead of the current config represented by ArchPlanner.
 * Returns nonpositive value if there's no overhead.
 */

double HcEngine::GetAreaOverhead(ArchPlanner& ap) const
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

  return totalArea - config.MaxArea();
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
