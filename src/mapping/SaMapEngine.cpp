// ----------------------------------------------------------------------
//   Copyright 2011-2014 Nikita Nikitin <nikita.i.nikitin@gmail.com>
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
#include <cmath>

#include "SaMapEngine.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include <Timer.hpp>
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "PhysicalModel.hpp"
#include "RouterDefs.hpp"
#include "power/PowerModel.hpp"
#include "MapConf.hpp"
#include "MapTransform.hpp"
#include "workload/WlConfig.hpp"
#include "cmp/Processor.hpp"

using namespace std;

namespace cmpex {

  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern stat::Statistics stats;
  extern workload::WlConfig wlConfig;

  using namespace cmp;
  using namespace arch;
  using namespace stat;
  using namespace perf;
  using namespace phys;
  using namespace power;
  using namespace workload;

  namespace mapping {

  typedef WlConfig::Task Task;
  typedef WlConfig::Thread Thread;

//=======================================================================
/*
 * Constructors and destructor
 */

SaMapEngine::SaMapEngine() {}

SaMapEngine::~SaMapEngine() {}

//=======================================================================
/*
 * Calculate initial temprature for SA.
 * Perform mcnt random moves and aim that the probability of accepting
 * downhill solutions at initial temperature is equal to prob.
 */

double SaMapEngine::GetInitTemp(const MapConf * mc, int mcnt, double prob) const
{
  double downhillCost = 0.0;
  int downhillMoves = 0;

  MapConf * curMap = new MapConf(*mc);
  double prevCost = curMap->cost;

  for (int iter = 0; iter < mcnt || !downhillMoves; ++iter) {
    // change curMap to get new random solution
    int idx = int(RandUDouble()*Transforms().size());
    Transforms()[idx]->UpdateMap(*curMap);

    // Estimate cost of new mapping
    EvalMappingCost(curMap, 1.0);

    // Record delta cost and accept solution
    double deltaCost = curMap->cost - prevCost;
    if (deltaCost < 0) {
      downhillCost += -deltaCost;
      ++downhillMoves;
    }

    prevCost = curMap->cost;
  }

  delete curMap;

  double tInit = -downhillCost/downhillMoves/log(prob);

  cout << "downhillMoves = " << downhillMoves
       << ", avg downillCost = " << downhillCost/downhillMoves
       << ", tInit = " << tInit << endl;

  return tInit;
}

//=======================================================================
/*
 * Main method that invokes the mapping.
 */

void SaMapEngine::Map(MapConf * mc, bool silent_mode)
{
  MapConf *curMap, *bestMap;

  Timer timer;
  timer.Start();

  double lambda = 0.5;

  // 1. Initialize mapping with mc or greedily
  // For now, assume that all cores will be busy
  curMap = (mc ? new MapConf(*mc) : CreateGreedyMapping());
  bestMap = 0;
  EvalMappingCost(curMap, lambda);
  if (!silent_mode) {
    cout << "Initial Thr = " << curMap->thr << ", Pow = " << curMap->power << endl;
    curMap->Print();
  }

  // 2. Run the annealing schedule

  // number of local iterations
  double lIterCnt = cmpConfig.ProcCnt()*4;

  double tInit = GetInitTemp(curMap, lIterCnt, 0.95);
  double tCur = tInit;
  double alpha = config.SaAlpha();

  int cntAll = 0;
  int cntAcc = 0;
  int cntAccProb = 0;

  if (!silent_mode) cout << "Number of local iters = " << lIterCnt << endl;

  double no_impr_limit = lIterCnt*lIterCnt*config.SEffort();
  double last_impr = 0;
  int oIter = 0;

  do { // outer cooling loop

    for (int iter = 0; iter < lIterCnt; ++iter) { // inner cooling loop
      // 2a. Generate new solution (apply transform)
      MapConf * newMap = new MapConf(*curMap); // copy current mapping

      int idx = int(RandUDouble()*Transforms().size());
      Transforms()[idx]->UpdateMap(*newMap);

      //if (idx == 1) cout << "NM:::"; newMap->Print();

      // 2b. Estimate cost of new mapping
      EvalMappingCost(newMap, lambda);

      // 2c. Decide acceptance
      lambda = tInit/tCur;
      double curC = curMap->cost;
      double newC = newMap->cost;

      cntAll++;
      if (newC >= curC) { // accept
        if (curMap != bestMap) delete curMap;
        curMap = newMap;
        cntAcc++;
      }
      else if ( RandUDouble() < std::exp(-(curC-newC)/tCur) ) { // accept
        if (curMap != bestMap) delete curMap;
        curMap = newMap;
        cntAccProb++;
      }
      else { // reject
        delete newMap;
      }

      // 2d. Update best mapping
      double bestObj = bestMap ? bestMap->obj : 0.0;
      //bool cur_budgets_met = (fabs(curMap->cost-curMap->thr) < E_DOUBLE);
      bool cur_budgets_met = (config.MaxPower() - curMap->power > 0);
      if (curMap->obj > bestObj && cur_budgets_met) {
        last_impr = oIter*lIterCnt + iter;
        if (bestMap) delete bestMap;
        bestMap = curMap;
        if (!silent_mode) {
          cout << "(Time " << timer.Current() << ") ";
          curMap->Print();
        }
      }
    }

    //cout << "cntAll = " << cntAll;
    //cout << ", ProbAcc = " << double(cntAcc)/cntAll;
    //cout << ", ProbAccProb = " << double(cntAccProb)/cntAll << endl;

    tCur = tCur*alpha;
    ++oIter;
  } while (oIter*lIterCnt - last_impr < no_impr_limit);

  // if this assert fails it is probably that no feasible solution has been found,
  // check if the constraints are too strict
  assert(bestMap);

  if (!silent_mode) {
    cout << "Finished search (no improvement during the last "
         << no_impr_limit << " iterations)" << endl;

    cout << "Best Thr = " << bestMap->thr << endl;
    cout << "Params: tCur = " << tCur << endl;
  }

  // cleanup
  if (curMap != bestMap) delete curMap;
  if (mc) { // copy data back to config
    if (mc != bestMap) *mc = *bestMap;
  }
  else {
    delete bestMap;
  }
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
