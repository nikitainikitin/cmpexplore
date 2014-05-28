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
 * Main method that invokes the mapping.
 */

void SaMapEngine::Map(MapConf * mconf, bool silent_mode)
{
  MapConf *curMap, *bestMap;

  Timer timer;
  timer.Start();

  double lambda = 0.5;

  // 1. Initialize mapping with mconf or greedily
  // For now, assume that all cores will be busy
  curMap = (mconf ? new MapConf(*mconf) : CreateGreedyMapping());
  bestMap = 0;
  EvalMappingCost(curMap, lambda);
  if (!silent_mode) {
    cout << "Initial Thr = " << curMap->thr << ", Pow = " << curMap->power << endl;
    curMap->Print();
  }

  // 2. Run the annealing schedule

  double tInit = 100;
  double tCur = tInit;
  double alpha = config.SaAlpha();

  // number of local iterations
  double lIterCnt = cmpConfig.ProcCnt()*4;

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
      lambda = 0.5*tInit/tCur;
      double curC = curMap->cost;
      double newC = newMap->cost;

      if (newC > curC) { // accept
        if (curMap != bestMap) delete curMap;
        curMap = newMap;
      }
      else if ( RandUDouble() < // accept
              std::exp(- curC/newC * curC/newC * tInit/tCur) ) {
        if (curMap != bestMap) delete curMap;
        curMap = newMap;
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
  if (mconf) { // copy data back to config
    if (mconf != bestMap) *mconf = *bestMap;
  }
  else {
    delete bestMap;
  }
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
