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

void SaMapEngine::Map()
{
  int coreCnt = cmpConfig.ProcCnt();

  MapConf *curMap, *bestMap;

  Timer timer;
  timer.Start();

  double lambda = 0.5;

  // 1a. Initialize mapping greedily
  // For now, assume that all cores will be busy
  curMap = bestMap = new MapConf(coreCnt);

  int busyCores = 0;
  Task * nextTask = wlConfig.GetNextPendingTask();
  while (nextTask && nextTask->task_dop <= (coreCnt - busyCores)) {
    for (int th = 0; th < nextTask->task_dop; ++th) {
      // assign thread to the next available core
      curMap->map[busyCores] = nextTask->task_threads[th]->thread_gid;
      ++busyCores;
      // mark thread as running
      nextTask->task_threads[th]->thread_status = WlConfig::RUNNING;
    }
    nextTask->task_status = WlConfig::RUNNING;
    nextTask = wlConfig.GetNextPendingTask();
  }

  // assign states (speeds) for all active cores
  for (int p = 0; p < coreCnt; ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    curMap->states[p] = (curMap->map[p] != MapConf::IDX_UNASSIGNED) ?
                        proc->Freq() : 0.0;
  }

  // 1b. Evaluate initial mapping
  EvalMappingCost(curMap, lambda);
  cout << "Thr = " << curMap->thr << ", Pow = " << curMap->power << endl;
  curMap->Print();

  // 2. Run the annealing schedule

  /// TODO

}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
