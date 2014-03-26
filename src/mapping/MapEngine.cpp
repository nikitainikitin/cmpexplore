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

#include "MapEngine.hpp"
#include "MapConf.hpp"
#include "MapTransform.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "phys/PhysicalModel.hpp"
#include "power/PowerModel.hpp"
#include "RouterDefs.hpp"
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

MapEngine::MapEngine() {
  // create transformations
  AddTransform(new MapTrSwapTaskPair());
  AddTransform(new MapTrChangeCoreState());
}

MapEngine::~MapEngine() {
  for (MapTrIter it = transforms_.begin(); it != transforms_.end(); ++it)
    delete *it;
}

//=======================================================================
/*
 * Evaluates cost of the provided mapping solution.
 */

void MapEngine::EvalMappingCost(MapConf * mc, double lambda) const
{
  // 1. Prepare configuration: initialize processors according to the mapping
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread) {
      proc->SetActive(true);
      proc->SetIpc(thread->thread_ipc);
      proc->SetMpi(thread->thread_mpi);
      proc->SetMemAccessProbabilities(thread->missRatioOfMemSize);
      proc->SetFreq(1.6);
    }
    else {
      proc->SetActive(false);
      proc->SetFreq(0.0);
    }
  }

  /*for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    cout << "ProcIdx = " << p << ", ipc = " << proc->Ipc() << ", mpi = " << proc->Mpi()
         << ", freq = " << proc->Freq()
         << ", L1Prob = " << proc->L1AccessProbability()
         << ", L2Prob = " << proc->L2AccessProbability()
         << ", L3Prob = " << proc->L3AccessProbability()
         << ", MMProb = " << proc->MainMemAccessProbability() << endl;
  }*/

  // 2. Run analytical models
  IterativePerfModel m;
  StatMetrics * pSm = m.Run();
  double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
  pSm->Power(power);

  // 3. Save evaluation within the mapping object
  mc->thr = pSm->Throughput();
  mc->power = power;
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
