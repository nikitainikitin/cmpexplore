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
#include <algorithm>

#include "MapSim.hpp"
#include "SaMapEngine.hpp"
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
#include "PTsim.hpp"

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
  using namespace temperature;

  namespace mapping {

  typedef WlConfig::Task Task;
  typedef WlConfig::Thread Thread;

//=======================================================================
/*
 * Constructors and destructor
 */

MapSim::MapSim(int period_us) : period_ (period_us) {}

MapSim::~MapSim() {}

//=======================================================================
/*
 * Main method that invokes simulation
 */

void MapSim::Run() {

  int sysElapsedPeriod = 0; // system time in multiples of MapSim::period_
  int coreCnt = cmpConfig.ProcCnt();
  int L3ClusterCnt = cmpConfig.ProcCnt()/cmpConfig.L3ClusterSize();

  // this is current mapping configuration
  MapConf * mconf = new MapConf(coreCnt, L3ClusterCnt);

  int maxPeriods = 1000;

  cout << "-I- MapSim: starting mapping simulator for " << maxPeriods << " periods" << endl;
  cout << "-I- MapSim: one period simulates " << PeriodUs() << " usec ("
       << PeriodUs()/1e6 << " sec) of system time" << endl;

  // start the mapping loop
  while (!wlConfig.AllTasksCompleted() && sysElapsedPeriod < maxPeriods) {

    // PRINT SYSTEM INFO
    cout << "-I- MapSim: sysElapsedPeriod = " << sysElapsedPeriod
         << ", system time = " << sysElapsedPeriod*PeriodUs()/1e6 << " sec" << endl;
    for (int p = 0; p < mconf->map.size(); ++p) {
      int thr_id = mconf->map[p];
      if (thr_id != MapConf::IDX_UNASSIGNED) {
        Thread * thread = wlConfig.GetThreadByGid(thr_id);
        cout << "   -MAP- Thread " << thr_id << " from task " << thread->task->task_id
             << " is mapped to proc " << p << ", progress = "
             << min(100,100*thread->thread_progress/thread->thread_instructions) << "%" << endl;
      }
    }

    // 1. Map new tasks
    while (wlConfig.HasPendingTasks()) {
      Task * nextTask = wlConfig.GetNextPendingTask();
      if (nextTask->task_dop > cmpConfig.L3ClusterSize()) {
        cout << "-E-: Can't assign task (id = " << nextTask->task_id << "), since its DOP = "
             << nextTask->task_dop << " is larger than the L3 cluster size = "
             << cmpConfig.L3ClusterSize() << " -> Exiting... " << endl;
        exit(1);
      }

      bool assigned = mconf->AssignTaskToFreeProcs(nextTask->task_id);
      // assignment fails if not enough free procs available
      if (!assigned) break;

      cout << "   -MAP- Mapping new task (id = "
           << nextTask->task_id << ") with DOP = " << nextTask->task_dop << endl;

      // NOTE: task and its threads remain in the scheduled state until they
      // starts execution. The beginning of execution can be delayed
      // to prioritize those threads which are close to their deadlines.
      for (int th = 0; th < nextTask->task_dop; ++th) {
        nextTask->task_threads[th]->thread_status = WlConfig::SCHEDULED;
      }
      nextTask->task_status = WlConfig::SCHEDULED;
    }

    // 2. Find the new best mapping
    cout << "***** Mapconf before mapping: "; mconf->Print();
    cout << "   -MAP- Running SA mapping..." << endl;
    SaMapEngine me;
    me.Map(mconf, true); // run SA mapping
    //me.EvalMappingCost(mconf); // no remapping, evaluate default mapping only
    cout << "   -MAP- Mapconf after mapping: "; mconf->Print();

    // 3. Run performance model with the new mapping;
    //    reuse cost estimation function from mapping
    me.EvalMappingCost(mconf);

    // Run Hotspot every hs_period_sec
    //double hs_period_sec = 0.01;
    double hs_period_sec = 0.001;
    int hs_period_usec = int(hs_period_sec * 1e6);
    if ((sysElapsedPeriod*PeriodUs() % hs_period_usec) == 0) {
      vector<double> power_vec;
      PowerModel::CreatePTsimPowerVector(cmpConfig.Cmp(), power_vec);
      cout << "-I- Running Hotspot..." << endl;
      PTsim::CallHotSpot(cmpConfig.Cmp(), &power_vec, true);

      cout << "   -MAP- Temperature of cores: ";
      for (int p = 0; p < mconf->map.size(); ++p) {
        cout << PTsim::CoreTemp(p) << ' ';
      }
      cout << endl;
    }

    // BEGIN PRINTOUT
    cout << "   -MAP- Throughput of cores: ";
    for (int p = 0; p < mconf->map.size(); ++p) {
      cout << cmpConfig.GetProcessor(p)->Thr() << ' ';
    }
    cout << endl;

    cout << "   -MAP- Frequency of cores: ";
    for (int p = 0; p < mconf->map.size(); ++p) {
      cout << cmpConfig.GetProcessor(p)->Freq() << ' ';
    }
    cout << endl;
    // END PRINTOUT

    // 4. Advance every task
    for (int p = 0; p < mconf->map.size(); ++p) {
      int thr_id = mconf->map[p];
      if (thr_id != MapConf::IDX_UNASSIGNED) {
        Thread * thread = wlConfig.GetThreadByGid(thr_id);
        // increment number of completed instructions
        // by instr_per_ns * length_period_ns
        cout << "   -MAP- Advancing thread " << thr_id << " progress by "
             << cmpConfig.GetProcessor(p)->Thr()*PeriodUs()*1000 << " instructions" << endl;
        thread->thread_progress += cmpConfig.GetProcessor(p)->Thr()*PeriodUs()*1000;
        // Mark as running tasks and threads who have started execution
        if (thread->thread_progress > 0 && thread->thread_status != WlConfig::RUNNING) {
          assert(thread->thread_status = WlConfig::SCHEDULED);
          thread->thread_status = WlConfig::RUNNING;
          thread->task->task_status = WlConfig::RUNNING;
          // Add to the running queue, when processing first thread from the task
          bool task_not_in_the_running_queue =
            find(wlConfig.running_tasks.begin(), wlConfig.running_tasks.end(), thread->task) ==
              wlConfig.running_tasks.end();
          if (task_not_in_the_running_queue) {
            wlConfig.running_tasks.push_back(thread->task);
          }
        }
      }
    }

    for (WlConfig::TaskCIter it = wlConfig.running_tasks.begin();
                             it != wlConfig.running_tasks.end(); ++it) {
      (*it)->task_elapsed += PeriodUs() / 1000; // period in ms
    }

    // 5. Check for completed tasks
    for (int p = 0; p < mconf->map.size(); ++p) {
      int thr_id = mconf->map[p];
      if (thr_id != MapConf::IDX_UNASSIGNED) {
        Thread * thread = wlConfig.GetThreadByGid(thr_id);
        // check if thread has completed
        if (thread->CheckProgressMarkCompleted()) {
          // release core
          mconf->map[p] = MapConf::IDX_UNASSIGNED;
          mconf->coreActiv[p] = false;
          cout << "   -MAP- Thread " << thr_id << " has completed" << endl;
        }
        // check if the task has completed
        if (thread->task->CheckProgressMarkCompleted()) {
          cout << "   -MAP- Task " << thread->task->task_id << " has completed" << endl;
          // update the running_tasks and completed_tasks queues
          bool task_in_the_running_queue =
            find(wlConfig.running_tasks.begin(), wlConfig.running_tasks.end(), thread->task) !=
              wlConfig.running_tasks.end();
          assert(task_in_the_running_queue);
          bool task_not_in_the_completed_queue =
            find(wlConfig.completed_tasks.begin(), wlConfig.completed_tasks.end(), thread->task) ==
              wlConfig.completed_tasks.end();
          assert(task_not_in_the_completed_queue);
          WlConfig::TaskIter it =
              find(wlConfig.running_tasks.begin(), wlConfig.running_tasks.end(), thread->task);
          wlConfig.running_tasks.erase(it);
          wlConfig.completed_tasks.push_back(thread->task);
        }
      }
    }

    cout << "   -MAP- Period instantaneous QoS = " << wlConfig.GetInstantQoS()
         << ", total QoS = " << wlConfig.GetTotalQoS() << endl;

    // advance system time
    ++sysElapsedPeriod;
    cout << " * * * " << endl;
    wlConfig.PrintTasks();
  }

  if (wlConfig.AllTasksCompleted()) {
    // sanity checks
    assert(wlConfig.running_tasks.empty());
    assert(wlConfig.completed_tasks.size() == wlConfig.tasks.size());

    cout << "-I- All tasks completed after " << sysElapsedPeriod << " periods ("
         << sysElapsedPeriod*PeriodUs()/1e6 << " sec)" << endl;
  }
  else {
    cout << "-I- There are running/pending tasks left after " << sysElapsedPeriod << " periods ("
         << sysElapsedPeriod*PeriodUs()/1e6 << " sec)" << endl;
  }

}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
