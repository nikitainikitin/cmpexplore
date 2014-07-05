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
#include "MeshIc.hpp"
#include "Cluster.hpp"

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

MapSim::MapSim(int period_us) : period_ (period_us), prevMap(0) {}

MapSim::~MapSim() {
  if (prevMap) delete prevMap;
}

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

  bool lastPeriodWlChanged = true; // if workload changed in the last period

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
             << min(100,int(100*(double(thread->thread_progress)/thread->thread_instructions)))
             << "%" << endl;
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
      // start execution. The beginning of execution can be delayed
      // to prioritize those threads which are close to their deadlines.
      for (int th = 0; th < nextTask->task_dop; ++th) {
        nextTask->task_threads[th]->thread_status = WlConfig::SCHEDULED;
      }
      nextTask->task_status = WlConfig::SCHEDULED;
      lastPeriodWlChanged = true;
    }

    cout << "   -MAP- Period instantaneous QoS = " << wlConfig.GetInstantQoS()
         << ", total QoS = " << wlConfig.GetTotalQoS() << endl;

    // 2. Find the new best mapping
    //cout << "***** Mapconf before mapping: "; mconf->Print();

    // 2.a. Unset suspended state for migrated cores - assume migration penalty is 1 period
    for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
      Thread * thread = (mconf->map[p] != MapConf::IDX_UNASSIGNED) ?
          wlConfig.GetThreadByGid(mconf->map[p]) : 0;
      if (thread && thread->thread_status == WlConfig::SUSPENDED) {
        cout << "   -MAP- MIGRATION: Restoring suspended thread with id "
             << thread->thread_gid << endl;
        thread->thread_status = WlConfig::RUNNING;
        lastPeriodWlChanged = true;
      }
    }

    // 2.b. Run SA-based remapping
    SaMapEngine me;
    if (sysElapsedPeriod == 0 || !SkipRemapping(lastPeriodWlChanged)) {
      cout << "   -MAP- Running SA mapping..." << endl;
      //me.Map(mconf, prevMap, prevProcThr, true); // run SA mapping
      if(!(me.Map(mconf, prevMap, prevProcThr, true))) {// failure due to temperature violation
	cout << "-MAP- Re-running SA with all off initial condition..." << endl;
	me.Map(mconf, prevMap, prevProcThr, true); // re-run SA mapping starting with
      }
    }
    else {
      cout << "   -MAP- Skipped SA mapping for this period: "
           << "no change in workload, all budgets satisfied" << endl;
    }
    lastPeriodWlChanged = false; // period change history is reset after mapping
    cout << "   -MAP- Mapconf after mapping: "; mconf->Print();

    // 3.a. Run performance model with the new mapping;
    //    reuse cost estimation function from mapping
    me.EvalMappingCost(mconf, prevMap, prevProcThr);

    // 3.b. Suspend migrating threads
    SuspendMigratingThreads(mconf);

    PrintMappingChange(mconf);

    // Run Hotspot every hs_period_sec
    //double hs_period_sec = 0.01;
    double hs_period_sec = 0.001;
    int hs_period_usec = int(hs_period_sec * 1e6);
    if ((sysElapsedPeriod*PeriodUs() % hs_period_usec) == 0) {
      vector<double> power_vec;
      PowerModel::CreatePTsimPowerVector(cmpConfig.Cmp(), power_vec);
      cout << "-I- Running Hotspot..." << endl;
      PTsim::CallHotSpot(cmpConfig.Cmp(), &power_vec, true);

      cout << endl;
      cout << "   -MAP- Temperature of cores (real values): ";
      for (int p = 0; p < mconf->map.size(); ++p) {
        cout << PTsim::CoreTemp(p) << ' ';
      }
      cout << endl;
      cout << "   -MAP- Temperature of cores (est. values): ";
      for (int p = 0; p < mconf->map.size(); ++p) {
        cout << PTsim::CoreTempEst(p) << ' ';
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
      Thread * thread = (mconf->map[p] != MapConf::IDX_UNASSIGNED) ?
          wlConfig.GetThreadByGid(mconf->map[p]) : 0;
      if (thread && thread->thread_status != WlConfig::SUSPENDED) {
        // increment number of completed instructions
        // by instr_per_ns * length_period_ns
        cout << "   -MAP- Advancing thread " << thr_id << " progress by "
             << cmpConfig.GetProcessor(p)->Thr()*PeriodUs()*1000 << " instructions" << endl;
        ///cout << "Thread progress before " << thread->thread_progress;
        ///cout << ", advancing for " << cmpConfig.GetProcessor(p)->Thr()*PeriodUs()*1000;
        thread->thread_progress += cmpConfig.GetProcessor(p)->Thr()*PeriodUs()*1000;
        ///cout << ", thread progress now " << thread->thread_progress << endl;
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
          //mconf->coreActiv[p] = false;
          cout << "   -MAP- Thread " << thr_id << " has completed" << endl;
          lastPeriodWlChanged = true;
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

    // 6. Save history mapping and throughput
    SaveHistory(mconf);

    //cout << "   -MAP- Period instantaneous QoS = " << wlConfig.GetInstantQoS()
    //     << ", total QoS = " << wlConfig.GetTotalQoS() << endl;

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
/*
 * Returns true if remapping can be skipped for this period.
 */

bool MapSim::SkipRemapping(bool lastPeriodWlChanged) {
  // DEBUG
  cout << "ENTERING SKIP REMAPPING...";

  if (lastPeriodWlChanged) return false;

  int num_cores_on = 0;
  //bool all_cores_off = true;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    if (cmpConfig.GetProcessor(p)->Thr() > E_DOUBLE) {
      //all_cores_off = false;
      num_cores_on++;
      //break;
    }
  }
  
  //if (all_cores_off) return false;
  if (num_cores_on == 0) return false;

  // evaluate temperature at the end of the next period
  // if running with the same mapping

  // DEBUG
  cout << "ESTIMATING TEMPERATURE...";

  // obtaining the previous power pattern
  vector<double> power_vec;
  PowerModel::CreatePTsimPowerVector(cmpConfig.Cmp(), power_vec);

  // updating the predicted temperatures
  PTsim::PredictTemp(cmpConfig.Cmp(), &power_vec);  

  double max_temp = 0.0;

  // MCs
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    max_temp = max(max_temp, PTsim::MCTempEst(mc));
  }

  // Tiles
  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());
  for (int tile = 0; tile < mic->TCnt(); ++tile) {
    max_temp = max(max_temp, PTsim::GetMaxEstTempInTile(tile));
  }
  
  // DEBUG
  cout << "MAX TEMP = " << max_temp << endl;


  double temp_slack = config.MaxTemp() - max_temp;
  double min_temp_slack = 10.0;
  double total_power = 0;
  for (int i=0 ; i < power_vec.size(); i++) {
    total_power += power_vec[i];
  }
  double power_slack = config.MaxPower() - total_power;
  double min_power_slack_fraction = 0.2;

  if ((num_cores_on < cmpConfig.ProcCnt()) &&
      (wlConfig.HasPendingTasks()) &&
      (temp_slack > min_temp_slack) && 
      (power_slack > min_power_slack_fraction*config.MaxPower())) {
    return false;
  }

  double min_freq = MAX_FREQ;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    if (cmpConfig.GetProcessor(p)->Freq() < min_freq) {
      min_freq = cmpConfig.GetProcessor(p)->Freq();
      break;
    }
  }

  if ((min_freq < MAX_FREQ) &&
      (!wlConfig.HasPendingTasks()) &&
      (temp_slack > min_temp_slack) && 
      (power_slack > min_power_slack_fraction*config.MaxPower())) {
    return false;
  }

  bool temp_satisfied = (max_temp <= config.MaxTemp());
  return temp_satisfied;
}

//=======================================================================
/*
 * Saves mapping history for previous period.
 */

void MapSim::SaveHistory ( MapConf * mconf ) {
  int coreCnt = cmpConfig.ProcCnt();

  // processor throughput
  if (!prevMap) {
    prevProcThr.resize(coreCnt);
  }
  for (int p = 0; p < coreCnt; ++p) {
    prevProcThr[p] = cmpConfig.GetProcessor(p)->Thr();
  }

  // mapping configuration
  if (prevMap) {
    delete prevMap;
  }
  prevMap = new MapConf(*mconf);

}

//=======================================================================
/*
 * Prints changes in mapping, w.r.t the previous mapping.
 */

void MapSim::PrintMappingChange ( MapConf * mconf )  const {
  if (!prevMap) return;

  int coreCnt = cmpConfig.ProcCnt();
  int L3ClusterCnt = cmpConfig.ProcCnt()/cmpConfig.L3ClusterSize();

  int mapChangedCnt = 0;
  int activChangedCnt = 0;
  int freqChangedCnt = 0;
  for (int p = 0; p < coreCnt; ++p) {
    if (mconf->coreActiv[p] != prevMap->coreActiv[p]) ++activChangedCnt;
    if (mconf->coreActiv[p] &&
          mconf->map[p] != prevMap->map[p]) ++mapChangedCnt;
    if (mconf->coreActiv[p] &&
          mconf->coreFreq[p] != prevMap->coreFreq[p]) ++freqChangedCnt;
  }

  int L3ActivChangedCnt = 0;
  for (int c = 0; c < L3ClusterCnt; ++c) {
    if (mconf->L3ClusterActiv[c] != prevMap->L3ClusterActiv[c]) ++L3ActivChangedCnt;
  }

  cout << "   -MAP- Number of processors changed values w.r.t. prevMap:"
       << " mapping = " << mapChangedCnt << "; activity = " << activChangedCnt
       << "; frequency = " << freqChangedCnt << "; L3activity = " << L3ActivChangedCnt
       << endl;
}

//=======================================================================
/*
 * Suspends threads which are migrating between the clusters
 * during the next period.
 */

void MapSim::SuspendMigratingThreads ( MapConf * mconf ) {
  if (!prevMap) return;

  // identify migrating threads: those whose map indices change
  // so that they migrate between the clusters
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mconf->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mconf->map[p]) : 0;
    // NOTICE: we perform check only for RUNNING threads.
    // Since the status of the thread has not been updated for this period,
    // this check will only apply to the threads which ran at previous period.
    // This is fine, because we don't consider migration penalty for the
    // scheduled threads.
    if (thread && thread->thread_status == WlConfig::RUNNING) {
      // check if thread has migrated to another cluster
      bool migratedToOtherCluster = false;
      std::vector<int>::const_iterator it =
          find(prevMap->map.begin(), prevMap->map.end(), mconf->map[p]);
      int prevProcIdx = -1;
      if (it != prevMap->map.end()) {
        prevProcIdx = it - prevMap->map.begin();
        assert(prevProcIdx >= 0 && prevProcIdx < cmpConfig.ProcCnt());
        // assume one memory per proc per tile
        if (cmpConfig.GetMemory(p)->L3ClusterIdx() !=
            cmpConfig.GetMemory(prevProcIdx)->L3ClusterIdx()) {
          migratedToOtherCluster = true;
        }
      }

      if (migratedToOtherCluster) {
        cout << "   -MAP- MIGRATION: Thread id " << thread->thread_gid
             << " has migrated between L3 clusters "
             << cmpConfig.GetMemory(prevProcIdx)->L3ClusterIdx() << " and "
             << cmpConfig.GetMemory(p)->L3ClusterIdx() << " and will be suspended" << endl;
        thread->thread_status = WlConfig::SUSPENDED;
      }
    }
  }
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
