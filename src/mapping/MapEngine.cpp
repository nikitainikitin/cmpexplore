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
#include <set>

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
#include "cmp/Memory.hpp"
#include "ptsim/PTsim.hpp"
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

MapEngine::MapEngine() {
  // create transformations
  if (cmpConfig.L3ClusterSize() > 1)
    AddTransform(new MapTrSwapThreadPairInL3Cluster());

  AddTransform(new MapTrChangeCoreActiv());
  AddTransform(new MapTrIncreaseCoreFreq());
  AddTransform(new MapTrDecreaseCoreFreq());
  AddTransform(new MapTrChangeL3ClusterActiv());

  if (cmpConfig.L3ClusterCnt() > 1)
    AddTransform(new MapTrMoveTaskToOtherL3Cluster());

  AddTransform(new MapTrIncreaseUncoreFreq());
  AddTransform(new MapTrDecreaseUncoreFreq());
}

MapEngine::~MapEngine() {
  for (MapTrIter it = transforms_.begin(); it != transforms_.end(); ++it)
    delete *it;
}

//=======================================================================
/*
 * Create a greedy mapping solution.
 * Take pending tasks from the workload config sequentially and
 * assign them to the CMP cores.
 * Notice: a new mapping object is created and its ownership is
 * transferred to the calling function.
 */

MapConf * MapEngine::CreateGreedyMapping() const
{
  int coreCnt = cmpConfig.ProcCnt();
  int L3ClusterCnt = cmpConfig.ProcCnt()/cmpConfig.L3ClusterSize();

  MapConf * newMap = new MapConf(coreCnt, L3ClusterCnt);

  int busyCores = 0;
  Task * nextTask = wlConfig.GetNextPendingTask();
  while (nextTask && nextTask->task_dop <= (coreCnt - busyCores)) {
    cout << "GreedyMapping: next task with DOP = " << nextTask->task_dop << endl;
    for (int th = 0; th < nextTask->task_dop; ++th) {
      // assign thread to the next available core
      newMap->map[busyCores] = nextTask->task_threads[th]->thread_gid;
      ++busyCores;
      // mark thread as running
      // Greedy mapping starts threads as soon as they are scheduled.
      nextTask->task_threads[th]->thread_status = WlConfig::RUNNING;
    }
    nextTask->task_status = WlConfig::RUNNING;
    nextTask = wlConfig.GetNextPendingTask();
  }

  // assign states (speeds) for all active cores
  for (int p = 0; p < coreCnt; ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    newMap->coreActiv[p] = (newMap->map[p] != MapConf::IDX_UNASSIGNED) ?
                            true : false;
    newMap->coreFreq[p] = (newMap->map[p] != MapConf::IDX_UNASSIGNED) ?
                           proc->Freq() : 0.0;
  }

  return newMap;
}

//=======================================================================
/*
 * Returns QoS-penalty for the mapping objective.
 * Heuristic:
 * for every running thread
 *   Evaluate complete time with current execution rate
 *   Add penalty if deadline is violated at current rate
 */

double MapEngine::CalcQoSObjPenalty(MapConf * mc) const
{
  double total_qos_penalty = 0.0;
  int running_threads_cnt = 0;

  double sch_pen = 1.0; // penalty for scheduled tasks

  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread && (thread->thread_status == WlConfig::RUNNING ||
                   thread->thread_status == WlConfig::SCHEDULED && mc->coreActiv[p])) {
      //cout << "Thread id " << thread->thread_gid << " is running" << endl;
      double thread_penalty = 1.0;

      double instr_to_complete = thread->thread_instructions - thread->thread_progress;
      double ms_to_complete = cmpConfig.GetProcessor(p)->Thr() > E_DOUBLE ?
            instr_to_complete/(cmpConfig.GetProcessor(p)->Thr()*1e6) : MAX_DOUBLE;
      assert(ms_to_complete >= 0);
      double slack_avail_ms = thread->task->task_deadline - thread->task->task_elapsed;
      double slack_to_complete_ms = slack_avail_ms - ms_to_complete;
      //cout << "slack_avail = " << slack_avail_ms
      //     << ", ms_to_complete = " << ms_to_complete << endl;
      if (slack_to_complete_ms < 0) {
        // penalty
        double relative_violation = max(0.0, slack_avail_ms)/ms_to_complete;
        // set lower bound on penalty to 0.1
        thread_penalty = max(relative_violation*relative_violation, 0.1);
      }
      /*if (thread_penalty > 1.0) {
        cout << "thread_penalty = " << thread_penalty << endl;
        cout << "slack_avail = " << slack_avail_ms
             << ", ms_to_complete = " << ms_to_complete << endl;
      }*/
      assert(thread_penalty <= 1.0);
      //cout << "thread_penalty = " << thread_penalty << endl;

      // strong penalty for starting the tasks at speeds that
      // violate the deadline
      if (thread->thread_status == WlConfig::SCHEDULED && thread_penalty < 1.0-E_DOUBLE) {
        sch_pen /= 2.0;
      }

      total_qos_penalty += thread_penalty;
      ++running_threads_cnt;
    }
  }

  return sch_pen < 1.0 - E_DOUBLE ? sch_pen :
      (running_threads_cnt ? total_qos_penalty/running_threads_cnt : 1.0);
}

//=======================================================================
/*
 * Returns New QoS-penalty for the mapping objective.
 */

double MapEngine::CalcNewQoSObjPenalty(MapConf * mc) const
{
  // 1. Penalty for scheduled active tasks
  const double sch_dl_viol_penalty = 1.0/(2.0*cmpConfig.ProcCnt());

  int sch_cnt = 0;
  int sch_pen_cnt = 0;
  set<Task*> scheduled_tasks;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread && thread->thread_status == WlConfig::SCHEDULED && mc->coreActiv[p]) {
      //cout << "Thread id " << thread->thread_gid << " is scheduled active" << endl;
      double instr_to_complete = thread->thread_instructions - thread->thread_progress;
      double ms_to_complete = cmpConfig.GetProcessor(p)->Thr() > E_DOUBLE ?
            instr_to_complete/(cmpConfig.GetProcessor(p)->Thr()*1e6) : MAX_DOUBLE;
      assert(ms_to_complete >= 0);
      double thread_penalty = 1.0;
      // account one thread per task only
      if (thread->task->task_deadline < ms_to_complete) {
        thread_penalty = sch_dl_viol_penalty;
        if (scheduled_tasks.find(thread->task) == scheduled_tasks.end()) ++sch_pen_cnt;
      }
      //cout << "thread_penalty = " << thread_penalty << endl;
      if (scheduled_tasks.find(thread->task) == scheduled_tasks.end()) ++sch_cnt;
      scheduled_tasks.insert(thread->task);
    }
  }

  double qos_sch = (sch_cnt == 0) ? 1.0 : max(sch_cnt-sch_pen_cnt, 1)/
      (sch_pen_cnt/sch_dl_viol_penalty + (sch_cnt-sch_pen_cnt)*1.0);
  qos_sch *= qos_sch;
  //cout << "Scheduled active threads total " << sch_cnt
  //     << ", successful " << sch_pen_cnt << ", qos_sch " << qos_sch << endl;
  assert(qos_sch > 0);


  // 2. Penalty for running tasks
  int procCnt = cmpConfig.ProcCnt();
  double run_dl_viol_pen_at_start = double(procCnt-1)/(procCnt-1+1.0/sch_dl_viol_penalty);
  run_dl_viol_pen_at_start *= run_dl_viol_pen_at_start;
  double run_dl_viol_pen_at_end = 1.0 - 2*run_dl_viol_pen_at_start;

  vector<double> run_thr_penalties;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread && thread->thread_status == WlConfig::RUNNING) {
      //cout << "Thread id " << thread->thread_gid << " is running" << endl;
      double instr_to_complete = thread->thread_instructions - thread->thread_progress;
      double ms_to_complete = cmpConfig.GetProcessor(p)->Thr() > E_DOUBLE ?
            instr_to_complete/(cmpConfig.GetProcessor(p)->Thr()*1e6) : MAX_DOUBLE;
      assert(ms_to_complete >= 0);
      double slack_avail_ms = thread->task->task_deadline - thread->task->task_elapsed;
      double thread_penalty = 1.0;
      if (slack_avail_ms <= 0) {
        thread_penalty = run_dl_viol_pen_at_end;
      }
      else if (ms_to_complete > slack_avail_ms) {
        double task_rel_progr = double(thread->task->task_elapsed)/thread->task->task_deadline;
        assert(task_rel_progr <= 1.0);
        //thread_penalty = sqrt(1.0-task_rel_progr)*run_dl_viol_pen_at_start +
        //                 sqrt(task_rel_progr)*run_dl_viol_pen_at_end;
        thread_penalty = (1.0-task_rel_progr)*run_dl_viol_pen_at_start +
                         task_rel_progr*run_dl_viol_pen_at_end;
      }
      assert(thread_penalty <= 1.0);
      //cout << "thread_penalty = " << thread_penalty << endl;
      run_thr_penalties.push_back(thread_penalty);
    }
  }

  // calculate hmean for running threads penalties
  double qos_run = 1.0;
  if (!run_thr_penalties.empty()) {
    qos_run = 0.0;
    for (vector<double>::const_iterator it = run_thr_penalties.begin();
                                        it != run_thr_penalties.end(); ++it) {
      qos_run += 1.0/(*it);
    }
    qos_run = run_thr_penalties.size()/qos_run;
  }
  assert(qos_run > 0);

  //cout << "# running threads = " << run_thr_penalties.size()
  //     << ", qos_run = " << qos_run << endl;

  return qos_sch*qos_run;
}

//=======================================================================
/*
 * Returns temperature penalty for the mapping objective.
 * Calculates maximum temperature on the chip and adds
 * penalty proportional to the square of relative excess of temperature.
 */

double MapEngine::CalcTempPenalty(double lambda) const
{
  double temp_penalty = 1.0;

  lambda = 1.0;

  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  double max_temp_excess = 0.0;

  // MCs
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    double mc_temp = PTsim::MCTempEst(mc);
    double temp_excess = max(0.0, mc_temp - config.MaxTemp());
    max_temp_excess = max(max_temp_excess, temp_excess);
  }

  // Tiles
  for (int tile = 0; tile < mic->TCnt(); ++tile) {
    double tile_temp = PTsim::GetMaxEstTempInTile(tile);
    double temp_excess = max(0.0, tile_temp - config.MaxTemp());
    max_temp_excess = max(max_temp_excess, temp_excess);
  }

  // use square of maximum temp access
  temp_penalty = 1.0/(1.0 + lambda*max_temp_excess*max_temp_excess/config.MaxTemp());

  //cout << "   -MAP-DEBUG- Temperature penalty = " << temp_penalty << endl;

  return temp_penalty;
}

//=======================================================================
/*
 * Evaluates cost of the provided mapping solution.
 */

void MapEngine::EvalMappingCost(MapConf * mc, double lambda) const
{
  // 1. Prepare configuration: initialize processors according to the mapping

  // update L3 activities
  for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
    Memory * mem = cmpConfig.GetMemory(m);
    mem->SetActive(mc->L3ClusterActiv[mem->L3ClusterIdx()]);
  }

  // update Uncore voltage and frequency
  cmpConfig.UFreq(mc->uncoreFreq);
  cmpConfig.UVolt(PowerModel::VoltAtFreqU(mc->uncoreFreq));

  // !!! NOTICE: L3 activities have to be updated before
  // calling proc->SetMemAccessProbabilities(),
  // because probabilities are set depending on the L3 activity

  // update core states
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread) {
      proc->SetActive(mc->coreActiv[p]);
      proc->SetFreq(mc->coreFreq[p]);
      proc->SetVolt(PowerModel::VoltAtFreqProc(mc->coreFreq[p]));
      proc->SetIpc(thread->thread_ipc);
      proc->SetMpi(thread->thread_mpi);
      // NOTICE: set SMTDegree before updating memory access probabilities
      proc->SetSMTDegree(thread->thread_dop);
      proc->SetMemAccessProbabilities(thread->missRatioOfMemSize);
    }
    else {
      proc->SetActive(false);
      proc->SetFreq(0.0);
      proc->SetVolt(PowerModel::VoltAtFreqProc(mc->coreFreq[p]));
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
  // 2a. Performance
  IterativePerfModel m;
  StatMetrics * pSm = m.Run();

  // 2b. Power
  double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
  pSm->Power(power);
  double powerExcess = max(0.0, power - config.MaxPower());
  double power_penalty = 1.0 / (1.0 + lambda*powerExcess/config.MaxPower());

  // 2c. Temperature
  vector<double> power_vec;
  PowerModel::CreatePTsimPowerVector(cmpConfig.Cmp(), power_vec);
  if (!PTsim::WarmupDone()) {
    cout << "   -MAP- Running Hotspot warmup (first invocation)... " << flush;
    PTsim::CallHotSpot(cmpConfig.Cmp(), &power_vec, true);
    cout << "done." << endl;
  }
  PTsim::PredictTemp(cmpConfig.Cmp(), &power_vec);
  double temp_penalty = CalcTempPenalty(lambda);
  //double temp_penalty = 1.0;

  // 2d. Delay/qos
  double obj_penalty = config.QoS() ?
        (config.NewQoS() ? CalcNewQoSObjPenalty(mc) : CalcQoSObjPenalty(mc)) : 1.0;

  // 3. Save evaluation within the mapping object
  mc->thr = pSm->Throughput();
  mc->power = power;

  mc->obj = mc->thr*obj_penalty;

  // cost including hard penalties
  double cost = mc->obj * power_penalty * temp_penalty;
  mc->cost = cost;
  delete pSm;
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
