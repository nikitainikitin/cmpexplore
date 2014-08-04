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
#include <algorithm>

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

  //AddTransform(new MapTrChangeCoreActiv());
  //AddTransform(new MapTrIncreaseCoreFreq());
  //AddTransform(new MapTrDecreaseCoreFreq());
  AddTransform(new MapTrChangeL3ClusterActiv());

  //if (cmpConfig.L3ClusterCnt() > 1)
  //  AddTransform(new MapTrMoveTaskToOtherL3Cluster());

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

double MapEngine::CalcNewQoSObjPenalty(MapConf * mc, const vector<double>& prevProcThr) const
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


  // 2. Penalty for running and suspended tasks
  int procCnt = cmpConfig.ProcCnt();
  double run_dl_viol_pen_at_start = double(procCnt-1)/(procCnt-1+1.0/sch_dl_viol_penalty);
  run_dl_viol_pen_at_start *= run_dl_viol_pen_at_start;
  double run_dl_viol_pen_at_end = 1.0 - 2*run_dl_viol_pen_at_start;

  vector<double> run_thr_penalties;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread && (thread->thread_status == WlConfig::RUNNING ||
                   thread->thread_status == WlConfig::SUSPENDED)) {
      //cout << "Thread id " << thread->thread_gid << " is running" << endl;
      double instr_to_complete = thread->thread_instructions - thread->thread_progress;
      // thr. to estimate exec. time for suspended task is taken from the previous period
      double thr = (thread->thread_status == WlConfig::RUNNING) ?
                     cmpConfig.GetProcessor(p)->Thr() : prevProcThr[p];
      double ms_to_complete = thr > E_DOUBLE ? instr_to_complete/(thr*1e6) : MAX_DOUBLE;
      assert(ms_to_complete >= 0);
      // deadline for suspended task is reduced by migration period of 1 ms
      double deadline = (thread->thread_status == WlConfig::RUNNING) ?
                          thread->task->task_deadline : thread->task->task_deadline-1;
      double slack_avail_ms = deadline - thread->task->task_elapsed;
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
    else if (thread && thread->thread_status == WlConfig::SUSPENDED) {

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
 * Returns predicted temperature of the mapping solution.
 * Calculates maximum temperature on the chip.
 */

double MapEngine::GetTemperature() const
{
  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  double max_temp = 0.0;

  // MCs
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    double mc_temp = PTsim::MCTempEst(mc);
    max_temp = max(max_temp, mc_temp);
  }

  // Tiles
  for (int tile = 0; tile < mic->TCnt(); ++tile) {
    double tile_temp = PTsim::GetMaxEstTempInTile(tile);
    max_temp = max(max_temp, tile_temp);
  }

  return max_temp;
}

//=======================================================================
/*
 * Evaluates cost of the provided mapping solution.
 */

void MapEngine::EvalMappingCost(MapConf * mc, MapConf * prevMap,
                                const vector<double>& prevProcThr, double lambda) const
{
  // 1. Prepare configuration: initialize processors according to the mapping

  // create list of migrating threads to be suspended
  vector<Thread*> suspendedThreads;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (thread && thread->thread_status == WlConfig::RUNNING) {
      // check if thread has migrated to another cluster
      std::vector<int>::const_iterator it =
          find(prevMap->map.begin(), prevMap->map.end(), mc->map[p]);
      if (it != prevMap->map.end()) {
        int prevProcIdx = it - prevMap->map.begin();
        assert(prevProcIdx < cmpConfig.ProcCnt());
        // assume one memory per proc per tile
        if (cmpConfig.GetMemory(p)->L3ClusterIdx() !=
            cmpConfig.GetMemory(prevProcIdx)->L3ClusterIdx()) {
          suspendedThreads.push_back(thread);
        }
      }
    }
  }

  // temporarily set suspended state
  for (int th = 0; th < suspendedThreads.size(); ++th) {
    suspendedThreads[th]->thread_status = WlConfig::SUSPENDED;
  }


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
    if (thread && thread->thread_status != WlConfig::SUSPENDED) {
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
      //proc->SetFreq(0.0);
      //proc->SetVolt(PowerModel::VoltAtFreqProc(mc->coreFreq[p]));
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
  //double obj_penalty = config.QoS() ?
  //      (config.NewQoS() ? CalcNewQoSObjPenalty(mc, prevProcThr) : CalcQoSObjPenalty(mc)) : 1.0;

  // 3. Save evaluation within the mapping object
  mc->thr = pSm->Throughput();
  mc->power = power;
  mc->temp = GetTemperature();
  mc->obj = mc->thr/**obj_penalty*/;

  // cost including hard penalties
  double cost = mc->obj * power_penalty * temp_penalty;
  mc->cost = cost;
  delete pSm;

  // restore threads which were temporarily suspended
  for (int th = 0; th < suspendedThreads.size(); ++th) {
    suspendedThreads[th]->thread_status = WlConfig::RUNNING;
  }
}

//=======================================================================
/*
 * A heuristic that chooses active cores and their frequencies.
 */

void MapEngine::ChooseActiveCores(MapConf * mc, MapConf * prevMap,
                                  const vector<double>& prevProcThr) const
{
  vector<double> corePriorities;
  const double UNASSIGNED_PRIORITY = -1.0;
  corePriorities.assign(cmpConfig.ProcCnt(), UNASSIGNED_PRIORITY);

  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  // 1. Go over all cores and assign priorities to the respective threads
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);
    Thread * thread = (mc->map[p] != MapConf::IDX_UNASSIGNED) ?
        wlConfig.GetThreadByGid(mc->map[p]) : 0;
    if (!thread) continue;
    if (thread->thread_status == WlConfig::RUNNING ||
        thread->thread_status == WlConfig::SCHEDULED) {
      double instr_to_complete = thread->thread_instructions - thread->thread_progress;
      double slack_avail_ms = thread->task->task_deadline - thread->task->task_elapsed;
      bool deadline_not_passed = (slack_avail_ms > 0);
      // approximate slack_avail_ms = 0.5 ms (< 1 ms) if the deadline has passed
      double req_thr_ipns = deadline_not_passed ? instr_to_complete/(slack_avail_ms*1e6) :
                                                  instr_to_complete/(0.5*1e6);
      // calculate minimum req freq under worst-case uncore latency
      proc->SetSMTDegree(thread->thread_dop);
      proc->SetMemAccessProbabilities(thread->missRatioOfMemSize);
      double cont_coeff = 1.2;
      double wc_l3_noc_hops = 2.0*sqrt(cmpConfig.L3ClusterSize())-1.0;
      double wc_l3_noc_lat_cyc = wc_l3_noc_hops*(mic->LinkDelay()+mic->RouterDelay());
      double wc_l3_lat_cyc = cmpConfig.GetMemory(p)->Latency() + wc_l3_noc_lat_cyc*cont_coeff;
      double wc_mc_noc_hops = sqrt(cmpConfig.ProcCnt())-1.0;
      double wc_mc_noc_lat_cyc = wc_mc_noc_hops*(mic->LinkDelay()+mic->RouterDelay());
      double wc_mc_lat_cyc = cmpConfig.GetMemCtrl(0)->latency + wc_mc_noc_lat_cyc*cont_coeff;
      double wc_lat_pen_cyc = proc->L1AccessProbability()*proc->L1Lat() +
                              proc->L2AccessProbability()*proc->L2Lat() +
                              proc->L3AccessProbability()*wc_l3_lat_cyc +
                              proc->MainMemAccessProbability()*wc_mc_lat_cyc;
      double wc_cpi = 1.0/thread->thread_ipc + thread->thread_mpi*wc_lat_pen_cyc;
      double req_freq = CalcMinReqFreq(proc, req_thr_ipns, wc_cpi);
      // use minimum req freq as priority
      corePriorities[p] = req_freq;
      //cout << "core " << p << ": req_thr_ipns = " << req_thr_ipns
      //     << ", wc_lat_pen = " << wc_lat_pen_cyc << ", wc_cpi = " << wc_cpi << endl;
      // decrease the priority of scheduled threads by 50%
      if (thread->thread_status == WlConfig::SCHEDULED)
        corePriorities[p] /= 2.0;
    }
    else {
      cout << "-E- ChooseActiveCores: Thread status is different from SCHEDULED and RUNNING"
           << " -> Exiting..." << endl;
      exit(1);
    }
  }

  cout << "   -MAP- Core priorities:";
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    cout << ' ' << corePriorities[p];
  }
  cout << endl;

  // 2. Iteratively choose cores with the highest priorities and activate them,
  //    as long as within the budgets
  int tasks_available_cnt = 0;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    if (corePriorities[p] != UNASSIGNED_PRIORITY) {
      ++tasks_available_cnt;
    }
  }

  mc->coreActiv.assign(cmpConfig.ProcCnt(), false);
  bool budgets_met = true;
  while (tasks_available_cnt > 0 && budgets_met) {
    // get the core with the highest priority
    int core_idx = max_element(corePriorities.begin(), corePriorities.end()) -
                     corePriorities.begin();
    // activate the core and set its frequency
    Thread * thread = wlConfig.GetThreadByGid(mc->map[core_idx]);
    mc->coreActiv[core_idx] = true;
    double core_priority = corePriorities[core_idx];
    if (thread->thread_status == WlConfig::SCHEDULED) core_priority *= 2.0;
    mc->coreFreq[core_idx] = MapCorePriorityToFreq(core_priority);
    //cout << "Priority = " << core_priority << ", freq = " << mc->coreFreq[core_idx] << endl;
    // remove the core from priority list
    corePriorities[core_idx] = UNASSIGNED_PRIORITY;
    // evaluate the system state
    EvalMappingCost(mc, prevMap, prevProcThr, 1.0);
    budgets_met = (config.MaxPower() - mc->power > 0) &&
                  (config.MaxTemp() - mc->temp > 0);
    if (!budgets_met)
      mc->coreActiv[core_idx] = false;
    else {
      //cout << "Enabling core " << core_idx << " at freq " << mc->coreFreq[core_idx];
      //cout << ", pow =  " << mc->power << endl;
    }
    --tasks_available_cnt;
  }

  // 3. If there is power budget left, distribute it among the active cores.
  if (budgets_met) {
    int core_idx = 0;
    while (budgets_met) {
      int prev_core_idx = core_idx;
      bool core_found = true;
      while (!mc->coreActiv[core_idx] || mc->coreFreq[core_idx] > MAX_FREQ-FREQ_STEP) {
        core_idx = (core_idx+1)%mc->coreActiv.size();
        if (prev_core_idx == core_idx) {
          core_found = false;
          break;
        }
      }
      if (!core_found) break;

      mc->coreFreq[core_idx] += FREQ_STEP;
      // evaluate the system state
      EvalMappingCost(mc, prevMap, prevProcThr, 1.0);
      budgets_met = (config.MaxPower() - mc->power > 0) &&
                    (config.MaxTemp() - mc->temp > 0);
      if (!budgets_met) {
        mc->coreFreq[core_idx] -= FREQ_STEP;
        break;
      }
      ++core_idx;
    }
  }
  cout << "!!!!!!!!!! Core activity seletion finished" << endl;
}

//=======================================================================
/*
 * Return frequency of the core corresponding to the provided priority.
 */

double MapEngine::MapCorePriorityToFreq ( double core_priority ) const
{
  if (core_priority >= MAX_FREQ)
    return MAX_FREQ;
  if (core_priority <= MIN_FREQ)
    return MIN_FREQ;

  double freq = MIN_FREQ;
  while (core_priority > freq) freq += FREQ_STEP;
  return freq;
}

//=======================================================================
/*
 * Calculates required minimum frequency for the processor.
 */

double MapEngine::CalcMinReqFreq ( Processor * proc,
                 double req_thr_ipns, double wc_cpi_thread ) const
{
  // if min == SMT/CPI_thread
  double freq1 = req_thr_ipns * wc_cpi_thread / proc->SMTDegree();

  // if min == PW*freq
  double freq2 = req_thr_ipns / proc->PLWidth();

  //cout << "freq1 = " << freq1 << ", freq2 = " << freq2 << endl;

  return max(freq1, freq2);
}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
