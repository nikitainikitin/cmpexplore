#include <iostream>
#include <string>
#include <vector>

#include "WlConfig.hpp"
#include "model/Function.hpp"

using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace cmpex {

  using model::Powerlaw;

  namespace workload {

//=======================================================================
/*
 * Constructors and destructor
 */

WlConfig::WlConfig ( void ) {}

WlConfig::~WlConfig () {

  for (int i = 0; i < tasks.size(); ++i) {
    for (int j = 0; j < tasks[i]->task_dop; ++j)
      delete tasks[i]->task_threads[j];
    delete tasks[i];
  }

}


//=======================================================================
/*
 * Creates Tasks
 */

int WlConfig::CreateTasks ( int ntasks )
{

  int i,j;
  Task * task = 0;
  Thread * thread = 0;
  XYloc * xyloc = 0;
  int instr, dop;

  int thread_gid = 0;
  for(i = 0; i < ntasks; i++) {
    task = new WlConfig::Task;
    task->task_id = i;
    task->task_status = PENDING;
    task->task_cluster_id = -1; // default, unmapped
    instr = INSTR_MIN+RandInt(INSTR_MAX-INSTR_MIN+1); // in [INSTR_MIN,INSTR_MAX]
    //task->task_instructions = instr;
    task->task_deadline = DEADLINE; // ms
    //task->task_progress = 0;
    //task->task_elapsed = 0;
    //task->task_slack = DEADLINE; // ms
    task->task_ipc = IPC_MIN+(IPC_MAX-IPC_MIN)*RandUDouble(); // in [IPC_MIN,IPC_MAX]
    task->task_mpi = MPI_MIN+(MPI_MAX-MPI_MIN)*RandUDouble(); // in [IPC_MIN,IPC_MAX]
    dop = 1 << RandInt(LOG2_DOP_MAX+1); // in [1,2**(LOG2_DOP_MAX)]
    task->task_dop = dop;
    // use randomly generated powerlaw functions for missRatioOfMemSize
    double alpha = MR_ALPHA_MIN + (MR_ALPHA_MAX-MR_ALPHA_MIN)*RandUDouble();
    double exp = MR_EXP_MIN + (MR_EXP_MAX-MR_EXP_MIN)*RandUDouble();
    task->missRatioOfMemSize = new Powerlaw(alpha, exp);

    if (task) AddTask(task);
    for(j = 0; j < dop; j++) {
      thread = new WlConfig::Thread;
      thread->thread_id = j;
      thread->thread_gid = thread_gid;
      thread->task = task;
      thread->thread_status = PENDING;
      thread->thread_xyloc.x= -1; // unmapped, default value
      thread->thread_xyloc.y= -1; // unmapped, default value
      thread->thread_instructions = instr; // inherited
      thread->thread_progress = 0;
      thread->thread_ipc = task->task_ipc; // inherited
      thread->thread_mpi = task->task_mpi; // inherited
      thread->missRatioOfMemSize = task->missRatioOfMemSize; // inherited
      if (thread) AddThread(thread,i);
      xyloc = new WlConfig::XYloc;
      xyloc->x = -1;
      xyloc->y = -1;
      if (xyloc) AddThreadLoc(xyloc,i);

      ++thread_gid;
    }
  }

  return 0;

}

int WlConfig::PrintTasks ( int ntasks )
{
  int tot = TaskCnt();
  for(int i = 0; (i < ntasks) && (i < tot); i++) {
    cout << "Task " << i << " ID = " << tasks[i]->task_id << ", DOP = " << tasks[i]->task_dop
         << ", instr = " << tasks[i]->task_threads[0]->thread_instructions;
    cout << ", status = " << TaskStatusString(i);
    cout << ", ipc = " << tasks[i]->task_ipc << ", mpi = " << tasks[i]->task_mpi << ", mr = ";
    tasks[i]->missRatioOfMemSize->Print();
    cout << ", threads: ";
    for(int j = 0; j < tasks[i]->task_dop; j++) {
      cout << tasks[i]->task_threads[j]->thread_gid << "(" << tasks[i]->task_cluster_xyloc[j]->x << "," << tasks[i]->task_cluster_xyloc[j]->y << "),";
    }
    cout << endl;
  }

  return tot;

}


void WlConfig::Cleanup()
{
  tasks.clear();
}

//=======================================================================
/*
 * Returns true if there is at least one pending task.
 */

bool WlConfig::HasPendingTasks ( void )
{
  for (TaskCIter it = tasks.begin(); it != tasks.end(); ++it) {
    if ((*it)->task_status == PENDING) return true;
  }
  return false;
}

//=======================================================================
/*
 * Returns next pending task in the order of creation.
 */

WlConfig::Task * WlConfig::GetNextPendingTask ( void )
{
  for (TaskCIter it = tasks.begin(); it != tasks.end(); ++it) {
    if ((*it)->task_status == PENDING) return *it;
  }
  return 0;
}

//=======================================================================
/*
 * Returns true if all tasks have completed.
 */

bool WlConfig::AllTasksCompleted ( void )
{
  for (TaskCIter it = tasks.begin(); it != tasks.end(); ++it) {
    if ((*it)->task_status != COMPLETED) return false;
  }
  return true;
}

//=======================================================================

  } // namespace workload

} // namespace cmpex
