#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "WlConfig.hpp"
#include "model/Function.hpp"
#include "Parser.hpp"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::ifstream;

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

int WlConfig::CreateTasks ( int ntasks, bool predefined )
{

  int i,j;
  Task * task = 0;
  Thread * thread = 0;
  XYloc * xyloc = 0;
  int instr, dop;

  if (predefined) ntasks = sizeof(TASK_IPC)/sizeof(double);

  int thread_gid = 0;
  for(i = 0; i < ntasks; i++) {
    task = new WlConfig::Task;
    task->task_id = i;
    task->task_status = PENDING;
    task->task_cluster_id = -1; // default, unmapped
    instr = predefined ? TASK_INSTR[i] : INSTR_MIN+RandInt(INSTR_MAX-INSTR_MIN+1); // in [INSTR_MIN,INSTR_MAX]
    //task->task_instructions = instr;
    task->task_deadline = predefined ? TASK_DEADLINE[i] : DEADLINE; // ms
    //task->task_progress = 0;
    task->task_elapsed = 0;
    //task->task_slack = DEADLINE; // ms
    task->task_ipc = predefined ? TASK_IPC[i] : IPC_MIN+(IPC_MAX-IPC_MIN)*RandUDouble(); // in [IPC_MIN,IPC_MAX]
    task->task_mpi = predefined ? TASK_MPI[i] : MPI_MIN+(MPI_MAX-MPI_MIN)*RandUDouble(); // in [IPC_MIN,IPC_MAX]
    dop = predefined ? TASK_DOP[i] : 1 << RandInt(LOG2_DOP_MAX+1); // in [1,2**(LOG2_DOP_MAX)]
    if (((dop >> 1) << 1) == dop) {// dop is even
      task->task_dop = dop >> 1; // dop is divided by 2 because of two-thread parallelism in our core
    }
    else { // dop is odd
      task->task_dop = (dop >> 1)+1; // dop is divided by 2, but we need to consider the extra thread (remainder)
    }
    int num_threads = task->task_dop;
    // use randomly generated powerlaw functions for missRatioOfMemSize
    double alpha = predefined ? TASK_MR_ALPHA[i] : MR_ALPHA_MIN + (MR_ALPHA_MAX-MR_ALPHA_MIN)*RandUDouble();
    double exp = predefined ? TASK_MR_EXP[i] : MR_EXP_MIN + (MR_EXP_MAX-MR_EXP_MIN)*RandUDouble();
    task->missRatioOfMemSize = new Powerlaw(alpha, exp);
    //task->task_start = -1;
    //task->task_finish = -1;

    if (task) AddTask(task);
    for(j = 0; j < num_threads; j++) {
      thread = new WlConfig::Thread;
      thread->thread_id = j;
      thread->thread_gid = thread_gid;
      thread->task = task;
      thread->thread_status = PENDING;
      thread->thread_xyloc.x= -1; // unmapped, default value
      thread->thread_xyloc.y= -1; // unmapped, default value
      thread->thread_progress = 0;
      if (j != (dop >> 1)) { 
	thread->thread_instructions = 2 * instr; //two threads in parallel
	thread->thread_ipc = 2 * task->task_ipc; // two threads in parallel
      }
      else {
	thread->thread_instructions = instr; //one thread remainder
	thread->thread_ipc = task->task_ipc; // one thread remainder
      }
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
  if (ntasks == 0) ntasks = TaskCnt();

  int tot = TaskCnt();
  for(int i = 0; (i < ntasks) && (i < tot); i++) {
    cout << "Task " << i << " ID = " << tasks[i]->task_id << ", DOP = " << tasks[i]->task_dop
         << ", instr = " << tasks[i]->task_threads[0]->thread_instructions;
    cout << ", status = " << TaskStatusString(i);
    cout << ", ipc = " << tasks[i]->task_ipc << ", mpi = " << tasks[i]->task_mpi << ", mr = ";
    tasks[i]->missRatioOfMemSize->Print();
    /*cout << ", threads: ";
    for(int j = 0; j < tasks[i]->task_dop; j++) {
      cout << tasks[i]->task_threads[j]->thread_gid << "(" << tasks[i]->task_cluster_xyloc[j]->x << "," << tasks[i]->task_cluster_xyloc[j]->y << "),";
    }*/
    cout << ", QoS = " << tasks[i]->GetQoS();
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
/*
 * Read tasks from file.
 */

int WlConfig::ReadTasks ( const string & fname )
{
  ifstream in(fname.c_str());

  if (!in.good()) {
    cout << "-E- Can't read from file " << fname.c_str() << " -> Exiting..." << endl;
    exit(1);
  }

  string line;

  int thread_gid = 0;
  int task_id = 0;

  while (getline(in, line)) {

    LStrip(line);

    // skip empty lines
    if (line.empty()) continue;

    // skip comments
    if (line[0] == '#') continue;

    // parse task parameters
    double ipc, mpi, mr_alpha, mr_exp, instr_dbl;
    int instr, deadl, dop;
    ValueParser::ExtractValue(line, ipc);
    ValueParser::ExtractValue(line, mpi);
    ValueParser::ExtractValue(line, mr_alpha);
    ValueParser::ExtractValue(line, mr_exp);
    ValueParser::ExtractValue(line, instr_dbl);
    ValueParser::ExtractValue(line, deadl);
    ValueParser::ExtractValue(line, dop);
    instr = int(instr_dbl);

    // create a task
    /// TODO: merge code with CreateTasks()
    Task * task = new Task();
    task->task_id = task_id;
    task->task_status = PENDING;
    task->task_cluster_id = -1; // default, unmapped
    task->task_deadline = deadl;
    task->task_elapsed = 0;
    task->task_ipc = ipc;
    task->task_mpi = mpi;
    if (((dop >> 1) << 1) == dop) {// dop is even
      task->task_dop = dop >> 1; // dop is divided by 2 because of two-thread parallelism in our core
    }
    else { // dop is odd
      task->task_dop = (dop >> 1)+1; // dop is divided by 2, but we need to consider the extra thread (remainder)
    }
    task->missRatioOfMemSize = new Powerlaw(mr_alpha, mr_exp);
    AddTask(task);

    for(int j = 0; j < task->task_dop; j++) {
      Thread * thread = new Thread();
      thread->thread_id = j;
      thread->thread_gid = thread_gid;
      thread->task = task;
      thread->thread_status = PENDING;
      thread->thread_xyloc.x= -1; // unmapped, default value
      thread->thread_xyloc.y= -1; // unmapped, default value
      thread->thread_progress = 0;
      if (j != (dop >> 1)) {
        thread->thread_instructions = 2 * instr; //two threads in parallel
        thread->thread_ipc = 2 * task->task_ipc; // two threads in parallel
      }
      else {
        thread->thread_instructions = instr; //one thread remainder
        thread->thread_ipc = task->task_ipc; // one thread remainder
      }
      thread->thread_mpi = task->task_mpi; // inherited
      thread->missRatioOfMemSize = task->missRatioOfMemSize; // inherited
      if (thread) AddThread(thread,task_id);
      XYloc * xyloc = new XYloc;
      xyloc->x = -1;
      xyloc->y = -1;
      if (xyloc) AddThreadLoc(xyloc,task_id);

      ++thread_gid;
    }
    ++task_id;
  }

  in.close();

  return task_id;
}

//=======================================================================

  } // namespace workload

} // namespace cmpex
