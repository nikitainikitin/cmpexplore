#include <iostream>
#include <string>
#include <vector>

#include "WlConfig.hpp"

using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace cmpex {

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

  for(i = 0; i < ntasks; i++) {
    task = new WlConfig::Task;
    task->task_id = i;
    task->task_status = PENDING;
    int instr = 1000000+RandInt(1000001); // between 1M and 2M instructions
    task->task_instructions = instr;
    task->task_deadline = 100; // ms
    task->task_progress = 0;
    task->task_elapsed = 0;
    int dop = 1 << RandInt(7);
    task->task_dop = dop; 
    task->task_progress = 0;
    if (task) AddTask(task);
    for(j = 0; j < dop; j++) {
      thread = new WlConfig::Thread;
      thread->thread_id = j;
      thread->thread_status = PENDING;
      thread->thread_xyloc.x= -1; // unmapped, default value
      thread->thread_xyloc.y= -1; // unmapped, default value
      thread->thread_instructions = instr;
      thread->thread_progress = 0;
      if (thread) AddThread(thread,i);
      xyloc = new WlConfig::XYloc;
      xyloc->x = -1;
      xyloc->y = -1;
      if (xyloc) AddThreadLoc(xyloc,i);
    }
  }

  return 0;

}

int WlConfig::PrintTasks ( int ntasks )
{
  int tot = TaskCnt();
  for(int i = 0; (i < ntasks) && (i < tot); i++) {
    cout << "Task " << i << " ID = " << tasks[i]->task_id << ", DOP = " << tasks[i]->task_dop << ", instr = " << tasks[i]->task_instructions; 
    cout << ", status = " << TaskStatusString(i) << ", threads: ";
    for(int j = 0; j < tasks[i]->task_dop; j++) {
      cout << tasks[i]->task_threads[j]->thread_id << "(" << tasks[i]->task_cluster_xyloc[j]->x << "," << tasks[i]->task_cluster_xyloc[j]->y << "),";
    }
    cout << endl;
  }

  return tot;

}


void WlConfig::Cleanup()
{
  tasks.clear();
}

  } // namespace workload
} // namespace cmpex
