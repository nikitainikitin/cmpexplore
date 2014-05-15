#ifndef _WL_CONFIG_H_
#define _WL_CONFIG_H_

#include <iostream>
#include <string>
#include <vector>

#include "Defs.hpp"
#include "model/Function.hpp"

using std::string;

namespace cmpex {

  namespace model {
    class Function;
  }

  namespace workload {

    const double IPC_MIN = 1.0;
    const double IPC_MAX = 3.0;
    const double MPI_MIN = 0.1;
    const double MPI_MAX = 0.3;
    const double MR_ALPHA_MIN = 0.05;
    const double MR_ALPHA_MAX = 0.2;
    const double MR_EXP_MIN = -0.5;
    const double MR_EXP_MAX = -0.1;
    const int INSTR_MIN = 1000000;
    const int INSTR_MAX = 2000000;
    const int DEADLINE = 100; // ms
    const int LOG2_DOP_MAX = 2; // max 2**6 = 64 threads per task


    //======================================================================
    // Config is a storage for workload parameters.
    //======================================================================

    class WlConfig
    {

    public:

      struct XYloc {
        int x;
        int y;
      };

      typedef std::vector<XYloc*> XYlocArray;

      enum Status {PENDING, RUNNING, SUSPENDED, COMPLETED};

      struct Task;

      // Data structure for a thread.
      struct Thread {
        int thread_id; // thread local id within the task
        int thread_gid; // thread global id
        Task * task; // parent task
        XYloc thread_xyloc; //0(1) is x(y) coordinate in cmp
        int thread_instructions;
        int thread_progress;
        Status thread_status;
        double thread_ipc;
        double thread_mpi;
        model::Function * missRatioOfMemSize;

        inline bool CheckProgressMarkCompleted ();
      };

      typedef std::vector<Thread*> ThreadArray;

      // Data structure for a task.
      struct Task {
        ~Task() { delete missRatioOfMemSize; }
        int task_id;
        int task_dop; // degree of parallelism, i.e. number of threads
        int task_cluster_id;
        XYlocArray task_cluster_xyloc; // coordinates of all threads forming a cluster
        model::Function * missRatioOfMemSize;
        double task_ipc;
        double task_mpi;
        //int task_instructions; // total instructions to be executed
        //int task_progress; // instructions executed till NOW
        int task_deadline; // deadline time in ms
        //int task_elapsed; // task runtime in ms
        //int task_slack; // remaining time from NOW till deadline in ms
        ThreadArray task_threads;
        Status task_status;

        inline bool CheckProgressMarkCompleted ();
      };

      typedef std::vector<Task*> TaskArray;

      typedef TaskArray::iterator TaskIter;

      typedef TaskArray::const_iterator TaskCIter;

      typedef vector<Thread*> MapIdxToThreads;

      TaskArray tasks;

      // ---------------------------- Methods ------------------------------

    public:

      WlConfig ( void );

      ~WlConfig ();

      inline void AddTask ( Task * t );

      inline void AddThread ( Thread * t, int i );

      inline void AddThreadLoc ( XYloc * xy, int i );

      inline int TaskCnt () const;

      inline string TaskStatusString ( int i );

      // Notice: this function is not safe, you have to be sure
      // that gid actually points to some thread
      inline Thread * GetThreadByGid ( int gid );

      // Create Tasks
      int CreateTasks ( int ntasks );

      bool HasPendingTasks ();

      Task * GetNextPendingTask ();

      bool AllTasksCompleted ();

      // DEBUG: Print Tasks
      int PrintTasks ( int ntasks );

      void Cleanup();

    private:

      MapIdxToThreads threads;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------
    
    void WlConfig::AddTask ( WlConfig::Task * t ) {
      tasks.push_back(t);
    }

    void WlConfig::AddThread ( WlConfig::Thread * t, int i ) {
      (tasks[i]->task_threads).push_back(t);
      if (threads.size() <= t->thread_gid) threads.resize(2*t->thread_gid+1);
      threads[t->thread_gid] = t;
    }

    void WlConfig::AddThreadLoc ( WlConfig::XYloc * xy, int i ) {
      (tasks[i]->task_cluster_xyloc).push_back(xy);
    }

    int WlConfig::TaskCnt () const {
      return tasks.size();
    }

    string WlConfig::TaskStatusString ( int i ) {
      string status_string;
      switch (tasks[i]->task_status) {
        case PENDING:
    status_string = "PENDING"; break;
        case RUNNING:
	  status_string = "RUNNING"; break;
        case SUSPENDED:
	  status_string = "SUSPENDED"; break;
        case COMPLETED:
	  status_string = "COMPLETED"; break;	  
      }
      return status_string;
    }

    WlConfig::Thread * WlConfig::GetThreadByGid ( int gid ) {
      return threads[gid];
    }

    // Thread inline functions

    bool WlConfig::Thread::CheckProgressMarkCompleted() {
      if (thread_progress >= thread_instructions) {
        thread_status = COMPLETED;
        return true;
      }
      return false;
    }

    // Task inline functions

    bool WlConfig::Task::CheckProgressMarkCompleted() {
      for (int i = 0; i < task_threads.size(); ++i) {
        if (task_threads[i]->thread_status != COMPLETED) return false;
      }
      task_status = COMPLETED;
      return true;
    }

  } // namespace workload

} // namespace cmpex

#endif // _WL_CONFIG_H_
