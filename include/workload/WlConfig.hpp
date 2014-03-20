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

      // Data structure for a thread.
      struct Thread {
        int thread_id;
	XYloc thread_xyloc; //0(1) is x(y) coordinate in cmp
	int thread_instructions;
	int thread_progress;
	Status thread_status;
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
        double ipc;
        double mpi;
	int task_instructions; // total instructions to be executed
	int task_progress; // instructions executed till NOW
	int task_deadline; // deadline time in ms
	int task_elapsed; // task runtime in ms
	int task_slack; // remaining time from NOW till deadline in ms
	ThreadArray task_threads;
	Status task_status;
      };

      typedef std::vector<Task*> TaskArray;

      typedef TaskArray::const_iterator TaskCIter;

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

      // Create Tasks
      int CreateTasks ( int ntasks );

      // DEBUG: Print Tasks
      int PrintTasks ( int ntasks );

      void Cleanup();

    private:

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------
    
    void WlConfig::AddTask ( WlConfig::Task * t ) {
      tasks.push_back(t);
    }

    void WlConfig::AddThread ( WlConfig::Thread * t, int i ) {
      (tasks[i]->task_threads).push_back(t);
    }

    void WlConfig::AddThreadLoc ( WlConfig::XYloc * xy, int i ) {
      (tasks[i]->task_cluster_xyloc).push_back(xy);
    }

    int WlConfig::TaskCnt () const {
      return tasks.size();
    }

    string WlConfig::TaskStatusString ( int i) {
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

  } // namespace workload

} // namespace cmpex

#endif // _WL_CONFIG_H_
