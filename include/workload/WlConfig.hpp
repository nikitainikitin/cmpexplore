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

    // generated tasks
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
    const int DEADLINE = 12; // ms
    const int LOG2_DOP_MAX = 6; // max 2**6 = 64 threads per task
    const int MAX_SMT_DEG = 8; // max 8 threads per core (simultaneous multithreading)
    const int LOG2_MAX_SMT_DEG = 3; // log2(MAX_SMT_DEG)

    // predefined tasks
    /*const double TASK_IPC [] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const double TASK_MPI [] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
                                0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    const double TASK_MR_ALPHA [] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    const double TASK_MR_EXP [] = {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
    const double TASK_INSTR [] = {2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6,
                                  2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6, 2e6};
    const int TASK_DEADLINE [] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
                                  20, 20, 20, 20, 20, 20, 20, 20, 20, 20}; // ms
    const int TASK_DOP [] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2, 2, 2, 2, 2, 2, 2};*/

    const double TASK_IPC [] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0};
    const double TASK_MPI [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0};
    const double TASK_MR_ALPHA [] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
0.1, 0.1, 0.1, 0.1,
                                     0.1, 0.1, 0.1, 0.1};
    const double TASK_MR_EXP [] = {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1, -0.1, -0.1,
-0.1, -0.1, -0.1, -0.1,
                                   -0.1, -0.1, -0.1, -0.1};
    const double TASK_INSTR [] = {20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6, 20e6, 20e6,
20e6, 20e6, 20e6, 20e6,
                                  20e6, 20e6, 20e6, 20e6};
    const int TASK_DEADLINE [] = {6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                  6, 6, 6, 6};
    const int TASK_DOP [] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                             2, 2, 2, 2};



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

      enum Status {PENDING, SCHEDULED, RUNNING, SUSPENDED, COMPLETED};

      struct Task;

      // Data structure for a thread.
      struct Thread {
        int thread_id; // thread local id within the task
        int thread_gid; // thread global id
	int thread_dop; // superthread degree of parallelism
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
        int task_elapsed; // task runtime in ms
        //int task_start; // timestamp when task has started in ms
        //int task_finish; // timestamp when task has finished in ms
        //int task_slack; // remaining time from NOW till deadline in ms
        ThreadArray task_threads;
        Status task_status;

        inline bool CheckProgressMarkCompleted ();

        // Get task QoS, defined on the scale (0.0, 1.0].
        // QoS is 1.0 if deadline constraint is not violated
        // and it is decreased proportionally to the square of violation slack.
        inline double GetQoS() const;

      };

      typedef std::vector<Task*> TaskArray;

      typedef TaskArray::iterator TaskIter;

      typedef TaskArray::const_iterator TaskCIter;

      typedef vector<Thread*> MapIdxToThreads;

      TaskArray tasks;

      TaskArray completed_tasks;

      TaskArray running_tasks;

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

      inline Task * GetTask ( int t_id );

      // Create Tasks
      int CreateTasks ( int ntasks, bool predefined = false );

      int ReadTasks ( const string& fname );

      bool HasPendingTasks ();

      Task * GetNextPendingTask ();

      bool AllTasksCompleted ();

      // Instantaneous QoS over running tasks
      inline double GetInstantQoS () const;

      // Total QoS over all tasks that have ever started
      inline double GetTotalQoS () const;

      // DEBUG: Print Tasks
      int PrintTasks ( int ntasks = 0 );

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
        case SCHEDULED:
        status_string = "SCHEDULED"; break;
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

    WlConfig::Task * WlConfig::GetTask ( int t_id ) {
      return tasks[t_id];
    }

    double WlConfig::GetInstantQoS() const {
      double qos = 0;
      for (TaskCIter it = running_tasks.begin();  it != running_tasks.end(); ++it) {
        qos += (*it)->GetQoS();
      }

      int total_tasks = running_tasks.size();
      return total_tasks ? qos/total_tasks : 1.0;
    }

    double WlConfig::GetTotalQoS() const {
      double qos = 0;
      for (TaskCIter it = running_tasks.begin();  it != running_tasks.end(); ++it) {
        qos += (*it)->GetQoS();
      }
      for (TaskCIter it = completed_tasks.begin();  it != completed_tasks.end(); ++it) {
        qos += (*it)->GetQoS();
      }

      int total_tasks = running_tasks.size()+completed_tasks.size();
      return total_tasks ? qos/total_tasks : 1.0;
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

    double WlConfig::Task::GetQoS() const {
      if (task_deadline - task_elapsed >= 0)
        return 1.0;
      double relative_slack = double(task_deadline)/double(task_elapsed);
      return relative_slack*relative_slack;
    }

  } // namespace workload

} // namespace cmpex

#endif // _WL_CONFIG_H_
