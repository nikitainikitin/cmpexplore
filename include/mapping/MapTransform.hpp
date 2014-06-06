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

#ifndef _MAPPING_MAPTRANSFORM_H_
#define _MAPPING_MAPTRANSFORM_H_

#include <string>
#include <vector>
#include <algorithm>

#include "Config.hpp"
#include "CmpConfig.hpp"
#include "mapping/MapConf.hpp"
#include "TechDefs.hpp"
#include "workload/WlConfig.hpp"
#include "cmp/Memory.hpp"

using std::string;
using std::vector;

using namespace std;

namespace cmpex {

  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern workload::WlConfig wlConfig;

  using namespace cmp;
  using namespace workload;

  namespace mapping {

  typedef WlConfig::Task Task;
  typedef WlConfig::Thread Thread;

    //======================================================================
    // MapTransform represents the interface for transformations used by
    // metaheuristical mapping engines.
    //======================================================================

    struct MapTransform {

      // ---------------------------- Methods ------------------------------

      // Destructor
      virtual ~MapTransform () {}
      
      // Main method that transforms the mapping.
      // Returns true if applied successfully.
      virtual bool UpdateMap(MapConf& mConf) const = 0;

    };


    //======================================================================
    // -1- Swap the workload of two random processors inside a cluster.
    //======================================================================
    struct MapTrSwapThreadPairInL3Cluster : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        // choose an L3 cluster
        int cl_idx = RandInt(cmpConfig.L3ClusterCnt());

        IdxArray& tiles = cmpConfig.GetTilesOfL3Cluster(cl_idx);
        // generate two different core indices within the cluster
        int core1_idx = tiles[RandInt(tiles.size())];
        int core2_idx = tiles[RandInt(tiles.size())];
        while (core1_idx == core2_idx)
          core2_idx = tiles[RandInt(tiles.size())];

        std::swap(mConf.map[core1_idx], mConf.map[core2_idx]);
        return true;
      }
    };

    //======================================================================
    // -2- Change activity of a random core.
    //======================================================================
    struct MapTrChangeCoreActiv : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = RandInt(mConf.coreCnt);
        // inverse activity
        mConf.coreActiv[core_idx] = !mConf.coreActiv[core_idx];
        return true;
      }
    };

    //======================================================================
    // -3- Increase frequency of a random core.
    //======================================================================
    struct MapTrIncreaseCoreFreq : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = RandInt(mConf.coreCnt);
        if (mConf.coreFreq[core_idx] <= MAX_FREQ - FREQ_STEP) {
          mConf.coreFreq[core_idx] += FREQ_STEP;
        }
        return true;
      }
    };

    //======================================================================
    // -4- Decrease frequency of a random core.
    //======================================================================
    struct MapTrDecreaseCoreFreq : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = RandInt(mConf.coreCnt);
        if (mConf.coreFreq[core_idx] >= 2*FREQ_STEP) {
          mConf.coreFreq[core_idx] -= FREQ_STEP;
        }
        return true;
      }
    };

    //======================================================================
    // -5- Change activity of a random L3 cluster.
    //======================================================================
    struct MapTrChangeL3ClusterActiv : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int cluster_idx = RandInt(mConf.L3ClusterCnt);
        // inverse activity
        mConf.L3ClusterActiv[cluster_idx] = !mConf.L3ClusterActiv[cluster_idx];
        return true;
      }
    };

    //======================================================================
    // -6- Swap two random tasks between clusters.
    //======================================================================
    struct MapTrSwapTaskPairBetweenL3Clusters : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        if (wlConfig.running_tasks.size() < 2) return false;

        bool success = false;

        // choose a random task
        int core_idx = RandInt(mConf.coreCnt);
        while (mConf.map[core_idx] == MapConf::IDX_UNASSIGNED)
          core_idx = RandInt(mConf.coreCnt);

        Task * task = wlConfig.GetThreadByGid(mConf.map[core_idx])->task;
        int taskClIdx = cmpConfig.GetMemory(core_idx)->L3ClusterIdx();

        // 1. First try to move the task to free cores in some other cluster

        // iterate over L3 clusters,
        // check how many procs available in this cluster
        vector<int> freeProcsPerL3Cluster;
        for (int cl = 0; cl < cmpConfig.L3ClusterCnt(); ++cl) {
          IdxArray& tiles = cmpConfig.GetTilesOfL3Cluster(cl);
          int freeCnt = 0;
          for (IdxCIter tile_it = tiles.begin(); tile_it != tiles.end(); ++tile_it) {
            if (mConf.map[*tile_it] == MapConf::IDX_UNASSIGNED) {
              ++freeCnt;
            }
          }
          freeProcsPerL3Cluster.push_back(freeCnt);
        }

        // find cluster with the max number of free procs != taskClIdx
        int maxClIdx = 0;
        for (int i = 0; i < freeProcsPerL3Cluster.size(); ++i) {
          if (i != taskClIdx &&
              freeProcsPerL3Cluster[i] > freeProcsPerL3Cluster[maxClIdx])
            maxClIdx = i;
        }

        // there are enough free procs in some cluster
        if (freeProcsPerL3Cluster[maxClIdx] >= task->task_dop) {
          int clIdx = RandInt(cmpConfig.L3ClusterCnt());
          while (clIdx == taskClIdx || freeProcsPerL3Cluster[clIdx] < task->task_dop)
            clIdx = RandInt(cmpConfig.L3ClusterCnt());

          // now move the task to the L3 cluster given by clIdx
          // remove old assignment
          /*IdxArray& tiles = cmpConfig.GetTilesOfL3Cluster(taskClIdx);
          for (IdxCIter tile_it = tiles.begin(); tile_it != tiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid != MapConf::IDX_UNASSIGNED &&
                wlConfig.GetThreadByGid(th_gid)->task->task_id == task->task_id)
              mConf.map[*tile_it] = MapConf::IDX_UNASSIGNED;
          }

          // do new assignment
          IdxCIter tile_it = cmpConfig.GetTilesOfL3Cluster(clIdx).begin();
          for (int thread = 0; thread < task->task_dop; ++thread) {
            // global id of next thread to be assigned
            int th_gid = task->task_threads[thread]->thread_gid;
            // find the next free proc
            while (mConf.map[*tile_it] != MapConf::IDX_UNASSIGNED) {
              ++tile_it;
            }
            mConf.map[*tile_it] = th_gid;
            ++tile_it;
          }*/

          return true;
        }

        // 2. Failed to move task to free cores. Try swapping with some other task.


        // limit the number of trials
        /*int max_trials = 100;
        int trial_cnt = 0;
        bool success = false;
        do {


          ++trial_cnt;
        } while (!success && trial_cnt < max_trials);
        return success;*/
        return false;
      }
    };

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPTRANSFORM_H_
