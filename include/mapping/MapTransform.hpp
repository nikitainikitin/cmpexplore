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
#include <map>

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
    // -6- Move random task to other L3 cluster.
    //======================================================================
    struct MapTrMoveTaskToOtherL3Cluster : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        if (wlConfig.running_tasks.size() < 1) return false;

        // 1. Choose a random task, identify its cluster and # of active threads
        int core_idx = RandInt(mConf.coreCnt);
        while (mConf.map[core_idx] == MapConf::IDX_UNASSIGNED)
          core_idx = RandInt(mConf.coreCnt);

        Task * srcTask = wlConfig.GetThreadByGid(mConf.map[core_idx])->task;
        int srcClIdx = cmpConfig.GetMemory(core_idx)->L3ClusterIdx();
        IdxArray& srcTiles = cmpConfig.GetTilesOfL3Cluster(srcClIdx);

        int srcActiveDop = 0;
        for (IdxCIter tile_it = srcTiles.begin(); tile_it != srcTiles.end(); ++tile_it) {
          int th_gid = mConf.map[*tile_it];
          if (th_gid != MapConf::IDX_UNASSIGNED &&
              wlConfig.GetThreadByGid(th_gid)->task->task_id == srcTask->task_id) {
            ++srcActiveDop;
          }
        }

        // 2. Choose another cluster randomly
        int dstClIdx = RandInt(cmpConfig.L3ClusterCnt());
        while (srcClIdx == dstClIdx)
          dstClIdx = RandInt(cmpConfig.L3ClusterCnt());

        // 3. Check if dst cluster has enough free procs available to move the task
        IdxArray& dstTiles = cmpConfig.GetTilesOfL3Cluster(dstClIdx);
        int dstFreeProcCnt = 0;
        for (IdxCIter tile_it = dstTiles.begin(); tile_it != dstTiles.end(); ++tile_it) {
          if (mConf.map[*tile_it] == MapConf::IDX_UNASSIGNED) {
            ++dstFreeProcCnt;
          }
        }

        // task can be moved to free processors
        if (dstFreeProcCnt >= srcActiveDop) {
          // now move the task to the L3 cluster given by dstClIdx

          // remove old assignment
          IdxArray srcTaskThreads;
          for (IdxCIter tile_it = srcTiles.begin(); tile_it != srcTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid != MapConf::IDX_UNASSIGNED &&
                wlConfig.GetThreadByGid(th_gid)->task->task_id == srcTask->task_id) {
              mConf.map[*tile_it] = MapConf::IDX_UNASSIGNED;
              srcTaskThreads.push_back(th_gid);
            }
          }

          // do new assignment
          IdxCIter th_gid_iter = srcTaskThreads.begin();
          for (IdxCIter tile_it = dstTiles.begin(); tile_it != dstTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid == MapConf::IDX_UNASSIGNED) {
              mConf.map[*tile_it] = *th_gid_iter;
              if (++th_gid_iter == srcTaskThreads.end()) break;
            }
          }

          //cout << "         moved task to FREE procs, task "
          //     << srcTask->task_id << ", actDop " << srcActiveDop << endl;
          return true;
        }

        // 4. If couldn't move the task to free procs,
        //    look for a replacement task with the same # of active threads the dst cluster
        map<int, int> activeThreadCntPerTask;
        for (IdxCIter tile_it = dstTiles.begin(); tile_it != dstTiles.end(); ++tile_it) {
          int th_gid = mConf.map[*tile_it];
          if (th_gid != MapConf::IDX_UNASSIGNED) {
            Task * curTask = wlConfig.GetThreadByGid(th_gid)->task;
            if (activeThreadCntPerTask.find(curTask->task_id) ==
                activeThreadCntPerTask.end()) {
              activeThreadCntPerTask[curTask->task_id] = 1;
            }
            else {
              activeThreadCntPerTask[curTask->task_id] += 1;
            }
          }
        }

        Task * dstTask = 0;
        for(map<int,int>::const_iterator it = activeThreadCntPerTask.begin();
            it != activeThreadCntPerTask.end(); ++it) {
          if (it->second == srcActiveDop) {
            dstTask = wlConfig.GetTask(it->first); break;
          }
        }

        // replacement task (dstTask) was found, swap with src task
        if (dstTask) {

          /// // save old mapping state
          /// vector<int> oldMap = mConf.map;

          // remove old assignments
          // src Task
          IdxArray srcTaskThreads;
          for (IdxCIter tile_it = srcTiles.begin(); tile_it != srcTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid != MapConf::IDX_UNASSIGNED &&
                wlConfig.GetThreadByGid(th_gid)->task->task_id == srcTask->task_id) {
              mConf.map[*tile_it] = MapConf::IDX_UNASSIGNED;
              srcTaskThreads.push_back(th_gid);
            }
          }
          // dst Task
          IdxArray dstTaskThreads;
          for (IdxCIter tile_it = dstTiles.begin(); tile_it != dstTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid != MapConf::IDX_UNASSIGNED &&
                wlConfig.GetThreadByGid(th_gid)->task->task_id == dstTask->task_id) {
              mConf.map[*tile_it] = MapConf::IDX_UNASSIGNED;
              dstTaskThreads.push_back(th_gid);
            }
          }

          // do new assignments
          // src Task
          IdxCIter th_gid_iter = srcTaskThreads.begin();
          for (IdxCIter tile_it = dstTiles.begin(); tile_it != dstTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid == MapConf::IDX_UNASSIGNED) {
              mConf.map[*tile_it] = *th_gid_iter;
              if (++th_gid_iter == srcTaskThreads.end()) break;
            }
          }
          // dst Task
          th_gid_iter = dstTaskThreads.begin();
          for (IdxCIter tile_it = srcTiles.begin(); tile_it != srcTiles.end(); ++tile_it) {
            int th_gid = mConf.map[*tile_it];
            if (th_gid == MapConf::IDX_UNASSIGNED) {
              mConf.map[*tile_it] = *th_gid_iter;
              if (++th_gid_iter == dstTaskThreads.end()) break;
            }
          }

          /// vector<int> oldMapSorted = oldMap;
          /// vector<int> newMapSorted = mConf.map;
          /// sort(oldMapSorted.begin(), oldMapSorted.end());
          /// sort(newMapSorted.begin(), newMapSorted.end());
          /// if (oldMapSorted != newMapSorted) {
          ///   cout << "        swapped task " << srcTask->task_id << ", actDop " << srcActiveDop
          ///        << " with task " << dstTask->task_id << endl;
          ///   cout << "Oldmap:";
          ///   for (int i = 0; i < oldMap.size(); ++i) cout << ' ' << oldMap[i]; cout << endl;
          ///   cout << "Newmap:";
          ///   for (int i = 0; i < mConf.map.size(); ++i) cout << ' ' << mConf.map[i]; cout << endl;
          ///   exit(1);
          /// }

          return true;
        }

        //cout << "         failed to move task " << srcTask->task_id << ", actDop "
        //     << srcActiveDop << endl;
        return false;
      }
    };

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPTRANSFORM_H_
