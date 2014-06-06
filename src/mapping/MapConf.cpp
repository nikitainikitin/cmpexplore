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
#include <algorithm>

#include "mapping/MapConf.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include "cmp/Processor.hpp"
#include "TechDefs.hpp"
#include "workload/WlConfig.hpp"

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

//=======================================================================
/*
 * Constructors and destructor
 */

MapConf::MapConf (int core_cnt, int l3_cl_cnt,
                  double t, double p, double tmp):
  coreCnt (core_cnt), L3ClusterCnt (l3_cl_cnt), thr (t), power(p), temp(tmp),
  obj (0.0), cost (0.0)
{
  map.assign(coreCnt, IDX_UNASSIGNED);
  coreActiv.assign(coreCnt, false);
  coreFreq.assign(coreCnt, MIN_FREQ);
  L3ClusterActiv.assign(L3ClusterCnt, false);
}

MapConf::~MapConf () {}

//=======================================================================
/*
 * Assign specified thread to the next free processor.
 * Return true if thread has been assigned successfully.
 */

bool MapConf::AssignThreadToFreeProc (int th_gid) {
  for (int p = 0; p < map.size(); ++p) {
    if (map[p] == IDX_UNASSIGNED) {
      map[p] = th_gid;
      //coreActiv[p] = true;
      //coreFreq[p] = cmpConfig.GetProcessor(p)->Freq();
      return true;
    }
  }
  return false;
}

//=======================================================================
/*
 * Assign specified task to the free processors.
 * Assumption: threads of the task have to be assigned to one L3 cluster.
 * Return true if task has been assigned successfully.
 */

bool MapConf::AssignTaskToFreeProcs (int t_id) {
  // iterate over L3 clusters,
  // check how many procs available in this cluster
  vector<int> freeProcsPerL3Cluster;
  for (int cl = 0; cl < cmpConfig.L3ClusterCnt(); ++cl) {
    IdxArray& tiles = cmpConfig.GetTilesOfL3Cluster(cl);
    int freeCnt = 0;
    for (IdxCIter tile_it = tiles.begin(); tile_it != tiles.end(); ++tile_it) {
      if (map[*tile_it] == IDX_UNASSIGNED) {
        ++freeCnt;
      }
    }
    freeProcsPerL3Cluster.push_back(freeCnt);
  }

  // find cluster with the max number of free procs
  int maxClIdx = max_element(freeProcsPerL3Cluster.begin(), freeProcsPerL3Cluster.end()) -
                 freeProcsPerL3Cluster.begin();

  Task * task = wlConfig.GetTask(t_id);

  // not enough procs in any of the clusters
  if (freeProcsPerL3Cluster[maxClIdx] < task->task_dop) return false;

  // iterator to the list of tiles in the max cluster
  IdxCIter tile_it = cmpConfig.GetTilesOfL3Cluster(maxClIdx).begin();
  for (int thread = 0; thread < task->task_dop; ++thread) {
    // global id of next thread to be assigned
    int th_gid = task->task_threads[thread]->thread_gid;
    // find the next free proc
    while (map[*tile_it] != IDX_UNASSIGNED) ++tile_it;
    map[*tile_it] = th_gid;
    ++tile_it;
  }

  return true;
}

//=======================================================================
/*
 * Print mapping
 */

void MapConf::Print () const {
  cout << "Map:";
  for (CoresToThreadsMap::const_iterator it = map.begin(); it != map.end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << "; CoreActiv:";
  for (BoolArray::const_iterator it = coreActiv.begin(); it != coreActiv.end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << "; CoreFreq:";
  for (DoubleArray::const_iterator it = coreFreq.begin(); it != coreFreq.end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << "; L3ClusterActiv:";
  for (BoolArray::const_iterator it = L3ClusterActiv.begin(); it != L3ClusterActiv.end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << ";   Pow=" << power << "; Temp=" << temp
       << "; Thr=" << thr << "; Obj=" << obj << "; Cost=" << cost << endl;
}

//=======================================================================

  } //namespace mapping

} //namespace cmpex
