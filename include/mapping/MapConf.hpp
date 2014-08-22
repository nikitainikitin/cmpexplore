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

#ifndef _MAPPING_MAPCONF_H_
#define _MAPPING_MAPCONF_H_

#include <iostream>
#include <vector>
#include <algorithm>

namespace cmpex {

  namespace mapping {

  //======================================================================
  // MapConf represents a mapping solution.
  // It consists of an array of mapping variables between the indices
  // of cores in a CMP architecture and the indices of workload threads.
  // It also stores the core states, i.e. on/off, frequency, etc.
  //======================================================================

  struct MapConf
  {
  public:

    typedef std::vector<int> CoresToThreadsMap;

    typedef std::vector<double> DoubleArray;

    typedef std::vector<bool> BoolArray;

    // reserved value for unassigned indices
    static const int IDX_UNASSIGNED = -1;

    // ---------------------------- Methods ------------------------------

  public:

    MapConf ( int core_cnt, int l3_cl_cnt,
              double t = 0, double p = 0, double tmp = 0 );

    ~MapConf ();

    bool operator == (const MapConf&);

    // --- Service functions ---

    inline int GetUnassignedProcCnt() const;

    bool AssignThreadToFreeProc (int th_gid);

    bool AssignTaskToFreeProcs (int t_id);

    void Print() const;

  public:

    int coreCnt;

    int L3ClusterCnt;

    CoresToThreadsMap map;

    BoolArray coreActiv; // acitivities of the cores (on/off)

    DoubleArray coreFreq; // frequencies of the cores [GHz]

    BoolArray L3ClusterActiv; // acitivities of the cache clusters (on/off)

    double uncoreFreq; // frequency of the uncore [GHz]

    // mapping evaluation data

    double thr;

    double power;

    double temp;

    // Original optimization objective, weighted by the soft constraint penalties.
    // This is the final optimization objective of mapping.
    double obj;

    // Optimization objective penalized by the hard constraints.
    // This is the internal mapping objective, which incorporates
    // hard constraints.
    double cost;

  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int MapConf::GetUnassignedProcCnt() const {
    return std::count(map.begin(), map.end(), IDX_UNASSIGNED);
  }


  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPCONF_H_
