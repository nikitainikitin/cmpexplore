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

    typedef std::vector<double> CoreStateArray;

    // reserved value for unassigned indices
    static const int IDX_UNASSIGNED = -1;

    // ---------------------------- Methods ------------------------------

  public:

    MapConf ( int cc, double t = 0, double p = 0, double tmp = 0, double c = 0 );

    ~MapConf ();

    // --- Service functions ---

    inline int GetUnassignedProcCnt() const;

    bool AssignToFreeProc (int th_gid);

    void CopyTo (MapConf * target) const;

    void Print() const;

  public:

    int coreCnt;

    CoresToThreadsMap map;

    CoreStateArray states;

    // mapping evaluation data

    double thr;

    double power;

    double temp;

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
