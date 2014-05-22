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

#include "mapping/MapConf.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include "cmp/Processor.hpp"
#include "TechDefs.hpp"

using namespace std;

namespace cmpex {

  extern Config config;
  extern cmp::CmpConfig cmpConfig;

  using namespace cmp;

  namespace mapping {

//=======================================================================
/*
 * Constructors and destructor
 */

MapConf::MapConf (int core_cnt, int l3_cl_cnt,
                  double t, double p, double tmp, double c):
  coreCnt (core_cnt), L3ClusterCnt (l3_cl_cnt), thr (t), power(p), temp(tmp), cost (c)
{
  map.assign(coreCnt, IDX_UNASSIGNED);
  coreActiv.assign(coreCnt, true);
  coreFreq.assign(coreCnt, MAX_FREQ);
  L3ClusterActiv.assign(L3ClusterCnt, true);
}

MapConf::~MapConf () {}

//=======================================================================
/*
 * Assign specified thread to the next free processor.
 * Return true if thread has been assigned successfully.
 */

bool MapConf::AssignToFreeProc (int th_gid) {
  for (int p = 0; p < map.size(); ++p) {
    if (map[p] == IDX_UNASSIGNED) {
      map[p] = th_gid;
      coreActiv[p] = true;
      coreFreq[p] = cmpConfig.GetProcessor(p)->Freq();
      return true;
    }
  }
  return false;
}

//=======================================================================
/*
 * Copy mapping configuration to the target object
 */

void MapConf::CopyTo (MapConf * target) const {
  target->coreCnt = coreCnt;
  target->L3ClusterCnt = L3ClusterCnt;
  target->map = map;
  target->coreActiv = coreActiv;
  target->coreFreq = coreFreq;
  target->L3ClusterActiv = L3ClusterActiv;
  target->thr = thr;
  target->power = power;
  target->temp = temp;
  target->cost = cost;
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
       << "; Thr=" << thr << endl;
}

//=======================================================================

  } //namespace mapping

} //namespace cmpex
