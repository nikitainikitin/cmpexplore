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

MapConf::MapConf (int cc, double t, double p, double tmp, double c):
  coreCnt (cc), thr (t), power(p), temp(tmp), cost (c)
{
  map.assign(coreCnt, IDX_UNASSIGNED);
  states.assign(coreCnt, 0.0);
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
      states[p] = cmpConfig.GetProcessor(p)->Freq();
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
  target->thr = thr;
  target->power = power;
  target->temp = temp;
  target->cost = cost;
  target->map.assign(map.begin(), map.end());
  target->states.assign(states.begin(), states.end());
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
  cout << "; States:";
  for (CoreStateArray::const_iterator it = states.begin(); it != states.end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << ";   Pow=" << power << "; Temp=" << temp
       << "; Thr=" << thr << endl;
}

//=======================================================================

  } //namespace mapping

} //namespace cmpex
