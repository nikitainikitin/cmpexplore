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

using namespace std;

namespace cmpex {

  extern Config config;

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
