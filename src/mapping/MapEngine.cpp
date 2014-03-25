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
#include <string>
#include <limits>
#include <fstream>
#include <iomanip>

#include "MapEngine.hpp"
#include "MapConf.hpp"
#include "MapTransform.hpp"
#include "Config.hpp"
#include "cmp/CmpConfig.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "phys/PhysicalModel.hpp"
#include "power/PowerModel.hpp"
#include "RouterDefs.hpp"

using namespace std;

namespace cmpex {

  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern stat::Statistics stats;

  using namespace cmp;
  using namespace stat;
  using namespace perf;
  using namespace phys;
  using namespace power;

  namespace mapping {

//=======================================================================
/*
 * Constructors and destructor
 */

MapEngine::MapEngine() {
  // create transformations
  AddTransform(new MapTrSwapTaskPair());
  AddTransform(new MapTrChangeCoreState());
}

MapEngine::~MapEngine() {
  for (MapTrIter it = transforms_.begin(); it != transforms_.end(); ++it)
    delete *it;
}

//=======================================================================
/*
 * Evaluates cost of the provided mapping solution.
 */

void MapEngine::EvalMappingCost(MapConf& mc, double lambda) const
{

}

//=======================================================================

  } // namespace mapping

} //namespace cmpex
