// ----------------------------------------------------------------------
//   Copyright 2011-2012 Nikita Nikitin <nikita.i.nikitin@gmail.com>
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
#include <numeric>

#include "StatConfig.hpp"
#include "ArchConfig.hpp"
#include "Defs.hpp"

using namespace std;

namespace cmpex {

  using arch::ArchConfig;

  namespace stat {

//=======================================================================
/*
 * Constructors and destructor
 */

StatConfig::StatConfig(ArchConfig * ac) :
  config_(ac) {}

StatConfig::~StatConfig()
{
  if (config_) delete config_;

  for (MVCIter it = metrics_.begin(); it != metrics_.end(); ++it) {
    delete *it;
  }
}

//=======================================================================
/*
 * Aggregate metrics, manage the aggregate objective function.
 */

double StatConfig::AggThroughput() const {

  vector<double> thrs;
  for (MVCIter it = metrics_.begin(); it != metrics_.end(); ++it) {
    thrs.push_back((*it)->Throughput());
  }

  return AggWlObjective(thrs);
}

double StatConfig::AggLatency() const {

  vector<double> lats;
  for (MVCIter it = metrics_.begin(); it != metrics_.end(); ++it) {
    lats.push_back((*it)->Latency());
  }

  return AggWlObjective(lats);
}

double StatConfig::AggPower() const {

  vector<double> pows;
  for (MVCIter it = metrics_.begin(); it != metrics_.end(); ++it) {
    pows.push_back((*it)->Power());
  }

  return AggWlObjective(pows);
}

//=======================================================================
/*
 * Defines a generic aggregate objective function over the workloads,
 * which can be used with the above metrics.
 * Calculates aggregate value for the given list of values.
 */

double StatConfig::AggWlObjective (vector<double>& values) const {
  // Average
  double avg = accumulate(values.begin(), values.end(), 0.0);
  return avg/values.size();
}

//=======================================================================

  } // namespace stat

} //namespace cmpex
