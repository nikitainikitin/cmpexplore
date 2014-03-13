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

#ifndef _STAT_STATCONFIG_H_
#define _STAT_STATCONFIG_H_

#include <string>
#include <vector>
#include <sstream>
#include <map>

#include "Defs.hpp"

using std::string;
using std::vector;
using std::stringstream;
using std::istream;
using std::map;
using std::pair;

namespace cmpex {

  namespace arch {
    class ArchConfig;
  }

  namespace stat {

    //======================================================================
    // StatMetrics is a plain structure encapsulating the metrics of interest
    // for configuration obtained by simulation/modeling (area, throughput, etc).
    //======================================================================

    struct StatMetrics {

      // ---------------------------- Methods ------------------------------

      // Constructors & destructor
      StatMetrics(double thr, double lat = 0.0, double tr = 0.0,
                  double pow = 0.0, double rout = 0.0) :
        throughput(thr), latency(lat), traffic (tr), power(pow), routability (rout),
        stat_throughput(0.0), stat_traffic(0.0) {}

      // Accessors (for consistency)
      inline double Throughput() const { return throughput; }

      inline void Throughput(double t) { throughput = t; }

      inline double Latency() const { return latency; }

      inline void Latency(double l) { latency = l; }

      inline double Traffic() const { return traffic; }

      inline void Traffic(double t) { traffic = t; }

      inline double Power() const { return power; }

      inline void Power(double p) { power = p; }

      inline double Routability() const { return routability; }

      inline void Routability(double r) { routability = r; }

      inline double StatThroughput() const { return stat_throughput; }

      inline void StatThroughput(double t) { stat_throughput = t; }

      inline double StatTraffic() const { return stat_traffic; }

      inline void StatTraffic(double t) { stat_traffic = t; }

      // -------------------------- Attributes -----------------------------

      double throughput;

      double latency;

      double traffic;

      double power;

      double routability;

      double stat_throughput; // throughput obtained with static latency

      double stat_traffic; // traffic obtained with static latency

    };

    //======================================================================
    // StatConfig represents description of a cmp configuration for
    // statistical tracking. It includes the full stream-based cmp description
    // stored by ArchConfig, and metric values stored in StatMetrics,
    // for every workload defined in CmpConfig.workloads.
    // It owns the config and metric objects and deletes them upon destruction.
    //======================================================================

    class StatConfig {

    public:

      typedef vector<StatMetrics*> MetricVector;

      typedef MetricVector::const_iterator MVCIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      StatConfig (arch::ArchConfig * ac);

      virtual ~StatConfig ();
      
      // Accessors
      inline const arch::ArchConfig * Config () const;

      inline const StatMetrics * GetMetrics (int wlIdx) const;

      inline void AddMetrics (StatMetrics * sm);

      // Aggregate metrics over all workloads.
      // These functions manage the aggregate objective.
      double AggThroughput () const;

      double AggLatency () const;

      double AggPower () const;

      // Defines a generic aggregate objective function,
      // can be used for the above metrics
      double AggWlObjective (vector<double>& values) const;

    private:

      // Deprecated methods: prevent usage
      StatConfig ( const StatConfig& );

      StatConfig& operator = ( const StatConfig& );

      // -------------------------- Attributes -----------------------------

    private:

      arch::ArchConfig * config_;

      MetricVector metrics_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const arch::ArchConfig * StatConfig::Config () const {
      return config_;
    }

    const StatMetrics * StatConfig::GetMetrics (int wlIdx) const {
      DASSERT(wlIdx < metrics_.size());
      return metrics_[wlIdx];
    }

    void StatConfig::AddMetrics (StatMetrics *sm) {
      metrics_.push_back(sm);
    }

  } // namespace stat

} // namespace cmpex

#endif // _STAT_STATCONFIG_H_
