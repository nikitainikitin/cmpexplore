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

#ifndef _PERF_ITERATIVEPERFMODEL_H_
#define _PERF_ITERATIVEPERFMODEL_H_

#include <string>
#include <vector>

#include "CmpAnalytPerfModel.hpp"

namespace cmpex {

  namespace cmp {
    class Cluster;
  }
  
  namespace perf {

    //======================================================================
    // This class represents CMP performance analytical model based on
    // fixed-point iteration and bisection methods.
    //======================================================================

    class IterativePerfModel : public perf::CmpAnalytPerfModel {
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Constructors and destructor
      IterativePerfModel ();

      virtual ~IterativePerfModel ();
      
      // Implementation of the model interface
      
      stat::StatMetrics * Run();
      
      // Service functions

      stat::StatMetrics * RunFixedPoint(double statThr = 0, double statInj = 0);

      stat::StatMetrics * RunBisectionFp();

      stat::StatMetrics * RunSubgradient();

      stat::StatMetrics * RunSubgradientFp();

      void MarkPaths (const std::vector<double>& procRates);

      void MarkPathsNoCC (const std::vector<double>& procRates);

      void MarkPathsCC (const std::vector<double>& procRates);

      void InitModels ();

      bool EstimateDelays (bool fixNegDelays = false);

    private:

      // Deprecated methods: prevent usage
      IterativePerfModel ( const IterativePerfModel& );

      IterativePerfModel& operator = ( const IterativePerfModel& );

      // -------------------------- Attributes -----------------------------

    private:
      
    };

  } // namespace perf
  
} // namespace cmpex

#endif // _PERF_ITERATIVEPERFMODEL_H_
