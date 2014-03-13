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

#ifndef _PERF_CMPANALYTPERFMODEL_H_
#define _PERF_CMPANALYTPERFMODEL_H_

#include <string>
#include <vector>

#include "Defs.hpp"

using std::vector;

namespace cmpex {

  namespace stat {
    class StatMetrics;
  }
  
  namespace perf {

    //======================================================================
    // This class represents an interface for CMP performance analytical models.
    //======================================================================

    class CmpAnalytPerfModel {

    protected:
      
      typedef std::vector<double> ProbArray;
      
      typedef ProbArray::const_iterator PCIter;
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Constructors and destructor
      CmpAnalytPerfModel ();

      virtual ~CmpAnalytPerfModel ();

      inline double CalcThr (double ipc, double mpi, double lat) const;
      
      // Interface
      
      virtual stat::StatMetrics * Run() = 0;
      
      // Service functions
      
      // Calculates average latency for processor 'idx'.
      double CalculateProcLatency (UShort idx, bool dynamic = false);
      
      // Calculates average latency for processor 'idx'. No CC is considered.
      double CalculateProcLatencyNoCC (UShort idx, bool dynamic = false);

      // Calculates average latency for processor 'idx'. CC is considered.
      double CalculateProcLatencyCC (UShort idx, bool dynamic = false);

      // Returns latency for processor 'pIdx' to access the memory controller
      // with index 'mcIdx'. No CC is considered.
      double LatencyProcToMemCtrl (UShort pIdx, UShort mcIdx, bool dynamic = false);

      // Returns latency for processor 'pIdx' to access the memory
      // with index 'mIdx'. No CC is considered.
      double LatencyProcToMem (UShort pIdx, UShort mIdx, bool dynamic = false);

      // Precalculate latencies from L3 to MC (for cache coherence only)
      void PrecalcL3ToMcLatencies(bool dynamic = false);

      // Debug methods
      virtual void Print () const;

    protected:

      inline vector<double>& L3ToMcLat();

    private:

      // Deprecated methods: prevent usage
      CmpAnalytPerfModel ( const CmpAnalytPerfModel& );

      CmpAnalytPerfModel& operator = ( const CmpAnalytPerfModel& );

      // -------------------------- Attributes -----------------------------

    private:

      //  --- Runtime storages ---

      vector<double> l3ToMcLat_; // temporal storage for precalculated latencies
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    double CmpAnalytPerfModel::CalcThr (double ipc, double mpi, double lat) const {
      // cycles for executing LOAD instruction are included into the lat
      //return ipc / (1.0 + (lat*ipc-1.0)*mpi);
      /// cycles for executing LOAD instruction are NOT included into the lat
      return 1.0 / (1.0/ipc + lat*mpi);
    }

    vector<double>& CmpAnalytPerfModel::L3ToMcLat() {
      return l3ToMcLat_;
    }

  } // namespace perf
  
} // namespace cmpex

#endif // _PERF_CMPANALYTPERFMODEL_H_
