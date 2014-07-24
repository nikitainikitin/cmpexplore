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

#ifndef _MAPPING_MAPENGINE_H_
#define _MAPPING_MAPENGINE_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace cmp {
    class Processor;
  }

  namespace mapping {

    class MapTransform;
    class MapConf;

    //======================================================================
    // MapEngine represents the interface for mapping engines.
    //======================================================================

    class MapEngine {

    public:

      typedef vector<MapTransform*> MapTrArray;

      typedef MapTrArray::iterator MapTrIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      MapEngine ();

      virtual ~MapEngine ();

      // Accessors
      inline void AddTransform(MapTransform * t);

      inline const MapTrArray& Transforms() const;

      // Main method that invokes mapping.
      virtual void Map(MapConf * mconf, MapConf * prevMap,
                       const vector<double>& prevProcThr, bool silent_mode = false) = 0;

      // --- Service functions ---

      // Create a greedy mapping solution
      MapConf * CreateGreedyMapping () const;

      double CalcQoSObjPenalty(MapConf * mc) const;

      double CalcNewQoSObjPenalty(MapConf * mc, const vector<double>& prevProcThr) const;

      double CalcTempPenalty(double lambda = 1.0) const;

      double GetTemperature() const;

      void ChooseActiveCores(MapConf * mc, MapConf * prevMap,
                             const vector<double>& prevProcThr) const;

      double MapCorePriorityToFreq ( double core_priority ) const;

      double CalcMinReqFreq ( cmp::Processor * proc, double req_thr_ipns,
                                                     double wc_cpi_thread ) const;

      // Evaluate cost of the provided mapping
      void EvalMappingCost(MapConf * mc, MapConf * prevMap,
                           const vector<double>& prevProcThr, double lambda = 1.0) const;

    private:

      // Deprecated methods: prevent usage
      MapEngine ( const MapEngine& );

      MapEngine& operator = ( const MapEngine& );

      // -------------------------- Attributes -----------------------------

    private:

      MapTrArray transforms_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    void MapEngine::AddTransform(MapTransform * t) {
      transforms_.push_back(t);
    }

    const MapEngine::MapTrArray& MapEngine::Transforms() const {
      return transforms_;
    }

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPENGINE_H_
