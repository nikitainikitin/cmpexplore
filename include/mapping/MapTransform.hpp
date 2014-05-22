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

#ifndef _MAPPING_MAPTRANSFORM_H_
#define _MAPPING_MAPTRANSFORM_H_

#include <string>
#include <vector>

#include "Config.hpp"
#include "mapping/MapConf.hpp"
#include "TechDefs.hpp"

using std::string;
using std::vector;

namespace cmpex {

  extern Config config;

  namespace mapping {

    //======================================================================
    // MapTransform represents the interface for transformations used by
    // metaheuristical mapping engines.
    //======================================================================

    struct MapTransform {

      // ---------------------------- Methods ------------------------------

      // Destructor
      virtual ~MapTransform () {}
      
      // Main method that transforms the mapping.
      // Returns true if applied successfully.
      virtual bool UpdateMap(MapConf& mConf) const = 0;

    };


    //======================================================================
    // -1- Swap the workload of two random processors.
    // Note: this transformation currently updates the mapping only.
    //======================================================================
    struct MapTrSwapTaskPair : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        // generate two different core indices
        int core1_idx = int(RandUDouble()*mConf.coreCnt);
        int core2_idx = int(RandUDouble()*mConf.coreCnt);
        while (core1_idx == core2_idx)
          core2_idx = int(RandUDouble()*mConf.coreCnt);

        std::swap(mConf.map[core1_idx], mConf.map[core2_idx]);
        return true;
      }
    };

    //======================================================================
    // -2- Change activity of a random core.
    //======================================================================
    struct MapTrChangeCoreActiv : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = int(RandUDouble()*mConf.coreCnt);
        // inverse activity
        mConf.coreActiv[core_idx] = !mConf.coreActiv[core_idx];
        return true;
      }
    };

    //======================================================================
    // -3- Increase frequency of a random core.
    //======================================================================
    struct MapTrIncreaseCoreFreq : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = int(RandUDouble()*mConf.coreCnt);
        if (mConf.coreFreq[core_idx] <= MAX_FREQ - FREQ_STEP) {
          mConf.coreFreq[core_idx] += FREQ_STEP;
        }
        return true;
      }
    };

    //======================================================================
    // -4- Decrease frequency of a random core.
    //======================================================================
    struct MapTrDecreaseCoreFreq : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int core_idx = int(RandUDouble()*mConf.coreCnt);
        if (mConf.coreFreq[core_idx] >= 2*FREQ_STEP) {
          mConf.coreFreq[core_idx] -= FREQ_STEP;
        }
        return true;
      }
    };

    //======================================================================
    // -5- Change activity of a random L3 cluster.
    //======================================================================
    struct MapTrChangeL3ClusterActiv : public MapTransform {
      bool UpdateMap(MapConf& mConf) const {
        int cluster_idx = int(RandUDouble()*mConf.L3ClusterCnt);
        // inverse activity
        mConf.L3ClusterActiv[cluster_idx] = !mConf.L3ClusterActiv[cluster_idx];
        return true;
      }
    };

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPTRANSFORM_H_
