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

#ifndef _MAPPING_MAPSIM_H_
#define _MAPPING_MAPSIM_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace mapping {

    //======================================================================
    // MapSim is a mapping simulator, which emulates the behavior of a CMP.
    // It performs the following actions at the beginning of every period:
    // - Checks for completed tasks and starts the new tasks
    // - Remaps tasks and selects V/F to fit the CMP budgets and maximize IPC
    // - Reevaluates system state by running analytical models
    // - Advances the progress of each task
    //======================================================================

    class MapSim {

    public:

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      MapSim (int period_us);

      virtual ~MapSim ();

      // Accessors
      inline int PeriodUs() const;

      // Main method that invokes simulation.
      virtual void Run();

      bool SkipRemapping(bool lastPeriodWlChanged);

      // --- Service functions ---

    private:

      // Deprecated methods: prevent usage
      MapSim ( const MapSim& );

      MapSim& operator = ( const MapSim& );

      // -------------------------- Attributes -----------------------------

    private:

      int period_; // duration of remapping period in us

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    int MapSim::PeriodUs() const {
       return period_;
    }

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_MAPSIM_H_
