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

#ifndef _MAPPING_SAMAPENGINE_H_
#define _MAPPING_SAMAPENGINE_H_

#include "MapEngine.hpp"

namespace cmpex {

  namespace mapping {

    //======================================================================
    // SaMapEngine is simulated annealing-based mapping for cmp.
    //======================================================================

    class SaMapEngine : public MapEngine {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      SaMapEngine ();

      virtual ~SaMapEngine ();
      
      // Main method that invokes the mapping.
      void Map(MapConf * mconf = 0);

    private:

      // Deprecated methods: prevent usage
      SaMapEngine ( const SaMapEngine& );

      SaMapEngine& operator = ( const SaMapEngine& );

      // -------------------------- Attributes -----------------------------

    private:

    };

  } // namespace mapping

} // namespace cmpex

#endif // _MAPPING_SAMAPENGINE_H_
