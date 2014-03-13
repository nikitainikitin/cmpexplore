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

#ifndef _EXPLORE_HCENGINE_H_
#define _EXPLORE_HCENGINE_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace arch {
    class ArchPlanner;
  }

  namespace explore {

    //======================================================================
    // HcEngine is hill climbing-based exploration for cmp.
    //======================================================================

    class HcEngine {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      HcEngine ();

      virtual ~HcEngine ();
      
      // Main method that invokes the exploration.
      void Explore() const;

      double GetAreaOverhead(arch::ArchPlanner& ap) const;
      
    private:

      // Deprecated methods: prevent usage
      HcEngine ( const HcEngine& );

      HcEngine& operator = ( const HcEngine& );

      // -------------------------- Attributes -----------------------------

    private:

    };

  } // namespace explore

} // namespace cmpex

#endif // _EXPLORE_HCENGINE_H_
