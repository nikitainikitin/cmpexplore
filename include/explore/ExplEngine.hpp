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

#ifndef _ENGINE_EXPLENGINE_H_
#define _ENGINE_EXPLENGINE_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace arch {
    class ArchPlanner;
  }

  namespace explore {

    class Transform;
    class ExplConf;

    //======================================================================
    // ExplEngine represents an interface for exploration engines.
    //======================================================================

    class ExplEngine {

    public:

      typedef vector<Transform*> TrArray;

      typedef TrArray::iterator TrIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ExplEngine ();

      virtual ~ExplEngine ();

      // Accessors
      inline void AddTransform(Transform * t);

      inline const TrArray& Transforms() const;

      // Main method that invokes the exploration.
      virtual void Explore() = 0;

      // Service functions
      double GetAreaOverhead(arch::ArchPlanner& ap) const;

      ExplConf* GetCurConfigCost(arch::ArchPlanner& ap, double lambda) const;

    private:

      // Deprecated methods: prevent usage
      ExplEngine ( const ExplEngine& );

      ExplEngine& operator = ( const ExplEngine& );

      // -------------------------- Attributes -----------------------------

    private:

      TrArray transforms_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    void ExplEngine::AddTransform(Transform * t) {
      transforms_.push_back(t);
    }

    const ExplEngine::TrArray& ExplEngine::Transforms() const {
      return transforms_;
    }

  } // namespace explore

} // namespace cmpex

#endif // _ENGINE_EXPLENGINE_H_
