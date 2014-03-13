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

#ifndef _PERF_MATLABPERFMODEL_H_
#define _PERF_MATLABPERFMODEL_H_

#include <string>
#include <vector>

#include "CmpAnalytPerfModel.hpp"

namespace cmpex {
  
  namespace perf {

    //======================================================================
    // This class dumps system of equations in Matlab format for CMP
    // performance modeling.
    //======================================================================

    class MatlabPerfModel : public perf::CmpAnalytPerfModel {
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Constructors and destructor
      MatlabPerfModel ();

      virtual ~MatlabPerfModel ();
      
      // Implementation of the model interface
      
      stat::StatMetrics * Run();
      
      // Service functions
      
      stat::StatMetrics * RunMatlab();

    private:

      // Deprecated methods: prevent usage
      MatlabPerfModel ( const MatlabPerfModel& );

      MatlabPerfModel& operator = ( const MatlabPerfModel& );

      // -------------------------- Attributes -----------------------------

    private:
      
    };

  } // namespace perf
  
} // namespace cmpex

#endif // _PERF_MATLABPERFMODEL_H_
