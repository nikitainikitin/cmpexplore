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

#ifndef _PHYS_PHYSICALMODEL_H_
#define _PHYS_PHYSICALMODEL_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace cmp {
    class Component;
  }

  namespace arch {
    class ArchConfig;
  }

  namespace phys {

    //======================================================================
    // PhysicalModel is used to model different physical aspects
    // of a CMP configuration (i.e. routability).
    //======================================================================

    class PhysicalModel {

      // ---------------------------- Methods ------------------------------

    public:

      // Checks routability of the configuration
      static bool IsRoutable(arch::ArchConfig * pAc);

      // Returns the difference between the routable area and
      // routability requirement
      static double GetRoutabilitySlack(arch::ArchConfig * pAc);

      // Calculates the total routable area on chip
      static double GetRoutableArea(arch::ArchConfig * pAc);

      // Calculates the area required for routability on chip
      static double GetRoutableAreaReq(arch::ArchConfig * pAc);

      // Calculates area of a given configuration
      static double GetCmpArea(cmp::Component * cmp);

    public:

      // Constructors & destructor
      PhysicalModel ();

      virtual ~PhysicalModel ();
      
    private:

      // Deprecated methods: prevent usage
      PhysicalModel ( const PhysicalModel& );

      PhysicalModel& operator = ( const PhysicalModel& );

      // -------------------------- Attributes -----------------------------

    private:

    };

  } // namespace phys

} // namespace cmpex

#endif // _PHYS_PHYSICALMODEL_H_
