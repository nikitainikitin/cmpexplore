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

#ifndef _POWER_POWERMODEL_H_
#define _POWER_POWERMODEL_H_

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace cmp {
    class Component;
    class Interconnect;
    class MeshIc;
    class BusIc;
    class URingIc;
    class BRingIc;
    class XBarIc;
  }

  namespace power {

    //======================================================================
    // PowerModel is used to estimate power of various components
    // of CMP architectures.
    //======================================================================

    class PowerModel {

      // ---------------------------- Methods ------------------------------

      // temporary buffers to store power values
      // used for dumping the power trace file;
      // indexed by the component idx (core, router, etc)

      static vector<double> corePower_;

      static vector<double> L1Power_;

      static vector<double> L2Power_;

      static vector<double> L3Power_;

      static vector<double> MCPower_;

      static vector<double> MeshRouterPower_;

      static vector<double> MeshLinkPower_; // 4 links per router

    public:

      // Calculates power of given configuration
      static double GetTotalPower(cmp::Component * cmp);

      // Calculates power of the cmp interconnect
      static double GetIcPower(cmp::Component * cmp);

      // Calculates power of flat interconnect
      static double GetFlatIcPower(cmp::Interconnect * ic);

      // IC power for different types
      static double MeshPower(cmp::MeshIc * ic);

      static double BusPower(cmp::BusIc * ic);

      static double URingPower(cmp::URingIc * ic);

      static double BRingPower(cmp::BRingIc * ic);

      static double XBarPower(cmp::XBarIc * ic);

      static double DumpPTsimPower(cmp::Component * cmp);

      static double VScalDynPower ( double volt );

      static double VScalLeakPower ( double volt );

    public:

      // Constructors & destructor
      PowerModel ();

      virtual ~PowerModel ();
      
    private:

      // Deprecated methods: prevent usage
      PowerModel ( const PowerModel& );

      PowerModel& operator = ( const PowerModel& );

      // -------------------------- Attributes -----------------------------

    private:

    };

  } // namespace power

} // namespace cmpex

#endif // _POWER_POWERMODEL_H_
