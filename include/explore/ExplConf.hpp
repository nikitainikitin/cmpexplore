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

#ifndef _EXPLORE_EXPLCONF_H_
#define _EXPLORE_EXPLCONF_H_

#include <iostream>

namespace cmpex {

  namespace arch {
    class ArchPlanner;
  }

  namespace explore {

  //======================================================================
  // ExplConf is a collection of data representing configurations
  // during the exploration.
  //======================================================================

  struct ExplConf
  {
    // ---------------------------- Methods ------------------------------

  public:

    ExplConf ( arch::ArchPlanner& ap,
               double a = 0, double t = 0, double p = 0, double c = 0 );

    ~ExplConf ();

    void SetAp(arch::ArchPlanner& ap) const;

    void Print() const;

  public: // configuration parameters

    int xIdx;

    int yIdx;

    int procCnt0;

    int l1_0_idx;

    int l2_0_idx;

    int procCnt1;

    int l1_1_idx;

    int l2_1_idx;

    int procCnt2;

    int l1_2_idx;

    int l2_2_idx;

    int clType;

    double area;

    double thr;

    double power;

    double cost;

  };

  } // namespace explore

} // namespace cmpex

#endif // _EXPLORE_EXPLCONF_H_
