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

#ifndef _TECHDEFS_H_
#define _TECHDEFS_H_

#include <cstdlib>
#include <iostream>

namespace cmpex {

  //======================================================================
  // This file contains technology-related definitions.
  //======================================================================

  enum Technology { TECH_32NM = 0, TECH_22NM, TECH_16NM, TECH_CNT };

  const double MAX_FREQ = 3.5; // Maximum on-chip frequency [GHz]

  const double MIN_FREQ = 0.5; // Minimum on-chip frequency [GHz]

  const double FREQ_STEP = 0.5; // Dynamic frequency scaling step [GHz]

} // namespace cmpex

#endif // _TECHDEFS_H_
