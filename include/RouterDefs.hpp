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

#ifndef _ROUTERDEFS_H_
#define _ROUTERDEFS_H_

#include <cstdlib>
#include <iostream>

#include "Defs.hpp"
#include "TechDefs.hpp"

namespace cmpex {

  //======================================================================
  // This file contains parameters for precharacterized routers.
  //======================================================================

  // 5x5 port router area [mm^2] for 16nm, estimated using Orion 2.0
  // BufSize = 8 flits
  // LinkWidth supported: 128, 256, 512, 1024
  const double ROUTER_AREA_16NM[] = {
    0.124, 0.251, 0.500, 0.990
  };

  const double ROUTER_AREA_22NM[] = {
    0.174, 0.351, 0.700, 1.390
  };

  const double ROUTER_AREA_32NM[] = {
    0.247, 0.491, 0.979, 1.956
  };

  // Service function to access router parameters by link width
  inline int _LinkWidthIdx (int linkWidth) {
    int idx = -1;
    switch (linkWidth) {
    case 128: idx = 0; break;
    case 256: idx = 1; break;
    case 512: idx = 2; break;
    case 1024: idx = 3; break;
    default:
      std::cout << "-E- Wrong link width (" << linkWidth
           << "): 128, 256, 512 or 1024 bits are permitted" << std::endl;
      exit(1);
    }
    return idx;
  }

  inline double RouterArea (Technology t, int linkWidth) {
    switch (t) {
      case TECH_32NM: return ROUTER_AREA_32NM[_LinkWidthIdx(linkWidth)]; break;
      case TECH_22NM: return ROUTER_AREA_22NM[_LinkWidthIdx(linkWidth)]; break;
      case TECH_16NM: return ROUTER_AREA_16NM[_LinkWidthIdx(linkWidth)]; break;
    }
  }

  // ============================= Router power ===============================
  // Router leakage power [W] and energy per flit [nJ] for 16nm, from Orion 2.0
  // BufSize = 8 flits
  // LinkWidth supported: 128, 256, 512, 1024

  // 10x10 routers
  const double ROUTER_PLEAK_10X10_16NM[] = {
    0.167, 0.34, 0.66, 1.30
  };

  const double ROUTER_EPF_10X10_16NM[] = {
    0.33e-2, 0.9e-2, 2.55e-2, 8.7e-2
  };

  // 9x9 routers
  const double ROUTER_PLEAK_9X9_16NM[] = {
    0.148, 0.30, 0.60, 1.23
  };

  const double ROUTER_EPF_9X9_16NM[] = {
    0.35e-2, 0.95e-2, 2.65e-2, 8.75e-2
  };

  // 8x8 routers
  const double ROUTER_PLEAK_8X8_16NM[] = {
    0.130, 0.26, 0.53, 1.15
  };

  const double ROUTER_EPF_8X8_16NM[] = {
    0.38e-2, 0.99e-2, 2.7e-2, 8.82e-2
  };

  // 7x7 routers
  const double ROUTER_PLEAK_7X7_16NM[] = {
    0.115, 0.23, 0.45, 0.90
  };

  const double ROUTER_EPF_7X7_16NM[] = {
    0.41e-2, 1.06e-2, 2.8e-2, 8.9e-2
  };

  // 6x6 routers
  const double ROUTER_PLEAK_6X6_16NM[] = {
    0.093, 0.19, 0.38, 0.75
  };

  const double ROUTER_EPF_6X6_16NM[] = {
    0.45e-2, 1.13e-2, 3.0e-2, 9.0e-2
  };

  // 5x5 routers
  const double ROUTER_PLEAK_5X5_16NM[] = {
    0.08, 0.16, 0.32, 0.64
  };

  const double ROUTER_EPF_5X5_16NM[] = {
    0.5e-2, 1.25e-2, 3.25e-2, 9.375e-2
  };
  
  // 4x4 routers
  const double ROUTER_PLEAK_4X4_16NM[] = {
    0.063, 0.126, 0.252, 0.504
  };

  const double ROUTER_EPF_4X4_16NM[] = {
    0.664e-2, 1.484e-2, 3.242e-2, 9.57e-2
  };
  
  // 3x3 routers
  const double ROUTER_PLEAK_3X3_16NM[] = {
    0.045, 0.09, 0.18, 0.36
  };

  const double ROUTER_EPF_3X3_16NM[] = {
    0.694e-2, 1.388e-2, 3.194e-2, 7.986e-2
  };
  
  // 2x2 routers
  const double ROUTER_PLEAK_2X2_16NM[] = {
    0.03, 0.06, 0.12, 0.24
  };

  const double ROUTER_EPF_2X2_16NM[] = {
    1.094e-2, 2.031e-2, 4.219e-2, 10.16e-2
  };

  inline double RouterPleak (Technology t, int portNum, int linkWidth) {
    DASSERT(t == TECH_16NM);
    DASSERT(portNum >= 2 && portNum <= 10);
    switch (portNum) {
      case 2: return ROUTER_PLEAK_2X2_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 3: return ROUTER_PLEAK_3X3_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 4: return ROUTER_PLEAK_4X4_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 5: return ROUTER_PLEAK_5X5_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 6: return ROUTER_PLEAK_6X6_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 7: return ROUTER_PLEAK_7X7_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 8: return ROUTER_PLEAK_8X8_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 9: return ROUTER_PLEAK_9X9_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 10: return ROUTER_PLEAK_10X10_16NM[_LinkWidthIdx(linkWidth)]; break;
    }
  }

  inline double RouterEpf (Technology t, int portNum, int linkWidth) {
    DASSERT(t == TECH_16NM);
    DASSERT(portNum >= 2 && portNum <= 10);
    switch (portNum) {
      case 2: return ROUTER_EPF_2X2_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 3: return ROUTER_EPF_3X3_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 4: return ROUTER_EPF_4X4_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 5: return ROUTER_EPF_5X5_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 6: return ROUTER_EPF_6X6_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 7: return ROUTER_EPF_7X7_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 8: return ROUTER_EPF_8X8_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 9: return ROUTER_EPF_9X9_16NM[_LinkWidthIdx(linkWidth)]; break;
      case 10: return ROUTER_EPF_10X10_16NM[_LinkWidthIdx(linkWidth)]; break;
    }
  }

  // ============================= Buffer power ===============================
  // Buffer leakage power [W] and energy per flit [nJ] for 16nm, from Orion 2.0
  // BufSize = 8 flits
  // LinkWidth supported: 128, 256, 512, 1024

  const double BUFFER_PLEAK_16NM[] = {
    0.004, 0.008, 0.015, 0.03
  };

  const double BUFFER_EPF_16NM[] = {
    0.406e-2, 0.813e-2, 1.625e-2, 3.25e-2
  };

  inline double BufferPleak (Technology t, int linkWidth) {
    DASSERT(t == TECH_16NM);
    return BUFFER_PLEAK_16NM[_LinkWidthIdx(linkWidth)];
  }

  inline double BufferEpf (Technology t, int linkWidth) {
    DASSERT(t == TECH_16NM);
    return BUFFER_EPF_16NM[_LinkWidthIdx(linkWidth)];
  }

  // ============================= Link power ===============================
  // Link leakage power [W] and energy per flit [nJ] for 16nm, from Orion 2.0
  // Link length = 1mm
  // LinkWidth supported: 128, 256, 512, 1024

  const double LINK_PLEAK_16NM[] = {
    0.0018, 0.0042, 0.007, 0.014
  };

  const double LINK_EPF_16NM[] = {
    3.375e-2, 6.75e-2, 13.5e-2, 27e-2
  };

  inline double LinkPleak (Technology t, int linkWidth) {
    DASSERT(t == TECH_16NM);
    return LINK_PLEAK_16NM[_LinkWidthIdx(linkWidth)];
  }

  inline double LinkEpf (Technology t, int linkWidth) {
    DASSERT(t == TECH_16NM);
    return LINK_EPF_16NM[_LinkWidthIdx(linkWidth)];
  }

} // namespace cmpex

#endif // _ROUTERDEFS_H_
