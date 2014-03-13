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

#ifndef _EXPLORE_TRANSFORM_H_
#define _EXPLORE_TRANSFORM_H_

#include <string>
#include <vector>

#include "Config.hpp"
#include "arch/ArchPlanner.hpp"
#include "explore/ExplConf.hpp"

using std::string;
using std::vector;

namespace cmpex {

  extern Config config;

  using arch::ArchPlanner;

  namespace explore {

    //======================================================================
    // Transform represents the interface for transformations used by
    // metaheuristical exploration engines.
    /// TODO: Transformations are now hardcoded for 3 types of cores.
    /// TODO: Remove this limitation in the future.
    //======================================================================

    struct Transform {

      // ---------------------------- Methods ------------------------------

      // Destructor
      virtual ~Transform () {}
      
      // Main method that updates the architectural planner (AP).
      // First initialize AP with current configuration, then apply the transform.
      // Return true if applied successfully.
      virtual bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const = 0;

    };

    //======================================================================
    // -1- Next IC transformation.
    //======================================================================
    struct TrNextIC : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        curConf.SetAp(ap);
        ap.ClusterIcTypeIter((ap.ClusterIcTypeIter()+1)%config.ClusterIcType().size());
        return true;
      }
    };

    //======================================================================
    // -2- Prev IC transformation.
    //======================================================================
    struct TrPrevIC : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        curConf.SetAp(ap);
        ap.ClusterIcTypeIter((config.ClusterIcType().size()+ap.ClusterIcTypeIter()-1)
                             %config.ClusterIcType().size());
        return true;
      }
    };

    //======================================================================
    // -3- Increase X transformation.
    //======================================================================
    struct TrIncX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -4- Decrease X transformation.
    //======================================================================
    struct TrDecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -5- Increase Y transformation.
    //======================================================================
    struct TrIncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -6- Decrease Y transformation.
    //======================================================================
    struct TrDecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -7- Increase P1 transformation.
    //======================================================================
    struct TrIncP1 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        curConf.SetAp(ap);
        ap.ProcIter()[0] = ap.ProcIter()[0]+1;
        return true;
      }
    };

    //======================================================================
    // -8- Decrease P1 transformation.
    //======================================================================
    struct TrDecP1 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt0 > 1 || (curConf.procCnt0 == 1 && curConf.procCnt1 > 0)) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -9- Increase L1-1 transformation.
    //======================================================================
    struct TrIncL11 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_0_idx < config.ProcL1SizeCnt(0)-1) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[0] = ap.ProcL1SizeIter()[0]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -10- Decrease L1-1 transformation.
    //======================================================================
    struct TrDecL11 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_0_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[0] = ap.ProcL1SizeIter()[0]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -11- Increase L2-1 transformation.
    //======================================================================
    struct TrIncL21 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_0_idx < config.ProcL2SizeCnt(0)-1) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[0] = ap.ProcL2SizeIter()[0]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -12- Decrease L2-1 transformation.
    //======================================================================
    struct TrDecL21 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_0_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[0] = ap.ProcL2SizeIter()[0]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -13- Increase P2 transformation.
    //======================================================================
    struct TrIncP2 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        curConf.SetAp(ap);
        ap.ProcIter()[1] = ap.ProcIter()[1]+1;
        return true;
      }
    };

    //======================================================================
    // -14- Decrease P2 transformation.
    //======================================================================
    struct TrDecP2 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt1 > 1 || (curConf.procCnt1 == 1 && curConf.procCnt0 > 0)) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -15- Increase L1-2 transformation.
    //======================================================================
    struct TrIncL12 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_1_idx < config.ProcL1SizeCnt(1)-1) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[1] = ap.ProcL1SizeIter()[1]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -16- Decrease L1-2 transformation.
    //======================================================================
    struct TrDecL12 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_1_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[1] = ap.ProcL1SizeIter()[1]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -17- Increase L2-2 transformation.
    //======================================================================
    struct TrIncL22 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_1_idx < config.ProcL2SizeCnt(1)-1) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[1] = ap.ProcL2SizeIter()[1]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -18- Decrease L2-2 transformation.
    //======================================================================
    struct TrDecL22 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_1_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[1] = ap.ProcL2SizeIter()[1]-1;
          return true;
        }
        return false;
      }
    };


    // -------------- complex updates -----------------------

    //======================================================================
    // -19- Increase P1 and decrease P2 transformation.
    //======================================================================
    struct TrIncP1DecP2 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt1 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]+1;
          ap.ProcIter()[1] = ap.ProcIter()[1]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -20- Increase P1 and decrease X transformation.
    //======================================================================
    struct TrIncP1DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]+1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -21- Increase P1 and decrease Y transformation.
    //======================================================================
    struct TrIncP1DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]+1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -22- Increase P2 and decrease P1 transformation.
    //======================================================================
    struct TrIncP2DecP1 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt0 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]-1;
          ap.ProcIter()[1] = ap.ProcIter()[1]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -23- Increase P2 and decrease X transformation.
    //======================================================================
    struct TrIncP2DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]+1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -24- Increase P2 and decrease Y transformation.
    //======================================================================
    struct TrIncP2DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]+1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -25- Decrease P1 and increase X transformation.
    //======================================================================
    struct TrDecP1IncX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt0 > 1 || curConf.procCnt0 == 1 && curConf.procCnt1 > 0) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]-1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -26- Decrease P1 and increase Y transformation.
    //======================================================================
    struct TrDecP1IncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt0 > 1 || curConf.procCnt0 == 1 && curConf.procCnt1 > 0) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]-1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -27- Decrease P2 and increase X transformation.
    //======================================================================
    struct TrDecP2IncX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt1 > 1 || curConf.procCnt1 == 1 && curConf.procCnt0 > 0) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]-1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -28- Decrease P2 and increase Y transformation.
    //======================================================================
    struct TrDecP2IncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt1 > 1 || curConf.procCnt1 == 1 && curConf.procCnt0 > 0) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]-1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -29- Increase X and decrease Y transformation.
    //======================================================================
    struct TrIncXDecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx < config.GMeshDimXVec().size()-1 && curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -30- Decrease X and increase Y transformation.
    //======================================================================
    struct TrDecXIncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx < config.GMeshDimYVec().size()-1 && curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -31- Recluster: Increase P1 and decrease X transformation.
    //======================================================================
    struct TrReclIncP1DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()-1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -32- Recluster: Increase P1 and decrease Y transformation.
    //======================================================================
    struct TrReclIncP1DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()-1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -33- Recluster: Increase P2 and decrease X transformation.
    //======================================================================
    struct TrReclIncP2DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()-1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -34- Recluster: Increase P2 and decrease Y transformation.
    //======================================================================
    struct TrReclIncP2DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()-1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -35- Recluster: Decrease P1 and increase X transformation.
    //======================================================================
    struct TrReclDecP1IncX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt0 > 1 || curConf.procCnt0 == 1 && curConf.procCnt1 > 0) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()+1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -36- Recluster: Decrease P1 and increase Y transformation.
    //======================================================================
    struct TrReclDecP1IncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt0 > 1 || curConf.procCnt0 == 1 && curConf.procCnt1 > 0) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()+1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -37- Recluster: Decrease P2 and increase X transformation.
    //======================================================================
    struct TrReclDecP2IncX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt1 > 1 || curConf.procCnt1 == 1 && curConf.procCnt0 > 0) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()+1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -38- Recluster: Decrease P2 and increase Y transformation.
    //======================================================================
    struct TrReclDecP2IncY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt1 > 1 || curConf.procCnt1 == 1 && curConf.procCnt0 > 0) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()+1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };



    // P3 cases, 1-type

    //======================================================================
    // -39- Increase P3 transformation.
    //======================================================================
    struct TrIncP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        curConf.SetAp(ap);
        ap.ProcIter()[2] = ap.ProcIter()[2]+1;
        return true;
      }
    };

    //======================================================================
    // -40- Decrease P3 transformation.
    //======================================================================
    struct TrDecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt2 > 1 || (curConf.procCnt2 == 1 &&
                                      (curConf.procCnt0 > 0 || curConf.procCnt1 > 0))) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -41- Increase L1-3 transformation.
    //======================================================================
    struct TrIncL13 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_2_idx < config.ProcL1SizeCnt(2)-1) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[2] = ap.ProcL1SizeIter()[2]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -42- Decrease L1-3 transformation.
    //======================================================================
    struct TrDecL13 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l1_2_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL1SizeIter()[2] = ap.ProcL1SizeIter()[2]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -43- Increase L2-3 transformation.
    //======================================================================
    struct TrIncL23 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_2_idx < config.ProcL2SizeCnt(2)-1) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[2] = ap.ProcL2SizeIter()[2]+1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -44- Decrease L2-3 transformation.
    //======================================================================
    struct TrDecL23 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.l2_2_idx > 0) {
          curConf.SetAp(ap);
          ap.ProcL2SizeIter()[2] = ap.ProcL2SizeIter()[2]-1;
          return true;
        }
        return false;
      }
    };


    // P3 cases, 2-type

    //======================================================================
    // -45- Increase P3 and decrease P1 transformation.
    //======================================================================
    struct TrIncP3DecP1 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt0 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]+1;
          ap.ProcIter()[0] = ap.ProcIter()[0]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -46- Increase P3 and decrease P2 transformation.
    //======================================================================
    struct TrIncP3DecP2 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt1 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]+1;
          ap.ProcIter()[1] = ap.ProcIter()[1]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -47- Increase P3 and decrease X transformation.
    //======================================================================
    struct TrIncP3DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]+1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -48- Increase P3 and decrease Y transformation.
    //======================================================================
    struct TrIncP3DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]+1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -49- Increase P1 and decrease P3 transformation.
    //======================================================================
    struct TrIncP1DecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt2 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[0] = ap.ProcIter()[0]+1;
          ap.ProcIter()[2] = ap.ProcIter()[2]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -50- Increase P2 and decrease P3 transformation.
    //======================================================================
    struct TrIncP2DecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.procCnt2 > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[1] = ap.ProcIter()[1]+1;
          ap.ProcIter()[2] = ap.ProcIter()[2]-1;
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -51- Increase X and decrease P3 transformation.
    //======================================================================
    struct TrIncXDecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt2 > 1 || curConf.procCnt2 == 1 &&
             (curConf.procCnt1 > 0 || curConf.procCnt0 > 0)) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]-1;
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -52- Increase Y and decrease P3 transformation.
    //======================================================================
    struct TrIncYDecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt2 > 1 || curConf.procCnt2 == 1 &&
             (curConf.procCnt1 > 0 || curConf.procCnt0 > 0)) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] = ap.ProcIter()[2]-1;
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -53- Recluster: Increase P3 and decrease X transformation.
    //======================================================================
    struct TrReclIncP3DecX : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.xIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()-1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -54- Recluster: Increase P3 and decrease Y transformation.
    //======================================================================
    struct TrReclIncP3DecY : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if (curConf.yIdx > 0) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()-1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()-1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -55- Recluster: Increase X and decrease P3 transformation.
    //======================================================================
    struct TrReclIncXDecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt2 > 1 || curConf.procCnt2 == 1 &&
             (curConf.procCnt1 > 0 || curConf.procCnt0 > 0)) &&
            curConf.xIdx < config.GMeshDimXVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] *= double(config.GMeshDimXVec()[ap.GMeshDimXIter()])/
                              double(config.GMeshDimXVec()[ap.GMeshDimXIter()+1]);
          ap.GMeshDimXIter(ap.GMeshDimXIter()+1);
          return true;
        }
        return false;
      }
    };

    //======================================================================
    // -56- Recluster: Increase Y and decrease P3 transformation.
    //======================================================================
    struct TrReclIncYDecP3 : public Transform {
      bool UpdateAP(const ExplConf& curConf, ArchPlanner& ap) const {
        if ((curConf.procCnt2 > 1 || curConf.procCnt2 == 1 &&
             (curConf.procCnt1 > 0 || curConf.procCnt0 > 0)) &&
            curConf.yIdx < config.GMeshDimYVec().size()-1) {
          curConf.SetAp(ap);
          ap.ProcIter()[2] *= double(config.GMeshDimYVec()[ap.GMeshDimYIter()])/
                              double(config.GMeshDimYVec()[ap.GMeshDimYIter()+1]);
          ap.GMeshDimYIter(ap.GMeshDimYIter()+1);
          return true;
        }
        return false;
      }
    };

  } // namespace explore

} // namespace cmpex

#endif // _EXPLORE_TRANSFORM_H_
