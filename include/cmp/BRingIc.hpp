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

#ifndef _CMP_BRINGIC_H_
#define _CMP_BRINGIC_H_

#include <vector>
#include <list>

#include "Interconnect.hpp"

namespace cmpex {

  namespace cmp {

    class Cluster;

    //======================================================================
    // This class represents a bidirectional ring interconnect for CMP.
    // The ring has a list of device components and possibly memory controllers
    // (not fully supported at the moment), plus an additional interface
    // component (NI). The interface component is present implicitly as the last
    // one in the list. The ring has two virtual channels to resolve deadlocks
    // (referred to as ports). Packets switch from VC0 to VC1 once they travel
    // from the NI component to the following one.
    // The routing algorithm is the shortest path. In case of equidistant
    // routing paths in both directions clockwise direction is chosen.
    //
    // Port indices (structure of traffic matrix and buffer arrays):
    // Every ring router has 3 physical (5 virtual) input ports:
    //   Index 0: component
    //   Index 1, 2: pass-through connection clockwise direction
    //   Index 3, 4: pass-through connection counterclockwise direction
    // Every ring router has 3 output ports:
    //   Index 0: component
    //   Index 1: pass-through clockwise
    //   Index 2: pass-through counterclockwise
    //======================================================================

    class BRingIc : public Interconnect {

      typedef std::vector<double> DoubleArray;

    public:

      // number of input ports in ring router: integer value for quick access
      static const UShort IPORT_NUM = 5;

      // number of output ports in ring router: integer value for quick access
      static const UShort OPORT_NUM = 3;

      // number of port pairs in router: integer value for quick access
      static const UShort PORT_PAIR_NUM = UShort(IPORT_NUM*OPORT_NUM);

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      BRingIc (Cluster * c, UInt subnCnt);

      virtual ~BRingIc ();

      // Accessors
      // Total number of components, with iface and mcs
      inline int TotalCompCnt () const;

      // Number of device-related components
      inline int CompCnt () const;

      inline void CompCnt ( int c );

      inline int RouterDelay () const;

      inline void RouterDelay ( int d );

      inline int LinkDelay () const;

      inline void LinkDelay ( int d );

      // Implementation of the Interconnect interface

      // Returns hop-count distance from component with index 'srcIdx' to
      // another component with index 'dstIdx'.
      virtual int DistanceCompToComp (UShort srcIdx, UShort dstIdx);

      // Returns hop-count distance from component with index 'idx' to
      // the Iface component.
      virtual int DistanceCompToIface (UShort idx);

      // Returns latency from component with index 'srcIdx' to
      // another component with index 'dstIdx'.
      virtual double LatencyCompToComp (
          UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx);

      // Returns latency from component with index 'idx' to the memory
      // controller 'mcIdx'.
      virtual double LatencyCompToMemCtrl (
          UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx);

      // Returns latency from component with index 'idx' to
      // the interface component.
      virtual double LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx);

      // Returns latency from the interface component to
      // component with index 'idx'.
      virtual double LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx);

      // Returns latency from memory controller with index 'mcIdx' to
      // component with index 'idx'.
      virtual double LatencyMemCtrlToComp (
          UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx);

      // Follow route between components (or a component and a memctrl)
      // and save the traffic rates into the matrix.
      virtual void MarkPathCompToComp (
          UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx);

      virtual void MarkPathCompToMemCtrl (
          UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx);

      virtual void MarkPathMemCtrlToComp (
          UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx);

      virtual void MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx);

      virtual void MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx);

      // Initialize traffic matrix and duffer delays.
      virtual void InitModel();

      // Run analytical modeling of the contention delays in input buffers.
      virtual int EstimateBufferDelays(bool fixNegDelays = false);

      // Access value of traffic matrix for 'iPort' -> 'oPort' of router 'rIdx'.
      inline double& Traffic (UShort subnIdx, UShort rIdx, UShort iPort, UShort oPort);

      // Access value of buffer delay for 'iPort' of router 'rIdx'.
      inline double& BufDelay (UShort subnIdx, UShort rIdx, UShort iPort);

      /* The following are the rudiments of the MATLAB model and should be eliminated. */

      virtual bool PortOnPathCompToComp(UShort srcIdx, UShort dstIdx, UShort rIdx, UShort pIdx) {
        return false;
      }

      virtual bool PortOnPathCompToMemCtrl(UShort srcIdx, UShort mc, UShort rIdx, UShort pIdx) {
        return false;
      }

      virtual bool PortOnPathMemCtrlToComp(UShort mc, UShort dstIdx, UShort rIdx, UShort pIdx) {
        return false;
      }

      virtual void DumpLatencyEqCompToComp(UShort srcIdx, UShort dstIdx,
                                           double prob, std::ostringstream& os) {}

      virtual void DumpLatencyEqCompToMemCtrl(UShort srcIdx, UShort mcIdx,
                                              double prob, std::ostringstream& os) {}

      virtual void DumpLatencyEqMemCtrlToComp(UShort mcIdx, UShort dstIdx,
                                              double prob, std::ostringstream& os) {}

      virtual bool PathCompToCompFollowsPorts(UShort srcIdx, UShort dstIdx,
                                              UShort r, UShort pi, UShort po) {}

      virtual bool PathCompToMemCtrlFollowsPorts(UShort srcIdx, UShort mcIdx,
                                                 UShort r, UShort pi, UShort po) {}

      virtual bool PathMemCtrlToCompFollowsPorts(UShort mcIdx, UShort dstIdx,
                                                 UShort r, UShort pi, UShort po) {}

      // Debug methods
      void Print () const;

    private:

      // Deprecated methods: prevent usage
      BRingIc ( const BRingIc& );

      BRingIc& operator = ( const BRingIc& );

      // -------------------------- Attributes -----------------------------

    private:

      int compCnt_; // number of components in ring

      int totalCompCnt_; // total number of components in ring, with iface and mcs

      int routerDelay_; // router delay (in cycles)

      int linkDelay_; // link delay (in cycles)

      // Matrix of traffic rates per IO-port pair per router per subnetwork.
      // To access the value of traffic from input port I to output port O
      // of router R in subnetwork S, use the the index
      // tm_[S*TotalCompCnt()*PORT_PAIR_NUM + R*PORT_PAIR_NUM + I*OPORT_NUM + O].
      DoubleArray tm_;

      // Matrix of buffer delays per I-port per router per subnetwork.
      // To access the delay value for input port I of router R in subnetwork S,
      // use the the index bufDelays_[S*TotalCompCnt()*IPORT_NUM + R*IPORT_NUM + I].
      DoubleArray bufDelays_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    int BRingIc::CompCnt () const {
      return compCnt_;
    }

    void BRingIc::CompCnt (int c) {
      compCnt_ = c;
      totalCompCnt_ = c+1; // add iface component
    }

    int BRingIc::TotalCompCnt () const {
      return totalCompCnt_;
    }

    int BRingIc::RouterDelay () const {
      return routerDelay_;
    }

    void BRingIc::RouterDelay (int d) {
      routerDelay_ = d;
    }

    int BRingIc::LinkDelay () const {
      return linkDelay_;
    }

    void BRingIc::LinkDelay (int d) {
      linkDelay_ = d;
    }

    double& BRingIc::Traffic (UShort subnIdx, UShort rIdx, UShort iPort, UShort oPort) {
      return tm_[subnIdx*TotalCompCnt()*PORT_PAIR_NUM + rIdx*PORT_PAIR_NUM + iPort*OPORT_NUM +oPort];
    }

    double& BRingIc::BufDelay (UShort subnIdx, UShort rIdx, UShort iPort) {
      return bufDelays_[subnIdx*TotalCompCnt()*IPORT_NUM + rIdx*IPORT_NUM + iPort];
    }

  } // namespace cmp

} // namespace cmpex

#endif // _CMP_BRINGIC_H_
