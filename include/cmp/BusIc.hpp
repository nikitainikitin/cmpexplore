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

#ifndef _CMP_BUSIC_H_
#define _CMP_BUSIC_H_

#include <vector>
#include <list>

#include "Interconnect.hpp"
#include <GGraph.hpp>

namespace cmpex {

  namespace cmp {

    class Cluster;

    //======================================================================
    // This class represents a bus interconnect for CMP.
    // The bus has a list of device components and possibly memory controllers
    // (not fully supported at the moment), plus an additional interface
    // component. The interface component is present implicitly as the last
    // one in the list (all matrices assume the last port is related to the iface).
    //======================================================================

    class BusIc : public Interconnect {

      typedef std::vector<double> DoubleArray;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      BusIc (Cluster * c, UInt subnCnt, double freq, double volt);

      virtual ~BusIc ();

      // Accessors
      // Total number of components, with iface and mcs
      inline int TotalCompCnt () const;

      // Number of device-related components
      inline int CompCnt () const;

      inline void CompCnt ( int c );

      inline int AccessTime () const;

      inline void AccessTime ( int t );

      inline double AccessTimeNs () const;

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

      // Access value of traffic matrix for 'iPort' -> 'oPort'.
      inline double& Traffic (UShort subnIdx, UShort iPort, UShort oPort);

      // Access value of buffer delay for 'iPort'.
      inline double& BufDelay (UShort subnIdx, UShort iPort);

      // Debug methods
      void Print () const;

    private:

      // Deprecated methods: prevent usage
      BusIc ( const BusIc& );

      BusIc& operator = ( const BusIc& );

      // -------------------------- Attributes -----------------------------

    private:

      int compCnt_; // number of components in bus
      
      int totalCompCnt_; // total number of components in bus, with iface and mcs
      
      int accessTime_; // bus access time (in cycles)

      // Matrix of traffic rates per IO-port pair per subnetwork.
      // To access the value of traffic from input port I to output port O
      // of subnetwork S use the the index
      // tm_[S*TotalCompCnt()*TotalCompCnt() + I*TotalCompCnt() + O].
      DoubleArray tm_;
      
      // Matrix of buffer delays per I-port per subnetwork.
      // To access the delay value for input port I of subnetwork S
      // use the the index bufDelays_[S*TotalCompCnt() + I].
      DoubleArray bufDelays_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    int BusIc::CompCnt () const {
      return compCnt_;
    }

    void BusIc::CompCnt (int c) {
      compCnt_ = c;
      totalCompCnt_ = c+1; // add iface component
    }

    int BusIc::TotalCompCnt () const {
      return totalCompCnt_;
    }

    int BusIc::AccessTime () const {
      return accessTime_;
    }

    void BusIc::AccessTime (int t) {
      accessTime_ = t;
    }

    double BusIc::AccessTimeNs () const {
      return accessTime_/Freq();
    }

    double& BusIc::Traffic (UShort subnIdx, UShort iPort, UShort oPort) {
      return tm_[subnIdx*TotalCompCnt()*TotalCompCnt() + iPort*TotalCompCnt() + oPort];
    }

    double& BusIc::BufDelay (UShort subnIdx, UShort iPort) {
      return bufDelays_[subnIdx*TotalCompCnt() + iPort];
    }

  } // namespace cmp

} // namespace cmpex

#endif // _CMP_BUSIC_H_
