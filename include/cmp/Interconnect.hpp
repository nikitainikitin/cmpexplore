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

#ifndef _CMP_INTERCONNECT_H_
#define _CMP_INTERCONNECT_H_

#include <vector>
#include <list>
#include <sstream>

#include "../Defs.hpp"

namespace cmpex {
  
  namespace cmp {

    class Cluster;
    
    //======================================================================
    // This class represents a basic class for cmp interconnects.
    // It guarantees common interface for interconnects.
    // The constructor is protected, so component itself can't be instantiated.
    //======================================================================

    class Interconnect {
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Destructor
      virtual ~Interconnect ();
      
      // Accessors
      
      inline const IcType Type() const;
      
      inline IcType Type();
      
      inline const Cluster * Parent() const;
      
      inline Cluster * Parent();

      inline UInt SubnCnt () const;
      
      inline double Freq () const;

      inline double Volt () const;

      // Pure virtual methods

      // Returns hop-count distance from component with index 'srcIdx' to
      // another component with index 'dstIdx'.
      virtual int DistanceCompToComp (UShort srcIdx, UShort dstIdx) = 0;
      
      // Returns hop-count distance from component with index 'idx' to
      // the Iface component.
      virtual int DistanceCompToIface (UShort idx) = 0;
      
      // Returns latency from component with index 'srcIdx' to
      // another component with index 'dstIdx'.
      virtual double LatencyCompToComp (
          UShort srcIdx, UShort dstIdx, UShort pSize, bool dynamic, UShort subnIdx) = 0;
      
      // Returns latency from component with index 'idx' to the memory
      // controller 'mcIdx'.
      virtual double LatencyCompToMemCtrl (
          UShort idx, UShort mcIdx, UShort pSize, bool dynamic, UShort subnIdx) = 0;
      
      // Returns latency from memory controller with index 'mcIdx' to
      // component with index 'idx'.
      virtual double LatencyMemCtrlToComp (
          UShort mcIdx, UShort idx, UShort pSize, bool dynamic, UShort subnIdx) = 0;
      
      // Returns latency from component with index 'idx' to
      // the interface component.
      virtual double LatencyCompToIface (UShort idx, UShort pSize, bool dynamic, UShort subnIdx) = 0;
      
      // Returns latency from the interface component to
      // component with index 'idx'.
      virtual double LatencyIfaceToComp (UShort idx, UShort pSize, bool dynamic, UShort subnIdx) = 0;
      
      // Follow route between components (or a component and a memctrl)
      // and save the traffic rates into the matrix.
      virtual void MarkPathCompToComp (
          UShort srcIdx, UShort dstIdx, double traffic, UShort subnIdx) = 0;
      
      virtual void MarkPathCompToMemCtrl (
          UShort cIdx, UShort mcIdx, double traffic, UShort subnIdx) = 0;
      
      virtual void MarkPathMemCtrlToComp (
          UShort mcIdx, UShort cIdx, double traffic, UShort subnIdx) = 0;
      
      virtual void MarkPathCompToIface (UShort idx, double traffic, UShort subnIdx) = 0;
      
      virtual void MarkPathIfaceToComp (UShort idx, double traffic, UShort subnIdx) = 0;
      
      // Initialize modeling data structure
      virtual void InitModel() = 0;
      
      // Run analytical modeling of the contention delays in input buffers
      virtual int EstimateBufferDelays(bool fixNegDelays = false) = 0;

      virtual bool PortOnPathCompToComp(UShort srcIdx, UShort dstIdx, UShort rIdx, UShort pIdx) = 0;
      
      virtual bool PortOnPathCompToMemCtrl(UShort srcIdx, UShort mcIdx, UShort rIdx, UShort pIdx) = 0;

      virtual bool PortOnPathMemCtrlToComp(UShort mcIdx, UShort dstIdx, UShort rIdx, UShort pIdx) = 0;

      virtual void DumpLatencyEqCompToComp(UShort srcIdx, UShort dstIdx,
                                           double prob, std::ostringstream& os) = 0;

      virtual void DumpLatencyEqCompToMemCtrl(UShort srcIdx, UShort mcIdx,
                                              double prob, std::ostringstream& os) = 0;

      virtual void DumpLatencyEqMemCtrlToComp(UShort mcIdx, UShort dstIdx,
                                              double prob, std::ostringstream& os) = 0;

      virtual bool PathCompToCompFollowsPorts(UShort srcIdx, UShort dstIdx,
                                              UShort r, UShort pi, UShort po) = 0;

      virtual bool PathCompToMemCtrlFollowsPorts(UShort srcIdx, UShort mcIdx,
                                                 UShort r, UShort pi, UShort po) = 0;

      virtual bool PathMemCtrlToCompFollowsPorts(UShort mcIdx, UShort dstIdx,
                                                 UShort r, UShort pi, UShort po) = 0;

      // Debug methods
      void Print () const;

    protected:

      Interconnect ( IcType type, Cluster * parent, double freq, double volt, UInt subnCnt = 1 );

    private:

      // Deprecated methods: prevent usage
      Interconnect ( const Interconnect& );

      Interconnect& operator = ( const Interconnect& );

      // -------------------------- Attributes -----------------------------

    private:
      
      IcType type_;
      
      Cluster * parent_;

      UInt subnCnt_; // number of physical subnetworks in the IC

      double freq_; // IC operating frequency

      double volt_; // IC operating voltage
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const IcType Interconnect::Type () const {
      return type_;
    }
    
    IcType Interconnect::Type () {
      return type_;
    }
    
    const Cluster * Interconnect::Parent () const {
      return parent_;
    }
    
    Cluster * Interconnect::Parent () {
      return parent_;
    }
    
    UInt Interconnect::SubnCnt () const {
      return subnCnt_;
    }

    double Interconnect::Freq () const {
      return freq_;
    }

    double Interconnect::Volt () const {
      return volt_;
    }

  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_INTERCONNECT_H_
