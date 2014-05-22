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

#ifndef _CMP_MESHIC_H_
#define _CMP_MESHIC_H_

#include <vector>
#include <list>

#include "Interconnect.hpp"
#include <GGraph.hpp>
#include "MeshIcTile.hpp"
#include "MeshIcLink.hpp"
#include "CmpBuilder.hpp"

using std::cout;
using std::endl;

namespace cmpex {

  extern cmp::CmpBuilder cmpBuilder;

  namespace cmp {

    class Cluster;

    //======================================================================
    // This class represents a mesh interconnect for CMP, using grid graph.
    //======================================================================

    class MeshIc : public Interconnect, public GGraph {

      typedef std::vector<double> DoubleArray;

    public:

      // number of ports in router: integer value for quick access
      static const UShort PORT_NUM = UShort(NUMRD);

      // number of port pairs in router: integer value for quick access
      static const UShort PORT_PAIR_NUM = UShort(NUMRD*NUMRD);

      // enumerate mesh directions (for readability of certain algorithms)
      enum MeshDir { MESHDIRNORTH = 0, MESHDIRWEST, MESHDIRSOUTH, MESHDIREAST };

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      MeshIc (Cluster * c, UInt subnCnt, double freq, double volt);

      virtual ~MeshIc ();

      // Factory method for vertex: creates Tile object
      virtual GVertex * CreateVertex ( int idx );

      // Factory method for edge: creates Link object
      virtual GDEdge * CreateEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal );

      // Accessors
      inline double RowHeight () const;

      inline void RowHeight ( double h );

      inline double ColWidth () const;

      inline void ColWidth ( double w );

      inline int LinkDelay () const;

      inline void LinkDelay ( int d );

      inline double LinkDelayNs () const;

      inline double LinkEpf () const;

      inline void LinkEpf ( double epf );

      inline double LinkPleakOfTemp(double tmp) const;
      
      inline double LinkPgPleakOfTemp(double tmp) const;
      
      inline int RouterDelay () const;

      inline void RouterDelay ( int d );

      inline double RouterDelayNs () const;

      inline double RouterEpf () const;

      inline void RouterEpf ( double epf );

      inline double RouterPleakOfTemp(double tmp) const;
      
      inline double RouterPgPleakOfTemp(double tmp) const;
      
      inline int TCnt() const;

      inline const MeshIcTile * GetTile ( int idx ) const;

      inline MeshIcTile * GetTile ( int idx );

      inline int LCnt() const;

      inline const MeshIcLink * GetLink ( int idx ) const;

      inline MeshIcLink * GetLink ( int idx );

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
      
      // Initialize traffic matrix and buffer delays.
      virtual void InitModel();
      
      // Run analytical modeling of the contention delays in input buffers.
      virtual int EstimateBufferDelays(bool fixNegDelays = false);

      virtual bool PortOnPathCompToComp(UShort srcIdx, UShort dstIdx, UShort rIdx, UShort pIdx);

      virtual bool PortOnPathCompToMemCtrl(UShort srcIdx, UShort mc, UShort rIdx, UShort pIdx);

      virtual bool PortOnPathMemCtrlToComp(UShort mc, UShort dstIdx, UShort rIdx, UShort pIdx);

      virtual void DumpLatencyEqCompToComp(UShort srcIdx, UShort dstIdx,
                                           double prob, std::ostringstream& os);

      virtual void DumpLatencyEqCompToMemCtrl(UShort srcIdx, UShort mcIdx,
                                              double prob, std::ostringstream& os);

      virtual void DumpLatencyEqMemCtrlToComp(UShort mcIdx, UShort dstIdx,
                                              double prob, std::ostringstream& os);

      virtual bool PathCompToCompFollowsPorts(UShort srcIdx, UShort dstIdx,
                                              UShort r, UShort pi, UShort po);

      virtual bool PathCompToMemCtrlFollowsPorts(UShort srcIdx, UShort mcIdx,
                                                 UShort r, UShort pi, UShort po);

      virtual bool PathMemCtrlToCompFollowsPorts(UShort mcIdx, UShort dstIdx,
                                                 UShort r, UShort pi, UShort po);

      // Service functions

      // Follows the route from srcPort of srcRouter to dstPort of dstRouter
      // and returns the latency between two ports.
      double Latency (UShort srcRouter, RouteDir srcPort,
                      UShort dstRouter, RouteDir dstPort, bool dynamic, UShort subnIdx);

      // Follows the route from srcPort of srcRouter to dstPort of dstRouter
      // and fills in the traffic data into the matrix.
      void MarkPath (UShort srcRouter, RouteDir srcPort,
                     UShort dstRouter, RouteDir dstPort, double traffic, UShort subnIdx);

      // Access value of traffic matrix for 'iPort' -> 'oPort' of router 'rIdx'.
      inline double& Traffic (UShort subnIdx, UShort rIdx, RouteDir iPort, RouteDir oPort);

      // Access value of buffer delay for 'iPort' of router 'rIdx'.
      inline double& BufDelay (UShort subnIdx, UShort rIdx, RouteDir iPort);

      // Create active vector and set all routers to active
      inline void CreateInitActive (UShort rNum );

      // Set all routers to active
      inline void SetAllActive ( );

      // Set a router to active
      inline void SetActive (UShort rIdx);

      // Get active status for given router
      inline bool Active (UShort rIdx);

      // Debug methods
      void Print () const;

    private:

      // Deprecated methods: prevent usage
      MeshIc ( const MeshIc& );

      MeshIc& operator = ( const MeshIc& );

      // -------------------------- Attributes -----------------------------

    private:

      double rowHeight_; // height of row (in units)

      double colWidth_; // width of column (in units)

      int linkDelay_; // link delay (in cycles)

      int routerDelay_; // router delay (in cycles)

      double linkEpf_; //Energy per flit (nJ)

      double routerEpf_; //Energy per flit (nJ)

      // Matrix of traffic rates per IO-port pair per router per subnetwork.
      // To access the value of traffic from input port I to output port O
      // of router R in subnetwork S, use the the index
      // tm_[S*TCnt()*PORT_PAIR_NUM + R*PORT_PAIR_NUM + I*PORT_NUM + O].
      // It is assumed there're 5 input and 5 output ports per router.
      DoubleArray tm_;
      
      // Matrix of buffer delays per I-port per router per subnetwork.
      // To access the delay value for input port I of router R, in subnetwork S
      // use the the index bufDelays_[S*TCnt()*PORT_NUM + R*PORT_NUM + I].
      // It is assumed there're 5 input ports per router.
      DoubleArray bufDelays_;

      // Active status of routers
      vector<bool> active_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    double MeshIc::RowHeight () const {
      return rowHeight_;
    }

    void MeshIc::RowHeight (double h) {
      rowHeight_ = h;
    }

    double MeshIc::ColWidth () const {
      return colWidth_;
    }

    void MeshIc::ColWidth (double w) {
      colWidth_ = w;
    }

    int MeshIc::LinkDelay () const {
      return linkDelay_;
    }

    void MeshIc::LinkDelay (int d) {
      linkDelay_ = d;
    }

    double MeshIc::LinkDelayNs () const {
      return linkDelay_/Freq();
    }

    double MeshIc::LinkEpf () const {
      return linkEpf_;
    }

    void MeshIc::LinkEpf (double epf) {
      linkEpf_ = epf;
    }

    double MeshIc::LinkPleakOfTemp ( double tmp ) const {
      return cmpBuilder.LinksLeakageOfTemp(tmp);
    }

    double MeshIc::LinkPgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.LinksPgLeakageOfTemp(tmp);
    }

    int MeshIc::RouterDelay () const {
      return routerDelay_;
    }

    void MeshIc::RouterDelay (int d) {
      routerDelay_ = d;
    }

    double MeshIc::RouterDelayNs () const {
      return routerDelay_/Freq();
    }

    double MeshIc::RouterEpf () const {
      return routerEpf_;
    }

    void MeshIc::RouterEpf (double epf) {
      routerEpf_ = epf;
    }

    double MeshIc::RouterPleakOfTemp ( double tmp ) const {
      return cmpBuilder.RouterLeakageOfTemp(tmp);
    }

    double MeshIc::RouterPgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.RouterPgLeakageOfTemp(tmp);
    }

    int MeshIc::TCnt () const {
      return VCnt();
    }

    int MeshIc::LCnt () const {
      return ECnt();
    }

    const MeshIcTile * MeshIc::GetTile ( int idx ) const {
      return static_cast<const MeshIcTile*>(*(VBegin()+idx));
    }

    MeshIcTile * MeshIc::GetTile ( int idx ) {
      return static_cast<MeshIcTile*>(*(VBegin()+idx));
    }

    const MeshIcLink * MeshIc::GetLink ( int idx ) const {
      return static_cast<const MeshIcLink*>(*(EBegin()+idx));
    }

    MeshIcLink * MeshIc::GetLink ( int idx ) {
      return static_cast<MeshIcLink*>(*(EBegin()+idx));
    }

    double& MeshIc::Traffic (UShort subnIdx, UShort rIdx, RouteDir iPort, RouteDir oPort) {
      return tm_[subnIdx*TCnt()*PORT_PAIR_NUM + rIdx*PORT_PAIR_NUM + iPort*PORT_NUM +oPort];
    }

    double& MeshIc::BufDelay (UShort subnIdx, UShort rIdx, RouteDir iPort) {
      return bufDelays_[subnIdx*TCnt()*PORT_NUM + rIdx*PORT_NUM + iPort];
    }

    void MeshIc::SetAllActive ( ) {
      std::fill(active_.begin(),active_.end(),1);
    }

    void MeshIc::CreateInitActive (UShort rNum ) {
      active_.resize(rNum);
      active_.assign(rNum,1);
    }

    void MeshIc::SetActive (UShort rIdx ) {
      active_[rIdx]=1;
    }

    bool MeshIc::Active (UShort rIdx) {
      return active_[rIdx];
    }

  } // namespace cmp

} // namespace cmpex

#endif // _CMP_MESHIC_H_
