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

#ifndef _CMP_CLUSTER_H_
#define _CMP_CLUSTER_H_

#include <vector>
#include <list>

#include "Component.hpp"

namespace cmpex {
  
  namespace cmp {

    class Interconnect;
    
    //======================================================================
    // This class represents a composite component of the Cmp.
    // It posesses a list of children component and an interconnect.
    //======================================================================

    class Cluster : public Component {
      
      friend class CmpBuilder;
      
    public:
      
      typedef std::vector<Component*> CompArray;
      
      typedef CompArray::const_iterator CCIter;
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Constructors and destructor
      Cluster ( UShort idx, UShort clIdx, Component * parent = 0 );

      virtual ~Cluster ();
      
      // Accessors
      inline const Cluster * ClParent() const; // returns parent cluster
      
      inline Cluster * ClParent(); // returns parent cluster
      
      inline UShort CompCnt() const;
      
      inline const Component * GetComponent (UShort idx) const;
      
      inline Component * GetComponent (UShort idx);
      
      inline const Interconnect * Ic() const;
      
      inline Interconnect * Ic();
      
      inline void SetProcOwnership ( int idx, Component * c );
      
      inline void SetMemOwnership ( int idx, Component * c);
      
      inline const Component * ProcOwner ( int idx ) const;
      
      inline Component * ProcOwner ( int idx );
      
      inline const Component * MemOwner ( int idx ) const;
      
      inline Component * MemOwner ( int idx );
      
      // Implementations of the Component interface.
      
      // Returns true if component contains the processor 'idx'.
      virtual bool HasProcessor (UShort idx);
      
      // Returns true if component contains the memory 'idx'.
      virtual bool HasMemory (UShort idx);
      
      // Returns hop-count distance from processor 'pIdx' to the Iface
      // component
      virtual int DistanceProcToIface (UShort pIdx);
      
      // Returns hop-count distance from memory 'mIdx' to the Iface
      // component
      virtual int DistanceMemToIface (UShort mIdx);
      
      // Returns uni-directional latency from the processor 'pIdx'
      // to the interface of component.
      virtual double ULatProcToIface (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx);
      
      // Returns uni-directional latency from the interface of component
      // to the processor 'pIdx'.
      virtual double ULatIfaceToProc (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx);
      
      // Returns uni-directional latency from the memory 'mIdx'
      // to the interface of component.
      virtual double ULatMemToIface (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx);
      
      // Returns uni-directional latency from the interface of component
      // to the memory 'mIdx'.
      virtual double ULatIfaceToMem (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx);
      
      // Returns uni-directional latency from the processor 'pIdx' to
      // the memory with index 'mIdx'.
      double ULatProcToMem (UShort pIdx, UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Returns uni-directional latency from the memory 'mIdx' to
      // the processor with index 'pIdx'.
      double ULatMemToProc (UShort mIdx, UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Returns full latency from the processor 'pIdx' to
      // the memory with index 'mIdx'.
      // !! NOTE: use this function only if cache coherence is not considered.
      double FLatProcToMem (UShort pIdx, UShort mIdx, bool dynamic);
      
      // Returns uni-directional latency from the processor 'pIdx' to
      // the memory controller with index 'mcIdx'.
      double ULatProcToMemCtrl (UShort pIdx, UShort mcIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Returns uni-directional latency from the memory controller 'mcIdx' to
      // the processor with index 'pIdx'.
      double ULatMemCtrlToProc (UShort mcIdx, UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Returns full latency from the processor 'pIdx' to
      // the memory controller with index 'mcIdx'.
      // !! NOTE: use this function only if cache coherence is not considered.
      double FLatProcToMemCtrl (UShort pIdx, UShort mcIdx, bool dynamic);
      
      // Returns uni-directional latency from the memory 'mIdx' to
      // the memory controller with index 'mcIdx'.
      double ULatMemToMemCtrl (UShort mIdx, UShort mcIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Returns uni-directional latency from the memory controller 'mcIdx' to
      // the memory with index 'mIdx'.
      double ULatMemCtrlToMem (UShort mcIdx, UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx);

      // Utility functions
      
      // Initializes ownership arrays using configuration data.
      void InitOwnerships();
      
      // Returns hop-count distance from processor 'pIdx' to memory 'mIdx'.
      int DistanceProcToMem (UShort pIdx, UShort mIdx);

      // Debug methods
      virtual void Print () const;

    protected:

      inline void SetIc ( Interconnect * ic );
      
      inline void AddComponent ( Component * c );
      
    private:

      // Deprecated methods: prevent usage
      Cluster ( const Cluster& );

      Cluster& operator = ( const Cluster& );

      // -------------------------- Attributes -----------------------------

    private:
      
      CompArray components_;
      
      Interconnect * ic_;
      
      // === Additional data structures for performance improvement ===
      
      // pointers to child components that contain processor/memory
      // with index 'idx'; procOwner_[idx] == 0 if no child component has 'idx'
      CompArray procOwners_;
      
      CompArray memOwners_;
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const Cluster * Cluster::ClParent () const {
      return static_cast<const Cluster*>(Parent());
    }
    
    Cluster * Cluster::ClParent () {
      return static_cast<Cluster*>(Parent());
    }
    
    UShort Cluster::CompCnt() const {
      return components_.size();
    }
    
    const Component * Cluster::GetComponent (UShort idx) const {
      return components_[idx];
    }
    
    Component * Cluster::GetComponent (UShort idx) {
      return components_[idx];
    }
    
    const Interconnect * Cluster::Ic () const {
      return ic_;
    }
    
    Interconnect * Cluster::Ic () {
      return ic_;
    }

    void Cluster::SetIc (Interconnect * ic) {
      ic_ = ic;
    }
    
    void Cluster::AddComponent (Component * c) {
      components_.push_back(c);
    }
    
    void Cluster::SetProcOwnership (int idx, Component * c) {
      procOwners_[idx] = c;
    }
    
    void Cluster::SetMemOwnership (int idx, Component * c) {
      memOwners_[idx] = c;
    }
    
    const Component * Cluster::ProcOwner (int idx) const {
      return procOwners_[idx];
    }
    
    Component * Cluster::ProcOwner (int idx) {
      return procOwners_[idx];
    }
    
    const Component * Cluster::MemOwner (int idx) const {
      return memOwners_[idx];
    }
    
    Component * Cluster::MemOwner (int idx) {
      return memOwners_[idx];
    }
    
  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_CLUSTER_H_
