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

#ifndef _CMP_COMPONENT_H_
#define _CMP_COMPONENT_H_

#include <vector>
#include <list>

#include "../Defs.hpp"

namespace cmpex {
  
  namespace cmp {

    //======================================================================
    // This class represents a basic component of the Cmp composite structure.
    // It guarantees common interface for extended component types.
    // The constructor is protected, so component itself can't be instantiated.
    //======================================================================

    class Component {
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Destructor
      virtual ~Component ();
      
      // Accessors
      
      inline const UShort Idx() const;
      
      inline const UShort ClIdx() const;
      
      inline const CompType Type() const;
      
      inline const Component * Parent() const;
      
      inline Component * Parent();
      
      inline double Left () const;
      
      inline void Left ( double l );
      
      inline double Right () const;
      
      inline void Right ( double r );
      
      inline double Top () const;
      
      inline void Top ( double t );
      
      inline double Bottom () const;
      
      inline void Bottom ( double b );
      
      // Pure virtual methods
      
      // Returns true if component contains the processor 'idx'.
      virtual bool HasProcessor (UShort idx) = 0;
      
      // Returns true if component contains the memory 'idx'.
      virtual bool HasMemory (UShort idx) = 0;
      
      // Returns hop-count distance from processor 'pIdx' to the Iface
      // component
      virtual int DistanceProcToIface (UShort pIdx) = 0;
      
      // Returns hop-count distance from memory 'mIdx' to the Iface
      // component
      virtual int DistanceMemToIface (UShort mIdx) = 0;
      
      // Returns uni-directional latency from the processor 'pIdx'
      // to the interface of component.
      virtual double ULatProcToIface (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx) = 0;
      
      // Returns uni-directional latency from the interface of component
      // to the processor 'pIdx'.
      virtual double ULatIfaceToProc (UShort pIdx, bool dynamic, UShort pSize, UShort subnIdx) = 0;
      
      // Returns uni-directional latency from the memory 'mIdx'
      // to the interface of component.
      virtual double ULatMemToIface (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx) = 0;
      
      // Returns uni-directional latency from the interface of component
      // to the memory 'mIdx'.
      virtual double ULatIfaceToMem (UShort mIdx, bool dynamic, UShort pSize, UShort subnIdx) = 0;
      
      // Debug methods
      virtual void Print () const;

    protected:

      Component ( UShort idx, UShort clIdx, CompType type, Component * parent = 0 );

    private:

      // Deprecated methods: prevent usage
      Component ( const Component& );

      Component& operator = ( const Component& );

      // -------------------------- Attributes -----------------------------

    private:
      
      UShort idx_; // index of component among those of the same type
      
      UShort clIdx_; // index of component in its cluster
      
      CompType type_;
      
      Component * parent_;
      
      // Physical parameters
      // Coordinates, relative to parent (in units)
      double left_;
      
      double right_;
      
      double top_;
      
      double bottom_;
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const UShort Component::Idx () const {
      return idx_;
    }
    
    const UShort Component::ClIdx () const {
      return clIdx_;
    }
    
    const CompType Component::Type () const {
      return type_;
    }
    
    const Component * Component::Parent () const {
      return parent_;
    }
    
    Component * Component::Parent () {
      return parent_;
    }
    
    double Component::Left () const {
      return left_;
    }
    
    void Component::Left (double l) {
      left_ = l;
    }
    
    double Component::Right () const {
      return right_;
    }
    
    void Component::Right (double r) {
      right_ = r;
    }
    
    double Component::Top () const {
      return top_;
    }
    
    void Component::Top (double t) {
      top_ = t;
    }
    
    double Component::Bottom () const {
      return bottom_;
    }
    
    void Component::Bottom (double b) {
      bottom_ = b;
    }
    
  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_COMPONENT_H_
