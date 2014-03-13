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

#ifndef _ARCH_PARAMITERATOR_H_
#define _ARCH_PARAMITERATOR_H_

#include <string>
#include <vector>

#include "Config.hpp"

using std::string;
using std::vector;

namespace cmpex {

  namespace arch {
  
    class ArchPlanner;

    //======================================================================
    // ParamIterator represents interface for parameter iterators.
    //======================================================================

    class ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ParamIterator (ArchPlanner * ap, const string& name);

      virtual ~ParamIterator ();
      
      inline const string& Name () const;

      // Initialize corresponding ArchPlanner iterators
      // and return false if no values are possible for the iterator.
      virtual bool Init() = 0;
      
      // Advances iterator to the next position.
      virtual void Advance() = 0;
      
      // Return true if there current iterator value is valid.
      virtual bool Valid() = 0;

    protected:
      
      inline ArchPlanner * Ap() const;
      
    private:

      // Deprecated methods: prevent usage
      ParamIterator ( const ParamIterator& );

      ParamIterator& operator = ( const ParamIterator& );

      // -------------------------- Attributes -----------------------------

    private:
    
      ArchPlanner * ap_;

      string name_;

    };

    //======================================================================
    // GMeshDimIter is ParamIterator for global mesh dimensions.
    //======================================================================

    class GMeshDimIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      GMeshDimIter (ArchPlanner * ap);

      virtual ~GMeshDimIter ();
      
      bool Init();
      
      void Advance();
      
      bool Valid();

    private:

      // Deprecated methods: prevent usage
      GMeshDimIter ( const GMeshDimIter& );

      GMeshDimIter& operator = ( const GMeshDimIter& );

    };
    
    //======================================================================
    // ProcSetIter is ParamIterator for set of processors in cluster.
    //======================================================================

    class ProcSetIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ProcSetIter (ArchPlanner * ap);

      virtual ~ProcSetIter ();

      // Accessors

      inline double MinProcArea () const;

      inline UInt MinProcAreaIdx () const;

      bool Init();

      void Advance();

      bool Valid();

      double ClusterArea() const;

      double ProcClusterArea() const;

      bool ProcClusterAreaViolated() const;

    protected:

      inline void Invalidate();

    private:

      // Deprecated methods: prevent usage
      ProcSetIter ( const ProcSetIter& );

      ProcSetIter& operator = ( const ProcSetIter& );

      // -------------------------- Attributes -----------------------------

    private:

      // Stores the area of the smallest available processor
      double minProcArea_;

      // Stores the idx of the processor with the smallest area
      UInt minProcAreaIdx_;

      // Iterator is valid (there are processor sets to explore).
      // Note that this doesn't guarantee the cluster area constraint.
      bool valid_;

    };

    //======================================================================
    // ProcL1SizeIter is ParamIterator for L1 sizes of processors.
    //======================================================================

    class ProcL1SizeIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ProcL1SizeIter (ArchPlanner * ap);

      virtual ~ProcL1SizeIter ();

      bool Init();

      void Advance();

      bool Valid();

    protected:

      inline void Invalidate();

    private:

      // Deprecated methods: prevent usage
      ProcL1SizeIter ( const ProcL1SizeIter& );

      ProcL1SizeIter& operator = ( const ProcL1SizeIter& );

      // -------------------------- Attributes -----------------------------

    private:

      // Iterator is valid (there are more l1 sizes to explore).
      // Note that this doesn't guarantee the cluster area constraint.
      bool valid_;
    };

    //======================================================================
    // ProcL2SizeIter is ParamIterator for L2 sizes of processors.
    //======================================================================

    class ProcL2SizeIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ProcL2SizeIter (ArchPlanner * ap);

      virtual ~ProcL2SizeIter ();

      bool Init();

      void Advance();

      bool Valid();

    protected:

      inline void Invalidate();

    private:

      // Deprecated methods: prevent usage
      ProcL2SizeIter ( const ProcL2SizeIter& );

      ProcL2SizeIter& operator = ( const ProcL2SizeIter& );

      // -------------------------- Attributes -----------------------------

    private:

      // Iterator is valid (there are more l2 sizes to explore).
      // Note that this doesn't guarantee the cluster area constraint.
      bool valid_;
    };

    //======================================================================
    // L3SetIter is ParamIterator for set of memories in cluster.
    //======================================================================

    class L3SetIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      L3SetIter (ArchPlanner * ap);

      virtual ~L3SetIter ();

      bool Init();

      void Advance();

      bool Valid();

    private:

      // Deprecated methods: prevent usage
      L3SetIter ( const L3SetIter& );

      L3SetIter& operator = ( const L3SetIter& );

    };

    //======================================================================
    // ClusterIcTypeIter is ParamIterator for interconnect type in cluster.
    //======================================================================

    class ClusterIcTypeIter : public ParamIterator {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ClusterIcTypeIter (ArchPlanner * ap);

      virtual ~ClusterIcTypeIter ();

      bool Init();

      void Advance();

      bool Valid();

    private:

      // Deprecated methods: prevent usage
      ClusterIcTypeIter ( const ClusterIcTypeIter& );

      ClusterIcTypeIter& operator = ( const ClusterIcTypeIter& );

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    ArchPlanner * ParamIterator::Ap() const {
      return ap_;
    }

    const string& ParamIterator::Name() const {
      return name_;
    }

    double ProcSetIter::MinProcArea() const {
      return minProcArea_;
    }

    UInt ProcSetIter::MinProcAreaIdx() const {
      return minProcAreaIdx_;
    }

    void ProcSetIter::Invalidate() {
      valid_ = false;
    }

    void ProcL1SizeIter::Invalidate() {
      valid_ = false;
    }

    void ProcL2SizeIter::Invalidate() {
      valid_ = false;
    }

  } // namespace arch

} // namespace cmpex

#endif // _ARCH_PARAMITERATOR_H_
