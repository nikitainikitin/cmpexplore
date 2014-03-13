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

#ifndef _ARCH_ARCHPLANNER_H_
#define _ARCH_ARCHPLANNER_H_

#include <string>
#include <vector>
#include <cmath>

#include "Defs.hpp"

using std::string;
using std::vector;

namespace cmpex {

  namespace arch {

    class ArchConfig;
    class ParamIterator;

    //======================================================================
    // ArchPlanner performs architectural planning of a cmp.
    //======================================================================

    class ArchPlanner {

      typedef vector<ParamIterator*> ParamIterVec;

      typedef ParamIterVec::const_iterator PCIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ArchPlanner ();

      virtual ~ArchPlanner ();
      
      // Accessors
      
      inline UInt GMeshDimXIter () const;
      
      inline void GMeshDimXIter (UInt d);

      inline UInt GMeshDimYIter () const;
      
      inline void GMeshDimYIter (UInt d);

      inline UInt ClusterIcTypeIter () const;

      inline void ClusterIcTypeIter (UInt d);

      inline vector<int>& ProcIter();

      inline vector<int>& ProcL1SizeIter();

      inline vector<int>& ProcL2SizeIter();

      inline UInt NumL3Iter () const;

      inline void NumL3Iter (UInt n);

      ArchConfig * GenerateNextArchConfig();

      ArchConfig * GenerateCurrentArchConfig();

      // adjust L3Size to realistic value, i.e. integer number of Mbs
      // or power of 2 in Kbs.
      inline double AdjustL3Size(double l3Size) const;

    protected:

      inline const ParamIterVec& ParamIters() const;

      inline ParamIterVec& ParamIters();

      bool AllItersValid() const;

    private:

      // Deprecated methods: prevent usage
      ArchPlanner ( const ArchPlanner& );

      ArchPlanner& operator = ( const ArchPlanner& );

      // -------------------------- Attributes -----------------------------

    private:

      ParamIterVec paramIters_;

      // ============== Iterators for config generation ==============

      // The following two iterators represent the indices in the Config's
      // vector of global mesh dimensions.
      UInt gMeshDimXIter_;

      UInt gMeshDimYIter_;

      // Iterates over the indices in the Config's vector of IC types
      UInt clusterIcTypeIter_;

      // Processor iterator represents the number of processors of every type
      // to fill in the cluster.
      // procIter_.size() is equal to the config.ProcCnt().
      vector<int> procIter_;

      // L1 iterator represents the L1 size per processor of every type.
      // procL1Iter_.size() is equal to the config.ProcCnt().
      vector<int> procL1SizeIter_;
      
      // L2 iterator represents the L2 size per processor of every type.
      // procL2Iter_.size() is equal to the config.ProcCnt().
      vector<int> procL2SizeIter_;

      // L3 iterator represents the index in the Config's
      // vector of the number of L3 memories per cluster.
      UInt numL3Iter_;
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    UInt ArchPlanner::GMeshDimXIter() const {
      return gMeshDimXIter_;
    }

    void ArchPlanner::GMeshDimXIter(UInt d) {
      gMeshDimXIter_ = d;
    }

    UInt ArchPlanner::GMeshDimYIter() const {
      return gMeshDimYIter_;
    }

    void ArchPlanner::GMeshDimYIter(UInt d) {
      gMeshDimYIter_ = d;
    }

    UInt ArchPlanner::ClusterIcTypeIter() const {
      return clusterIcTypeIter_;
    }

    void ArchPlanner::ClusterIcTypeIter(UInt t) {
      clusterIcTypeIter_ = t;
    }

    vector<int>& ArchPlanner::ProcIter() {
      return procIter_;
    }

    vector<int>& ArchPlanner::ProcL1SizeIter() {
      return procL1SizeIter_;
    }

    vector<int>& ArchPlanner::ProcL2SizeIter() {
      return procL2SizeIter_;
    }

    UInt ArchPlanner::NumL3Iter() const {
      return numL3Iter_;
    }

    void ArchPlanner::NumL3Iter(UInt n) {
      numL3Iter_ = n;
    }

    const ArchPlanner::ParamIterVec& ArchPlanner::ParamIters() const {
      return paramIters_;
    }

    ArchPlanner::ParamIterVec& ArchPlanner::ParamIters() {
      return paramIters_;
    }

    inline double ArchPlanner::AdjustL3Size(double l3Size) const {
      if (l3Size > 1.0+E_DOUBLE)
        return floor(l3Size);
      else if (l3Size > 0.75+E_DOUBLE)
        return 0.75;
      else if (l3Size > 0.5+E_DOUBLE)
        return 0.5;
      else if (l3Size > 0.25+E_DOUBLE)
        return 0.25;
      else if (l3Size > 0.125+E_DOUBLE)
        return 0.125;
      else if (l3Size > 0.0625+E_DOUBLE)
        return 0.0625; // min size of L3 = 64K
      else
        return 0.0;
    }

  } // namespace arch

} // namespace cmpex

#endif // _ARCH_ARCHPLANNER_H_
