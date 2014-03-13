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

#ifndef _ARCH_ARCHCONFIG_H_
#define _ARCH_ARCHCONFIG_H_

#include <string>
#include <vector>
#include <sstream>

#include "Defs.hpp"

using std::string;
using std::vector;
using std::stringstream;
using std::istream;
using std::pair;

namespace cmpex {

  namespace arch {

    //======================================================================
    // ArchConfig represents architectural description of a cmp. It includes
    // the full stream-based cmp description, and general parameters
    // (such as the number of processors and mesh dimensions, etc).
    //======================================================================

    class ArchConfig {

    public:

      typedef vector< pair<string, string> > ParamVector;

      typedef ParamVector::const_iterator PCIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ArchConfig ();

      virtual ~ArchConfig ();
      
      // Accessors
      inline const istream& IStream() const;

      inline istream& IStream();

      inline string ToString() const;

      template<typename T>
      inline stringstream& operator << (T& t);
      
      template<typename T>
      inline stringstream& operator >> (T& t);

      inline const ParamVector& Params() const;

      inline void AddParam(const string& name, const string& value);

      inline UInt GMeshDimX () const;

      inline void GMeshDimX (UInt x);

      inline UInt GMeshDimY () const;

      inline void GMeshDimY (UInt y);

      inline const string& ClusterIcType () const;

      inline void ClusterIcType(const string& t);

      inline const vector<int>& ProcCnt() const;

      inline vector<int>& ProcCnt();

      inline const vector<double>& ProcL1Size() const;

      inline vector<double>& ProcL1Size();

      inline const vector<double>& ProcL2Size() const;

      inline vector<double>& ProcL2Size();

      inline UInt NumL3 () const;

      inline void NumL3 (UInt n);

      inline double Area () const;

      inline void Area (double a);

    private:

      // Deprecated methods: prevent usage
      ArchConfig ( const ArchConfig& );

      ArchConfig& operator = ( const ArchConfig& );

      // -------------------------- Attributes -----------------------------

    private:

      // configuration description
      stringstream ios_;

      // string parameters
      ParamVector params_;

      // value parameters

      // The following two values are the global mesh dimensions.
      UInt gMeshDimX_;

      UInt gMeshDimY_;

      // Cluster IC types
      string clusterIcType_;

      // The number of processors of every type in the cluster.
      // procCnt_.size() is equal to the config.ProcCnt().
      vector<int> procCnt_;

      // L1 size per processor of every type.
      // procL1Size_.size() is equal to the config.ProcCnt().
      vector<double> procL1Size_;

      // L2 size per processor of every type.
      // procL2Size_.size() is equal to the config.ProcCnt().
      vector<double> procL2Size_;

      // The number of L3 memories per cluster.
      UInt numL3_;

      // config area (may be reduced because of the adjusted L3);
      double area_;
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const istream& ArchConfig::IStream() const {
      return ios_;
    }

    istream& ArchConfig::IStream() {
      return ios_;
    }

    string ArchConfig::ToString() const {
      return ios_.str();
    }

    const ArchConfig::ParamVector& ArchConfig::Params() const {
      return params_;
    }

    void ArchConfig::AddParam(const string& name, const string& value) {
      params_.push_back(make_pair(name, value));
    }

    inline UInt ArchConfig::GMeshDimX () const {
      return gMeshDimX_;
    }

    inline void ArchConfig::GMeshDimX (UInt x) {
      gMeshDimX_ = x;
    }

    inline UInt ArchConfig::GMeshDimY () const {
      return gMeshDimY_;
    }

    inline void ArchConfig::GMeshDimY (UInt y) {
      gMeshDimY_ = y;
    }

    inline const string& ArchConfig::ClusterIcType () const {
      return clusterIcType_;
    }

    inline void ArchConfig::ClusterIcType(const string& t) {
      clusterIcType_ = t;
    }

    inline const vector<int>& ArchConfig::ProcCnt() const {
      return procCnt_;
    }

    inline vector<int>& ArchConfig::ProcCnt() {
      return procCnt_;
    }

    inline const vector<double>& ArchConfig::ProcL1Size() const {
      return procL1Size_;
    }

    inline vector<double>& ArchConfig::ProcL1Size() {
      return procL1Size_;
    }

    inline const vector<double>& ArchConfig::ProcL2Size() const {
      return procL2Size_;
    }

    inline vector<double>& ArchConfig::ProcL2Size() {
      return procL2Size_;
    }

    inline UInt ArchConfig::NumL3 () const {
      return numL3_;
    }

    inline void ArchConfig::NumL3 (UInt n) {
      numL3_ = n;
    }

    inline double ArchConfig::Area () const {
      return area_;
    }

    inline void ArchConfig::Area (double a) {
      area_ = a;
    }

    //----------------------------------------------------------------------
    // Template functions
    //----------------------------------------------------------------------

    template<typename T>
    stringstream& ArchConfig::operator << (T& t) {
      ios_ << t;
      return ios_;
    }
    
    template<typename T>
    stringstream& ArchConfig::operator >> (T& t) {
      ios_ >> t;
      return ios_;
    }

  } // namespace arch

} // namespace cmpex

#endif // _ARCH_ARCHCONFIG_H_
