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

#ifndef _CMP_CONFIG_H_
#define _CMP_CONFIG_H_

#include <string>
#include <vector>
#include <iostream>

#include "Defs.hpp"
#include "model/Function.hpp"

using std::string;

namespace cmpex {

  namespace arch {
    class ArchConfig;
  }
  
  namespace model {
    class Function;
  }

  namespace cmp {

    class Component;
    class Processor;
    class Memory;
    class Cluster;
    
    //======================================================================
    // Config is a storage for cmp-specific parameters.
    //======================================================================

    class CmpConfig
    {

      friend class CmpBuilder;
      
      typedef std::vector<Processor*> ProcArray;
      
      typedef ProcArray::const_iterator PCIter;
      
      typedef std::vector<Memory*> MemArray;
      
      typedef MemArray::const_iterator MCIter;
      
      typedef std::vector<Cluster*> ClusterArray;
      
      typedef ClusterArray::const_iterator ClCIter;

    public:
      
      // Comment: memory controllers in our representation
      // are passive objects, hence they're prepresented as a part of
      // CmpConfig, rather than the real components. This may need to be changed,
      // once MC placement has to be investigated.
      struct MemCtrl {
        std::string name;
        double latency;
        double eacc; // access energy
        double pleak; // leakage power
        double lambda; // runtime parameter: probability of access per cycle
        double bufDelay; // contention delay in the input queue
        MemCtrl (std::string n, double l, double e, double p) :
          name (n), latency (l), eacc (e), pleak (p), lambda (0.0) {}
      };

      // Data structure for a workload application.
      struct Workload {
        ~Workload() { delete missRatioOfMemSize; }
        std::string name;
        std::string shortName;
        model::Function * missRatioOfMemSize;
        std::vector<double> ipc; // per type of core
        std::vector<double> mpi; // per type of core
      };
      
      typedef std::vector<Workload*> WlArray;

      typedef WlArray::const_iterator WlCIter;

      // ---------------------------- Methods ------------------------------

    public:

      CmpConfig ( void );

      ~CmpConfig ();
      
      // Accessors
      
      inline const Component * Cmp () const;

      inline Component * Cmp ();

      inline void SetCmp ( Component * c );

      inline double UnitLen () const;

      inline void UnitLen ( double l );

      inline double MemDensity () const;

      inline void MemDensity ( double d );
      
      inline double Freq () const;

      inline void Freq ( double f );

      inline double McFreq () const;

      inline void McFreq ( double f );

      inline void AddMemCtrl ( const std::string& n, double l, double e, double p );

      inline int MemCtrlCnt () const;

      inline const MemCtrl * GetMemCtrl ( int idx ) const;

      inline MemCtrl * GetMemCtrl ( int idx );

      inline int ProcCnt () const;

      inline int MemCnt () const;
      
      inline int ClusterCnt () const;
      
      inline const Processor * GetProcessor ( int idx ) const;

      inline Processor * GetProcessor ( int idx );

      inline const Memory * GetMemory ( int idx ) const;
      
      inline Memory * GetMemory ( int idx );
      
      inline Cluster * GetCluster ( int idx );
      
      inline void AddProcessor ( Processor * p );

      inline void AddMemory ( Memory * m );

      inline void AddCluster ( Cluster * c );

      inline double L3Latency () const;

      inline void L3Latency ( double l );
      
      inline UShort MemReplySize () const;

      inline void MemReplySize ( UShort s );
      
      inline double NiDelay () const;

      inline void NiDelay ( double d );
      
      inline double TotalL3Size () const;
      
      inline UInt SubnCnt () const;

      inline void SubnCnt ( UInt n );

      inline const Workload * GetWorkload ( int idx ) const;

      inline const std::string& GetCurWlName () const;

      inline const std::string& GetCurWlShortName () const;

      inline double EvalCurWlMissRate ( double sz ) const;

      inline double GetCurWlIpc ( int procType ) const;

      inline double GetCurWlMpi ( int procType ) const;

      inline void AddWorkload ( Workload * w );

      inline int GetWlCnt () const;

      inline const std::string& WlFile () const;

      inline void WlFile ( const std::string& f );

      inline bool FlatMeshIc () const;

      inline void FlatMeshIc ( bool f );

      inline int WlIdx () const;

      inline void SetWlIdx ( int i );

      // Service functions
      // Get total size of the L3 memory.
      void CalcTotalL3Size ();
      
      // Create cmp from file.
      int CreateCmp (const std::string& fname);

      // Create cmp from architectural configuration.
      int CreateCmp (arch::ArchConfig& ac);

      // Converts architectural file to simulator-compatible
      // workload + architecture files
      void WriteWorkloadArchFromArch (const std::string& fname);

      // The procedure sets ownership for composite components.
      void InitComponentOwnership ();

      // The procedure sets mapping from L3 to MC
      void InitL3ToMcMapping();

      // Initialize runtime properties
      void InitRuntimeProperties();

      // Initialize probabilities of procs to access L3 instances.
      void InitProcL3ProbDistr();

      void Cleanup();
      
    private:

      /*** ============== cmp configuration ================ ***/

      Component * cmp_;
      
      double unitLen_; // unit of length (in meters)
      
      double memDensity_; // area of 1Mb of memory (in units^2)
      
      double freq_; // master clock frequency (NoC & caches) [GHz]

      double mcFreq_; // frequency of MCs [GHz]

      std::vector<MemCtrl> memCtrl_; // list of controllers
      
      ProcArray processors_; // list of processors
      
      MemArray memories_; // list of memories
      
      ClusterArray clusters_; // list of clusters
      
      double l3Latency_; // default L3 access latency (in cycles)
      
      UShort memReplySize_; // coefficient of size of the memory reply packet
      
      double niDelay_; // simulation delay of network interface upon injection/ejection
      
      double totalL3Size_;

      // number of physical subnetworks in the IC:
      // equal to 3 if simulating cache coherence protocol, 1 otherwise
      UInt subnCnt_;

      WlArray workloads_; // list of workloads

      std::string wlFile_; // current workload file (for caching/speedup)

      //////////////// Runtime global parameters for speedup /////////////////

      bool flatMeshIc_; // true for CMPs with flat mesh IC (no hierarchy)

      int wlIdx_; // index of current workload
      
    };
  
    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    const Component * CmpConfig::Cmp () const {
      return cmp_;
    }

    Component * CmpConfig::Cmp () {
      return cmp_;
    }

    void CmpConfig::SetCmp ( Component * c ) {
      cmp_ = c;
    }

    double CmpConfig::UnitLen () const {
      return unitLen_;
    }
    
    void CmpConfig::UnitLen ( double l ) {
      unitLen_ = l;
    }
    
    double CmpConfig::MemDensity () const {
      return memDensity_;
    }
    
    void CmpConfig::MemDensity ( double d ) {
      memDensity_ = d;
    }
    
    double CmpConfig::Freq () const {
      return freq_;
    }

    void CmpConfig::Freq ( double f ) {
      freq_ = f;
    }

    double CmpConfig::McFreq () const {
      return mcFreq_;
    }

    void CmpConfig::McFreq ( double f ) {
      mcFreq_ = f;
    }

    void CmpConfig::AddMemCtrl ( const std::string& n, double l, double e, double p ) {
      memCtrl_.push_back(MemCtrl(n,l,e,p));
    }

    int CmpConfig::MemCtrlCnt () const {
      return memCtrl_.size();
    }
    
    const CmpConfig::MemCtrl * CmpConfig::GetMemCtrl (int idx) const {
      return &memCtrl_[idx];
    }
    
    CmpConfig::MemCtrl * CmpConfig::GetMemCtrl (int idx) {
      return &memCtrl_[idx];
    }

    int CmpConfig::ProcCnt () const {
      return processors_.size();
    }
    
    int CmpConfig::MemCnt () const {
      return memories_.size();
    }
    
    int CmpConfig::ClusterCnt () const {
      return clusters_.size();
    }
    
    const Processor * CmpConfig::GetProcessor ( int idx ) const {
      return processors_[idx];
    }
    
    Processor * CmpConfig::GetProcessor ( int idx ) {
      return processors_[idx];
    }
    
    const Memory * CmpConfig::GetMemory ( int idx ) const {
      return memories_[idx];
    }
    
    Memory * CmpConfig::GetMemory ( int idx ) {
      return memories_[idx];
    }
    
    Cluster * CmpConfig::GetCluster ( int idx ) {
      return clusters_[idx];
    }
    
    void CmpConfig::AddProcessor ( Processor * p ) {
      processors_.push_back(p);
    }
    
    void CmpConfig::AddMemory ( Memory * m ) {
      memories_.push_back(m);
    }
    
    void CmpConfig::AddCluster ( Cluster * c ) {
      clusters_.push_back(c);
    }
    
    double CmpConfig::L3Latency () const {
      return l3Latency_;
    }
    
    void CmpConfig::L3Latency ( double l ) {
      l3Latency_ = l;
    }
    
    UShort CmpConfig::MemReplySize () const {
      return memReplySize_;
    }
    
    void CmpConfig::MemReplySize ( UShort s ) {
      memReplySize_ = s;
    }
    
    double CmpConfig::NiDelay () const {
      return niDelay_;
    }
    
    void CmpConfig::NiDelay ( double d ) {
      niDelay_ = d;
    }
    
    double CmpConfig::TotalL3Size () const {
      return totalL3Size_;
    }
    
    UInt CmpConfig::SubnCnt () const {
      return subnCnt_;
    }

    void CmpConfig::SubnCnt ( UInt n ) {
      subnCnt_ = n;
    }

    const CmpConfig::Workload * CmpConfig::GetWorkload ( int idx ) const {
      return workloads_[idx];
    }

    void CmpConfig::AddWorkload ( CmpConfig::Workload * w ) {
      workloads_.push_back(w);
    }

    int CmpConfig::GetWlCnt () const {
      return workloads_.size();
    }

    const std::string& CmpConfig::WlFile() const {
      return wlFile_;
    }

    void CmpConfig::WlFile ( const std::string& f ) {
      wlFile_ = f;
    }

    bool CmpConfig::FlatMeshIc () const {
      return flatMeshIc_;
    }

    void CmpConfig::FlatMeshIc ( bool f ) {
      flatMeshIc_ = f;
    }

    int CmpConfig::WlIdx () const {
      return wlIdx_;
    }

    void CmpConfig::SetWlIdx ( int i ) {
      wlIdx_ = i;
    }

    const std::string& CmpConfig::GetCurWlName () const {
      return GetWorkload(WlIdx())->name;
    }

    const std::string& CmpConfig::GetCurWlShortName () const {
      return GetWorkload(WlIdx())->shortName;
    }

    double CmpConfig::EvalCurWlMissRate ( double sz ) const {
      return GetWorkload(WlIdx())->missRatioOfMemSize->eval(sz);
    }

    double CmpConfig::GetCurWlIpc ( int procType ) const {
      return GetWorkload(WlIdx())->ipc[procType];
    }

    double CmpConfig::GetCurWlMpi ( int procType ) const {
      return GetWorkload(WlIdx())->mpi[procType];
    }

  } // namespace cmp

} // namespace cmpex

#endif // _CMP_CONFIG_H_
