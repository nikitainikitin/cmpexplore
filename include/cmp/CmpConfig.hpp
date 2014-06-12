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
	bool active; // active or off
        double latency; // latency unit is [ns]
        double eacc; // access energy
	double pidle; // idle dynamic power
        double pleak; // leakage power
        double lambda; // runtime parameter: number of accesses per ns
        double bufDelay; // contention delay in the input queue
        MemCtrl (std::string n, bool a, double l, double e, double pi, double p) :
          name (n), active (a), latency (l), eacc (e), pidle (pi), pleak (p), lambda (0.0) {}
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
      
      inline double UFreq () const;

      inline void UFreq ( double f );

      inline double UFreqMax () const;

      inline void UFreqMax ( double f );

      inline double UVolt () const;

      inline void UVolt ( double v );

      inline double UVoltMin () const;

      inline void UVoltMin ( double v );

      inline double UVoltMax () const;

      inline void UVoltMax ( double v );

      inline double UVoltNom () const;

      inline void UVoltNom ( double v );

      inline double McFreq () const;

      inline void McFreq ( double f );

      inline double McFreqMax () const;

      inline void McFreqMax ( double f );

      inline double McVolt () const;

      inline void McVolt ( double v );

      inline double McVoltMax () const;

      inline void McVoltMax ( double v );

      inline double McVoltMin () const;

      inline void McVoltMin ( double v );

      inline double McVoltNom () const;

      inline void McVoltNom ( double v );

      inline double ProcFreqMax () const;

      inline void ProcFreqMax ( double f );

      inline double ProcVoltMax () const;

      inline void ProcVoltMax ( double v );

      inline double ProcVoltMin () const;

      inline void ProcVoltMin ( double v );

      inline double ProcVoltNom () const;

      inline void ProcVoltNom ( double v );

      inline double ProcMinVoltFreq () const;

      inline void ProcMinVoltFreq ( double f );

      inline void AddMemCtrl ( const std::string& n, bool a, double l, double e, double pi, double p );

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
      
      inline double NiDelayNs () const;

      inline int L3ClusterSize () const;

      inline void L3ClusterSize ( int c );

      inline int L3ClusterCnt () const;

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

      inline IdxArray& GetTilesOfL3Cluster (int clIdx);

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

      // Initialize lists of tiles per L3 cluster and assign cluster idxs to memories
      void InitL3Clusters();

      void Cleanup();

      // --- DEBUG ---
      void PrintL3Clusters() const;
      
    private:

      /*** ============== cmp configuration ================ ***/

      Component * cmp_;
      
      double unitLen_; // unit of length (in meters)
      
      double memDensity_; // area of 1Mb of memory (in units^2)
      
      double uFreq_; // uncore frequency (NoC & shared caches) [GHz]

      double uFreqMax_; // uncore max frequency (NoC & shared caches) [GHz]

      double uVolt_; // uncore voltage (NoC & shared caches) [V]

      double uVoltMax_; // uncore max voltage (NoC & shared caches) [V]

      double uVoltMin_; // uncore min voltage (NoC & shared caches) [V]

      double uVoltNom_; // uncore nom voltage (NoC & shared caches) [V]

      double mcFreq_; // frequency of MCs [GHz]

      double mcFreqMax_; // max frequency of MCs [GHz]

      double mcVolt_; // MCs voltage [V]

      double mcVoltMax_; // MCs max voltage [V]

      double mcVoltMin_; // MCs min voltage [V]

      double mcVoltNom_; // MCs nom voltage [V]

      double procFreqMax_; // max frequency of Processors [GHz]

      double procVoltMax_; // Processors max voltage [V]

      double procVoltMin_; // Processors min voltage [V]

      double procVoltNom_; // Processors nom voltage [V]

      double procMinVoltFreq_; // max frequency of Processors at min voltage [GHz]

      std::vector<MemCtrl> memCtrl_; // list of controllers
      
      ProcArray processors_; // list of processors
      
      MemArray memories_; // list of memories
      
      ClusterArray clusters_; // list of clusters
      
      double l3Latency_; // default L3 access latency (in cycles)
      
      UShort memReplySize_; // coefficient of size of the memory reply packet
      
      double niDelay_; // simulation delay of network interface upon injection/ejection
      
      int L3ClusterSize_; // defines how many L3 slices that can be accessed by every core.
                          // When == 0, every core can access all L3 slices on chip.
                          // Note: an L3 cluster is different from cmp:Cluster.
                          // For more info check CmpConfig::InitProcL3ProbDistr().

      vector<IdxArray> L3Clusters_; // IdxArrays with the indices of tiles per L3 cluster

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
    
    double CmpConfig::UFreq () const {
      return uFreq_;
    }

    void CmpConfig::UFreq ( double f ) {
      uFreq_ = f;
    }

    double CmpConfig::UFreqMax () const {
      return uFreqMax_;
    }

    void CmpConfig::UFreqMax ( double f ) {
      uFreqMax_ = f;
    }

    double CmpConfig::UVolt () const {
      return uVolt_;
    }

    void CmpConfig::UVolt ( double v ) {
      uVolt_ = v;
    }

    double CmpConfig::UVoltMin () const {
      return uVoltMin_;
    }

    void CmpConfig::UVoltMin ( double v ) {
      uVoltMin_ = v;
    }

    double CmpConfig::UVoltMax () const {
      return uVoltMax_;
    }

    void CmpConfig::UVoltMax ( double v ) {
      uVoltMax_ = v;
    }

    double CmpConfig::UVoltNom () const {
      return uVoltNom_;
    }

    void CmpConfig::UVoltNom ( double v ) {
      uVoltNom_ = v;
    }

    double CmpConfig::McFreq () const {
      return mcFreq_;
    }

    void CmpConfig::McFreq ( double f ) {
      mcFreq_ = f;
    }

    double CmpConfig::McFreqMax () const {
      return mcFreqMax_;
    }

    void CmpConfig::McFreqMax ( double f ) {
      mcFreqMax_ = f;
    }

    double CmpConfig::McVolt () const {
      return mcVolt_;
    }

    void CmpConfig::McVolt ( double v ) {
      mcVolt_ = v;
    }

    double CmpConfig::McVoltMax () const {
      return mcVoltMax_;
    }

    void CmpConfig::McVoltMax ( double v ) {
      mcVoltMax_ = v;
    }

    double CmpConfig::McVoltMin () const {
      return mcVoltMin_;
    }

    void CmpConfig::McVoltMin ( double v ) {
      mcVoltMin_ = v;
    }

    double CmpConfig::McVoltNom () const {
      return mcVoltNom_;
    }

    void CmpConfig::McVoltNom ( double v ) {
      mcVoltNom_ = v;
    }

    double CmpConfig::ProcFreqMax () const {
      return procFreqMax_;
    }

    void CmpConfig::ProcFreqMax ( double f ) {
      procFreqMax_ = f;
    }

    double CmpConfig::ProcVoltMax () const {
      return procVoltMax_;
    }

    void CmpConfig::ProcVoltMax ( double v ) {
      procVoltMax_ = v;
    }

    double CmpConfig::ProcVoltMin () const {
      return procVoltMin_;
    }

    void CmpConfig::ProcVoltMin ( double v ) {
      procVoltMin_ = v;
    }

    double CmpConfig::ProcVoltNom () const {
      return procVoltNom_;
    }

    void CmpConfig::ProcVoltNom ( double v ) {
      procVoltNom_ = v;
    }

    double CmpConfig::ProcMinVoltFreq () const {
      return procMinVoltFreq_;
    }

    void CmpConfig::ProcMinVoltFreq ( double f ) {
      procMinVoltFreq_ = f;
    }

    void CmpConfig::AddMemCtrl ( const std::string& n, bool a, double l, double e, double pi, double p ) {
      memCtrl_.push_back(MemCtrl(n,a,l,e,pi,p));
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
    
    double CmpConfig::NiDelayNs () const {
      return niDelay_/UFreq();
    }

    int CmpConfig::L3ClusterSize () const {
      return L3ClusterSize_;
    }

    void CmpConfig::L3ClusterSize ( int s ) {
      L3ClusterSize_ = s;
    }

    int CmpConfig::L3ClusterCnt () const {
      return L3Clusters_.size();
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

    IdxArray& CmpConfig::GetTilesOfL3Cluster (int clIdx) {
      DASSERT(clIdx < L3Clusters_.size());
      return L3Clusters_[clIdx];
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
