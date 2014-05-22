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

#ifndef _CMP_PROCESSOR_H_
#define _CMP_PROCESSOR_H_

#include <vector>
#include <list>

#include "Device.hpp"
#include "CmpConfig.hpp"
#include "CmpBuilder.hpp"
#include "Config.hpp"

using std::vector;

namespace cmpex {
  
  extern cmp::CmpConfig cmpConfig;
  extern cmp::CmpBuilder cmpBuilder;
  extern Config config;

  namespace cmp {

    //======================================================================
    // This class represents a processor component of the Cmp.
    //======================================================================

    class Processor : public Device {
      
      friend class CmpBuilder;

      // ---------------------------- Methods ------------------------------

    public:
      
      // Constructors and destructor
      Processor ( UShort idx, UShort clIdx, Component * parent = 0 );
      
      virtual ~Processor ();
      
      // Accessors
      inline double L1Size() const;
      
      inline int L1Lat() const;

      inline double L2Size() const;
      
      inline int L2Lat() const;

      inline double L3SizeEff() const;

      inline double Ipc () const;

      inline int OoO () const;

      inline double Area () const;

      inline double Mpi () const;

      inline double Freq() const;

      inline double Volt() const;

      inline double Thr () const;

      inline void Thr (double t);

      inline double Lambda () const;

      inline void Lambda (double l);

      inline int Type () const;

      inline bool Active () const;

      inline bool Sleep () const;

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
      
      // Processor-specific methods
      
      // Miss-ratio law
      inline double L1AccessProbability() const;
      
      inline double L2AccessProbability() const;
      
      inline double L3AccessProbability() const;

      inline double MainMemAccessProbability() const;

      inline const vector<double>& L3ProbDistr() const;

      inline vector<double>& L3ProbDistr();

      // Power
      inline double Epi() const;

      inline double Pidle() const;
      
      inline double Pleak() const;

      inline double CorePleakOfTemp(double tmp) const;
      
      inline double CorePgPleakOfTemp(double tmp) const;
      
      inline double Pgated() const;
      
      inline double L1Eacc() const;

      inline double L1Pleak() const;

      inline double L1PleakOfTemp(double tmp) const;

      inline double L1PgPleakOfTemp(double tmp) const;

      inline double L1IPleakOfTemp(double tmp) const;

      inline double L1IPgPleakOfTemp(double tmp) const;

      inline double L1DPleakOfTemp(double tmp) const;

      inline double L1DPgPleakOfTemp(double tmp) const;

      inline double L2Eacc() const;

      inline double L2Pleak() const;

      inline double L2PleakOfTemp(double tmp) const;

      inline double L2PgPleakOfTemp(double tmp) const;

      inline void SetL1Size ( double s );

      inline void SetL1Lat ( int l );

      inline void SetL2Size ( double s );

      inline void SetL2Lat ( int l );

      inline void SetL3SizeEff ( double s );

      inline void SetOoO ( int o );

      inline void SetArea ( double a );

      inline void SetActive ( bool a );

      inline void SetSleep ( bool s );

      inline void SetIpc ( double i );

      inline void SetMpi ( double m );

      inline void SetL1AccProb(double p);
      
      inline void SetL2AccProb(double p);

      inline void SetL3AccProb(double p);

      inline void SetMMAccProb(double p);

      inline void SetFreq(double f);

      inline void SetVolt(double v);

      inline void SetEpi(double e);

      inline void SetPleak(double p);

      inline void SetPidle(double p);

      inline void SetPgated(double p);

      inline void SetL1Eacc(double e);

      inline void SetL1Pleak(double p);

      inline void SetL2Eacc(double e);

      inline void SetL2Pleak(double p);

      inline void SetType(int t);

      inline void SetMemAccessProbabilities ( model::Function * mr );

      inline double L1IEa () const;

      inline void SetL1IEa ( double e );

      inline double L1IEm () const;

      inline void SetL1IEm ( double e );

      inline double L1DErda () const;

      inline void SetL1DErda ( double e );

      inline double L1DErdm () const;

      inline void SetL1DErdm ( double e );

      inline double L1DEwra () const;

      inline void SetL1DEwra ( double e );

      inline double L1DEwrm () const;

      inline void SetL1DEwrm ( double e );

      inline double L2Erda () const;

      inline void SetL2Erda ( double e );

      inline double L2Erdm () const;

      inline void SetL2Erdm ( double e );

      inline double L2Ewra () const;

      inline void SetL2Ewra ( double e );

      inline double L2Ewrm () const;

      inline void SetL2Ewrm ( double e );

    private:

      // Deprecated methods: prevent usage
      Processor ( const Processor& );

      Processor& operator = ( const Processor& );

      // -------------------------- Attributes -----------------------------

    private:
      
      double l1Size_; // size of L1 cache in Mb
      
      int l1Lat_; // L1 cache latency

      double l2Size_; // size of L2 cache in Mb
      
      int l2Lat_; // L2 cache latency

      double l3SizeEff_; // effective size of L3 in Mb

      int ooo_; // Out-of-order architecture (1 - yes, 0 - no)

      double area_; // processor area

      /// The ipc, mpi and cache probabilities are used ONLY in the mapping mode.
      /// In the exploration mode those are obtained from the current workload.

      double ipc_; // # of instructions per second

      double mpi_; // # of memory references per instruction

      double l1AccProb_;

      double l2AccProb_;

      double l3AccProb_;

      double mmAccProb_;

      vector<double> l3ProbDistr_; // prob. to access distributed L3 p/cluster

      double freq_; // frequency/voltage come as input-defined pairs

      double volt_;

      int type_; // type index

      bool active_; // whether the core is on or off

      bool sleep_; // whether the core is asleep or not

      // Power parameters

      double epi_; // energy per instruction

      double pleak_; // leakage power

      double pidle_; // leakage power

      double pgated_; // leakage power

      double l1Eacc_; // l1 access energy

      double l1Pleak_; // l1 leakage power

      double l1iEa_; // l1 instruction cache access energy 

      double l1iEm_; // l1 instruction cache additional miss access energy 

      double l1dErda_; // l1 data cache read access energy 

      double l1dErdm_; // l1 data cache additional read  miss access energy 

      double l1dEwra_; // l1 data cache write access energy 

      double l1dEwrm_; // l1 data cache additional write miss access energy 

      double l2Eacc_; // l2 access energy

      double l2Pleak_; // l2 leakage power

      double l2Erda_; // l2 read access energy 

      double l2Erdm_; // l2 additional read  miss access energy 

      double l2Ewra_; // l2 write access energy 

      double l2Ewrm_; // l2 additional write miss access energy 

      // Runtime parameters

      double thr_; // real throughput, instructions per ns

      double lambda_; // number of memory requests per ns
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    double Processor::L1Size () const {
      return l1Size_;
    }
    
    int Processor::L1Lat () const {
      return l1Lat_;
    }

    void Processor::SetL1Size ( double s ) {
      l1Size_ = s;
    }
    
    void Processor::SetL1Lat ( int l ) {
      l1Lat_ = l;
    }

    double Processor::L2Size () const {
      return l2Size_;
    }
    
    int Processor::L2Lat () const {
      return l2Lat_;
    }

    void Processor::SetL2Size ( double s ) {
      l2Size_ = s;
    }
    
    void Processor::SetL2Lat ( int l ) {
      l2Lat_ = l;
    }

    double Processor::L3SizeEff () const {
      return l3SizeEff_;
    }

    void Processor::SetL3SizeEff ( double s ) {
      l3SizeEff_ = s;
    }

    int Processor::OoO () const {
      return ooo_;
    }

    void Processor::SetOoO ( int o ) {
      ooo_ = o;
    }

    double Processor::Area () const {
      return area_;
    }

    void Processor::SetArea ( double a ) {
      area_ = a;
    }

    double Processor::Ipc () const {
      if (config.Tmap())
        return ipc_;
      else
        return cmpConfig.GetCurWlIpc(Type());
    }

    void Processor::SetIpc ( double i ) {
      ipc_ = i;
    }

    double Processor::Mpi () const {
      if (config.Tmap())
        return mpi_;
      else
        return cmpConfig.GetCurWlMpi(Type());
    }
    
    void Processor::SetMpi ( double m ) {
      mpi_ = m;
    }
    
    void Processor::SetL1AccProb(double p) {
      l1AccProb_ = p;
    }

    void Processor::SetL2AccProb(double p) {
      l2AccProb_ = p;
    }

    void Processor::SetL3AccProb(double p) {
      l3AccProb_ = p;
    }

    void Processor::SetMMAccProb(double p) {
      mmAccProb_ = p;
    }

    double Processor::L1AccessProbability() const {
      if (config.Tmap())
        return l1AccProb_;
      else
        return 1.0 - cmpConfig.EvalCurWlMissRate(L1Size());
    }

    double Processor::L2AccessProbability() const {
      if (config.Tmap())
        return l2AccProb_;
      else
        return cmpConfig.EvalCurWlMissRate(L1Size()) -
               cmpConfig.EvalCurWlMissRate(L2Size()+L1Size());
    }

    double Processor::L3AccessProbability() const {
      if (config.Tmap())
        return l3AccProb_;
      else
        return (L3SizeEff() < E_DOUBLE) ? 0.0 :
               (cmpConfig.EvalCurWlMissRate(L2Size()+L1Size()) -
                cmpConfig.EvalCurWlMissRate(L3SizeEff()+L2Size()+L1Size()));
    }

    double Processor::MainMemAccessProbability() const {
      if (config.Tmap())
        return mmAccProb_;
      else
        return (L3SizeEff() < E_DOUBLE) ?
               cmpConfig.EvalCurWlMissRate(L2Size()+L1Size()) :
               cmpConfig.EvalCurWlMissRate(L3SizeEff()+L2Size()+L1Size());
    }

    void Processor::SetMemAccessProbabilities ( model::Function * mr ) {
      SetL1AccProb(1.0 - mr->eval(L1Size()));
      SetL2AccProb(mr->eval(L1Size()) - mr->eval(L2Size()+L1Size()));
      SetL3AccProb(L3SizeEff() < E_DOUBLE ? 0.0 :
                   (mr->eval(L2Size()+L1Size()) -
                    mr->eval(L3SizeEff()+L2Size()+L1Size())));
      SetMMAccProb(L3SizeEff() < E_DOUBLE ?
                    mr->eval(L2Size()+L1Size()) :
                    mr->eval(L3SizeEff()+L2Size()+L1Size()));
    }

    const vector<double>& Processor::L3ProbDistr() const {
      return l3ProbDistr_;
    }

    vector<double>& Processor::L3ProbDistr() {
      return l3ProbDistr_;
    }

    double Processor::Thr() const {
      return thr_;
    }

    void Processor::Thr(double t) {
      thr_ = t;
    }

    double Processor::Lambda() const {
      return lambda_;
    }

    void Processor::Lambda(double l) {
      lambda_ = l;
    }

    double Processor::Freq () const {
      return freq_;
    }

    void Processor::SetFreq ( double f ) {
      freq_ = f;
    }

    double Processor::Volt () const {
      return volt_;
    }

    void Processor::SetVolt ( double v ) {
      volt_ = v;
    }

    double Processor::Epi () const {
      return epi_;
    }

    void Processor::SetEpi ( double e ) {
      epi_ = e;
    }

    double Processor::Pleak () const {
      return pleak_;
    }

    double Processor::CorePleakOfTemp ( double tmp ) const {
      return cmpBuilder.CoreLeakageOfTemp(tmp);
    }

    double Processor::CorePgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.CorePgLeakageOfTemp(tmp);
    }

    void Processor::SetPleak ( double p ) {
      pleak_ = p;
    }

    double Processor::Pidle () const {
      return pidle_;
    }

    void Processor::SetPidle ( double p ) {
      pidle_ = p;
    }

    double Processor::Pgated () const {
      return pgated_;
    }

    void Processor::SetPgated ( double p ) {
      pgated_ = p;
    }

    double Processor::L1Eacc () const {
      return l1Eacc_;
    }

    void Processor::SetL1Eacc ( double e ) {
      l1Eacc_ = e;
    }

    double Processor::L1IEa () const {
      return l1iEa_;
    }

    void Processor::SetL1IEa ( double e ) {
      l1iEa_ = e;
    }

    double Processor::L1IEm () const {
      return l1iEm_;
    }

    void Processor::SetL1IEm ( double e ) {
      l1iEm_ = e;
    }

    double Processor::L1DErda () const {
      return l1dErda_;
    }

    void Processor::SetL1DErda ( double e ) {
      l1dErda_ = e;
    }

    double Processor::L1DErdm () const {
      return l1dErdm_;
    }

    void Processor::SetL1DErdm ( double e ) {
      l1dErdm_ = e;
    }

    double Processor::L1DEwra () const {
      return l1dEwra_;
    }

    void Processor::SetL1DEwra ( double e ) {
      l1dEwra_ = e;
    }

    double Processor::L1DEwrm () const {
      return l1dEwrm_;
    }

    void Processor::SetL1DEwrm ( double e ) {
      l1dEwrm_ = e;
    }

    double Processor::L2Erda () const {
      return l2Erda_;
    }

    void Processor::SetL2Erda ( double e ) {
      l2Erda_ = e;
    }

    double Processor::L2Erdm () const {
      return l2Erdm_;
    }

    void Processor::SetL2Erdm ( double e ) {
      l2Erdm_ = e;
    }

    double Processor::L2Ewra () const {
      return l2Ewra_;
    }

    void Processor::SetL2Ewra ( double e ) {
      l2Ewra_ = e;
    }

    double Processor::L2Ewrm () const {
      return l2Ewrm_;
    }

    void Processor::SetL2Ewrm ( double e ) {
      l2Ewrm_ = e;
    }

    double Processor::L1Pleak () const {
      return l1Pleak_;
    }

    void Processor::SetL1Pleak ( double p ) {
      l1Pleak_ = p;
    }

    double Processor::L1PleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1LeakageOfTemp(tmp);
    }

    double Processor::L1PgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1PgLeakageOfTemp(tmp);
    }

    double Processor::L1IPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1ILeakageOfTemp(tmp);
    }

    double Processor::L1IPgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1IPgLeakageOfTemp(tmp);
    }

    double Processor::L1DPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1DLeakageOfTemp(tmp);
    }

    double Processor::L1DPgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L1DPgLeakageOfTemp(tmp);
    }

    double Processor::L2Eacc () const {
      return l2Eacc_;
    }

    void Processor::SetL2Eacc ( double e ) {
      l2Eacc_ = e;
    }

    double Processor::L2Pleak () const {
      return l2Pleak_;
    }

    void Processor::SetL2Pleak ( double p ) {
      l2Pleak_ = p;
    }

    double Processor::L2PleakOfTemp ( double tmp ) const {
      return cmpBuilder.L2LeakageOfTemp(tmp);
    }

    double Processor::L2PgPleakOfTemp ( double tmp ) const {
      return cmpBuilder.L2PgLeakageOfTemp(tmp);
    }

    int Processor::Type () const {
      return type_;
    }

    void Processor::SetType ( int t ) {
      type_ = t;
    }

    bool Processor::Active () const {
      return active_;
    }

    void Processor::SetActive ( bool a ) {
      active_ = a;
    }

    bool Processor::Sleep () const {
      return sleep_;
    }

    void Processor::SetSleep ( bool s ) {
      sleep_ = s;
    }

  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_PROCESSOR_H_
