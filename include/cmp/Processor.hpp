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

using std::vector;

namespace cmpex {
  
  extern cmp::CmpConfig cmpConfig;

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

      inline double Thr () const;

      inline void Thr (double t);

      inline double Lambda () const;

      inline void Lambda (double l);

      inline int Type () const;

      inline bool Active () const;

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

      inline double Pleak() const;
      
      inline double L1Eacc() const;

      inline double L1Pleak() const;

      inline double L2Eacc() const;

      inline double L2Pleak() const;

      // Debug methods
      //void Print () const;

    protected:

      inline void SetL1Size ( double s );

      inline void SetL1Lat ( int l );

      inline void SetL2Size ( double s );

      inline void SetL2Lat ( int l );

      inline void SetL3SizeEff ( double s );

      inline void SetOoO ( int o );

      inline void SetArea ( double a );

      inline void SetActive ( bool a );

      //inline void SetIpc ( double i );

      //inline void SetMpi ( double m );

      //inline void SetL1AccProb(double p);
      
      //inline void SetL2AccProb(double p);

      //inline void SetL3AccProb(double p);

      //inline void SetMMAccProb(double p);

      inline void SetFreq(double f);

      inline void SetEpi(double e);

      inline void SetPleak(double p);

      inline void SetL1Eacc(double e);

      inline void SetL1Pleak(double p);

      inline void SetL2Eacc(double e);

      inline void SetL2Pleak(double p);

      inline void SetType(int t);

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

      // The ipc, mpi and cache probabilities are now
      // functions of the workload;
      // still, processor API is maintained

      //double ipc_; // # of instructions per second

      //double mpi_; // # of memory references per instruction

      //double l1AccProb_;

      //double l2AccProb_;

      //double l3AccProb_;

      //double mmAccProb_;

      vector<double> l3ProbDistr_; // prob. to access distributed L3 p/cluster

      double freq_;

      int type_; // type index

      bool active_; // whether the core is on or off

      // Power parameters

      double epi_; // energy per instruction

      double pleak_; // leakage power

      double l1Eacc_; // l1 access energy

      double l1Pleak_; // l1 leakage power

      double l2Eacc_; // l2 access energy

      double l2Pleak_; // l2 leakage power

      // Runtime parameters

      double thr_; // real throughput

      double lambda_; // probability of memory request per cycle
      
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
      //return ipc_;
      return cmpConfig.GetCurWlIpc(Type());
    }

    /*void Processor::SetIpc ( double i ) {
      ipc_ = i;
    }*/

    double Processor::Mpi () const {
      //return mpi_;
      return cmpConfig.GetCurWlMpi(Type());
    }
    
    /*void Processor::SetMpi ( double m ) {
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
    }*/

    double Processor::L1AccessProbability() const {
      //return l1AccProb_;
      return 1.0 - cmpConfig.EvalCurWlMissRate(L1Size());
    }

    double Processor::L2AccessProbability() const {
      //return l2AccProb_;
      return cmpConfig.EvalCurWlMissRate(L1Size()) -
               cmpConfig.EvalCurWlMissRate(L2Size()+L1Size());
    }

    double Processor::L3AccessProbability() const {
      //return l3AccProb_;
      return (L3SizeEff() < E_DOUBLE) ? 0.0 :
               (cmpConfig.EvalCurWlMissRate(L2Size()+L1Size()) -
                cmpConfig.EvalCurWlMissRate(L3SizeEff()+L2Size()+L1Size()));
    }

    double Processor::MainMemAccessProbability() const {
      //return mmAccProb_;
      return (L3SizeEff() < E_DOUBLE) ?
               cmpConfig.EvalCurWlMissRate(L2Size()+L1Size()) :
               cmpConfig.EvalCurWlMissRate(L3SizeEff()+L2Size()+L1Size());
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

    double Processor::Epi () const {
      return epi_;
    }

    void Processor::SetEpi ( double e ) {
      epi_ = e;
    }

    double Processor::Pleak () const {
      return pleak_;
    }

    void Processor::SetPleak ( double p ) {
      pleak_ = p;
    }

    double Processor::L1Eacc () const {
      return l1Eacc_;
    }

    void Processor::SetL1Eacc ( double e ) {
      l1Eacc_ = e;
    }

    double Processor::L1Pleak () const {
      return l1Pleak_;
    }

    void Processor::SetL1Pleak ( double p ) {
      l1Pleak_ = p;
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

  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_PROCESSOR_H_
