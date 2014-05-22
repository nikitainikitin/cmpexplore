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

#ifndef _CMP_MEMORY_H_
#define _CMP_MEMORY_H_

#include <vector>
#include <list>

#include "Device.hpp"
#include "CmpBuilder.hpp"

namespace cmpex {

  extern cmp::CmpBuilder cmpBuilder;

  namespace cmp {

    //======================================================================
    // This class represents a memory component of the Cmp.
    //======================================================================

    class Memory : public Device {

      friend class CmpBuilder;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors and destructor
      Memory ( UShort idx, UShort clIdx, cmp::MemType mt, Component * parent = 0 );

      virtual ~Memory ();

      // Accessors
      inline double Size() const;

      inline double Latency() const;

      inline cmp::MemType MemType() const;

      inline UShort McIdx() const;

      inline void McIdx(UShort mc);

      inline double Lambda() const;

      inline void Lambda(double l);

      inline double BufDelay() const;

      inline void BufDelay(double d);

      inline double Eacc() const;

      inline double Pleak() const;

      inline bool Active () const;

      inline void SetActive ( bool a );


      // Implementations of the Component interface.

      // Returns true if component contains the processor 'idx'.
      virtual bool HasProcessor( UShort idx );

      // Returns true if component contains the memory 'idx'.
      virtual bool HasMemory( UShort idx );

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

      // Debug methods
      //void Print () const;

    protected:

      inline void SetSize ( double s );

      inline void SetLatency ( double l );

      inline void SetEacc ( double e );

      inline void SetPleak ( double p );

    private:

      // Deprecated methods: prevent usage
      Memory ( const Memory& );

      Memory& operator = ( const Memory& );

      // -------------------------- Attributes -----------------------------

    private:

      cmp::MemType mtype_;

      double size_; // memory size in Mb

      double lat_; // memory latency in cycles

      UShort mcIdx_; // index of the associated MC

      // Runtime parameters

      double lambda_; // number of requests per ns

      double bufDelay_; // contention delays in the input buffer

      // Power parameters

      double eacc_; // access energy

      double pleak_; // leakage power

      bool active_; // whether the memory is active ot not

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    double Memory::Size () const {
      return size_;
    }

    void Memory::SetSize ( double s ) {
      size_ = s;
    }

    double Memory::Latency () const {
      return lat_;
    }

    void Memory::SetLatency ( double l ) {
      lat_ = l;
    }

    cmp::MemType Memory::MemType () const {
      return mtype_;
    }

    UShort Memory::McIdx() const {
      return mcIdx_;
    }

    void Memory::McIdx(UShort mc) {
      mcIdx_ = mc;
    }

    double Memory::Lambda() const {
      return lambda_;
    }

    void Memory::Lambda(double l) {
      lambda_ = l;
    }

    double Memory::BufDelay() const {
      return bufDelay_;
    }

    void Memory::BufDelay(double d) {
      bufDelay_ = d;
    }

    double Memory::Eacc () const {
      return eacc_;
    }

    void Memory::SetEacc ( double e ) {
      eacc_ = e;
    }

    double Memory::Pleak () const {
      return pleak_;
    }

    void Memory::SetPleak ( double p ) {
      pleak_ = p;
    }

    bool Memory::Active () const {
      return active_;
    }

    void Memory::SetActive ( bool a ) {
      active_ = a;
    }

  } // namespace cmp

} // namespace cmpex

#endif // _CMP_MEMORY_H_
