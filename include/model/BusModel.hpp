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

#ifndef _MODEL_BUSMODEL_H_
#define _MODEL_BUSMODEL_H_

#include <vector>
#include <list>

#include "../Defs.hpp"

#include <newmat.h>

namespace cmpex {

  namespace model {

    //======================================================================
    // BusModel class implements a performance analytical model for bus,
    // as described in the paper by U. Ogras et al.
    // "Analytical router modeling for networks-on-chip performance analysis."
    //======================================================================

    class BusModel {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors and destructor
      BusModel ( UShort nPorts, double sTime );

      virtual ~BusModel ();

      // Accessors

      inline UShort NumPorts() const;

      inline double ServiceTime () const;

      inline void ServiceTime (double t);

      // Interface

      // This function calculates the delays in input buffers of the bus,
      // given the traffic matrix in 'traffic' parameter, and saves results
      // in the 'delays' array.
      // 'traffic' is assumed to represent a matrix of rates from each input
      // to each output and have the size of NumPorts()*NumPorts().
      // 'delays' is assumed to have the size of NumPorts() elements.
      // bool fixNegDelays defines whether negative delays in case violation of
      // the steady-state assumption have to be fixed to an arbitraty large value
      // (this is useful for bisection method to keep the sign of the difference function).
      int CalcBufferDelays (double * traffic, double * delays, bool fixNegDelays = false);

      // Service functions

      // Calculates traffic matrix, that is a diagonal matrix with traffics
      // for every input port. Calculation is performed using data from
      // extended input-to-output traffic matrix.
      void CalcTrafficMatrix (double * traffic);

      // Calculates the vector of residual times.
      void CalcResTimes ();

      // Debug methods
      //void Print () const;

    protected:

    private:

      // Deprecated methods: prevent usage
      BusModel ( const BusModel& );

      BusModel& operator = ( const BusModel& );

      // -------------------------- Attributes -----------------------------

    private:

      UShort numPorts_; // number of input/output ports of the router

      double serviceTime_; // packet service time

      // ----------- Model matrices -----------
      DiagonalMatrix mTraffic; // matrix of input traffic rates

      ColumnVector resTime; // vector of residual times

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    UShort BusModel::NumPorts() const {
      return numPorts_;
    }

    double BusModel::ServiceTime() const {
      return serviceTime_;
    }

    void BusModel::ServiceTime(double t) {
      serviceTime_ = t;
    }

  } // namespace model

} // namespace cmpex

#endif // _MODEL_BUSMODEL_H_
