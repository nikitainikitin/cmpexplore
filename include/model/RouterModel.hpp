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

#ifndef _MODEL_ROUTERMODEL_H_
#define _MODEL_ROUTERMODEL_H_

#include <vector>
#include <list>

#include "../Defs.hpp"

#include <newmat.h>

namespace cmpex {

  namespace model {

    //======================================================================
    // RouterModel class implements a performance analytical model for
    // NoC router, as described in the paper by U. Ogras et al.
    // "Analytical router modeling for networks-on-chip performance analysis."
    //======================================================================

    class RouterModel {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors and destructor
      RouterModel ( UShort nIPorts, UShort nOPorts, double serviceTime );

      virtual ~RouterModel ();

      // Accessors

      inline UShort NumIPorts() const;

      inline UShort NumOPorts() const;

      inline double ServiceTime () const;

      inline void ServiceTime (double t);

      // Interface

      // This function calculates the delays in input buffers of the router,
      // given the traffic matrix in 'traffic' parameter, and saves results
      // in the 'delays' array.
      // 'traffic' is assumed to represent a matrix of rates from each input
      // to each output and have the size of NumPorts()*NumPorts().
      // 'delays' is assumed to have the size of NumPorts() elements.
      // Output parameter adj will be used to save the required adjustment
      // guaranteeing the equillibrium condition (\lambda*T < 1) in case of an error.
      // If there is no error, adj is set to 1.
      // bool fixNegDelays defines whether negative delays in case violation of
      // the steady-state assumption have to be fixed to an arbitraty large value
      // (this is useful for bisection method to keep the sign of the difference function).
      int CalcBufferDelays (double * traffic, double * delays,
                            double * adj = 0, bool fixNegDelays = false);

      // Service functions

      // Calculates matrix of forward probabilities, given the input-to-output
      // traffic rates matrix of size NumPorts()*NumPorts().
      void CalcForwProbMatrix (double * traffic);

      // Calculates traffic matrix, that is a diagonal matrix with traffics
      // for every input port. Calculation is performed using data from
      // extended input-to-output traffic matrix.
      void CalcTrafficMatrix (double * traffic);

      // Calculates traffic matrix, that is a diagonal matrix with traffics
      // for every input port. Calculation is performed using data from
      // extended input-to-output traffic matrix.
      // Fills in the output traffic vector.
      void CalcTrafficMatrix (double * traffic, double * trOut);

      // Calculates the vector of residual times.
      void CalcResTimes ();

      // Calculates the contention matrix.
      void CalcContMatrix ();

      // Calculates the contention matrix from given traffic matrix
      // and fills in the output vector.
      void CalcContMatrix (double * traffic, double * conts);

      // Calculates utilizations for input ports of the router,
      // assuming infinite queues
      void CalcUtilizations (double * traffic, double * util);

      // Debug methods
      //void Print () const;

    protected:

    private:

      // Deprecated methods: prevent usage
      RouterModel ( const RouterModel& );

      RouterModel& operator = ( const RouterModel& );

      // -------------------------- Attributes -----------------------------

    private:

      UShort numIPorts_; // number of input ports of the router

      UShort numOPorts_; // number of output ports of the router

      double serviceTime_; // packet service time

      // ----------- Model matrices -----------
      double * mForwProb; // matrix of forwarding probabilities

      Matrix mCont; // contention matrix

      DiagonalMatrix mTraffic; // matrix of input traffic rates

      ColumnVector resTime; // vector of residual times

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    UShort RouterModel::NumIPorts() const {
      return numIPorts_;
    }

    UShort RouterModel::NumOPorts() const {
      return numOPorts_;
    }

    double RouterModel::ServiceTime() const {
      return serviceTime_;
    }

    void RouterModel::ServiceTime(double t) {
      serviceTime_ = t;
    }

  } // namespace model

} // namespace cmpex

#endif // _MODEL_ROUTERMODEL_H_
