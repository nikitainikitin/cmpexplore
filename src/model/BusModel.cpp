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

#include "BusModel.hpp"

#include <iostream>
#include <iomanip>

#include <newmatio.h>

using std::cout;
using std::endl;

namespace cmpex {

  namespace model {

//=======================================================================
/*
 * Constructors and destructor
 */

BusModel::BusModel (UShort nPorts, double sTime) : numPorts_ (nPorts),
  serviceTime_ (sTime),
  mTraffic(numPorts_), resTime(numPorts_)
{
  mTraffic = 0.0;
  resTime = 0.0;
}

BusModel::~BusModel () {}

//=======================================================================
/*
 * This function calculates the delays in input buffers of the bus,
 * given the traffic matrix in 'traffic' parameter, and saves results
 * in the 'delays' array.
 * 'traffic' is assumed to represent a matrix of rates from each input
 * to each output and have the size of NumPorts()*NumPorts().
 * 'delays' is assumed to have the size of NumPorts() elements.
 * bool fixNegDelays defines whether negative delays in case violation of
 * the steady-state assumption have to be fixed to an arbitraty large value
 * (this is useful for bisection method to keep the sign of the difference function).
 */

int BusModel::CalcBufferDelays (double * traffic, double * delays, bool fixNegDelays)
{
  CalcTrafficMatrix(traffic);
  CalcResTimes();

  // contention matrix is the matrix of all ones
  Matrix mCont(NumPorts(), NumPorts());
  mCont = 1.0;

  Matrix Inv = (IdentityMatrix(NumPorts()) - ServiceTime()*mTraffic*mCont).i();
  ColumnVector N = Inv*mTraffic*resTime;

  // divide by the traffic rates to get times, and fill the output vector
  for (int i = 1; i <= NumPorts(); ++i) {
    delays[i-1] = (mTraffic(i) < EMIN_DOUBLE) ? 0.0 : N(i)/mTraffic(i);
    //cout << delays[i-1] << endl;
  }

  for (int i = 1; i <= NumPorts(); ++i) {
    if (delays[i-1] < 0) {
      if (fixNegDelays) {
        delays[i-1]=1.0e6; // arbitrary large value
        continue;
      }
      return 1; // error
    }
  }

  return 0;
}

//=======================================================================
/*
 * Calculates traffic matrix, that is a diagonal matrix with traffics
 * for every input port. Calculation is performed using data from
 * extended input-to-output traffic matrix.
 */

void BusModel::CalcTrafficMatrix (double * traffic)
{
  for (int i = 0; i < NumPorts(); ++i) {
    double inTraffic = 0.0;
    double * p = traffic + i*NumPorts();
    for (int o = 0; o < NumPorts(); ++o, ++p) {
      inTraffic += *p;
    }
    mTraffic(i+1) = inTraffic;
  }
}

//=======================================================================
/*
 * Calculates vector of residual times.
 * Uses mTraffic matrix (should be precalculated!).
 */

void BusModel::CalcResTimes()
{
  for (int i = 1; i <= NumPorts(); ++i) {
    resTime(i) = 0.5*mTraffic(i)*ServiceTime()*ServiceTime();
  }
}

//=======================================================================
  } // namespace cmp

} // namespace cmpex
