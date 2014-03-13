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

#include "RouterModel.hpp"

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

RouterModel::RouterModel (UShort nIPorts, UShort nOPorts, double serviceTime) :
  numIPorts_ (nIPorts), numOPorts_ (nOPorts), serviceTime_ (serviceTime),
  mForwProb (new double[numIPorts_*numOPorts_]),
  mCont(numIPorts_, numIPorts_), mTraffic(numIPorts_), resTime(numIPorts_)
{
  for (int i = 0; i < nIPorts*nOPorts; ++i)
    mForwProb[i] = 0.0;

  mCont = 0.0;
  mTraffic = 0.0;
  resTime = 0.0;
}

RouterModel::~RouterModel ()
{
  delete[] mForwProb;
}

//=======================================================================
/*
 * This function calculates the delays in input buffers of the router,
 * given the traffic matrix in 'traffic' parameter, and saves results
 * in the 'delays' array.
 * 'traffic' is assumed to represent a matrix of rates from each input
 * to each output and have the size of NumPorts()*NumPorts().
 * 'delays' is assumed to have the size of NumPorts() elements.
 * Output parameter adj will be used to save the required adjustment
 * guaranteeing the equillibrium condition (\lambda*T < 1) in case of an error.
 * If there is no error, adj is set to 1.
 * bool fixNegDelays defines whether negative delays in case violation of
 * the steady-state assumption have to be fixed to an arbitraty large value
 * (this is useful for bisection method to keep the sign of the difference function).
 */

int RouterModel::CalcBufferDelays (double * traffic, double * delays,
                                   double * adj, bool fixNegDelays)
{
  CalcTrafficMatrix(traffic);
  CalcResTimes();
  CalcForwProbMatrix(traffic);
  CalcContMatrix();

  //cout << "mTraffic = " << mTraffic << endl;
  //cout << "mCont = " << mCont << endl;

  Matrix Inv = (IdentityMatrix(NumIPorts()) - ServiceTime()*mTraffic*mCont).i();
  ColumnVector N = Inv*mTraffic*resTime;

  // divide by the traffic rates to get times, and fill the output vector
  for (int i = 1; i <= NumIPorts(); ++i) {
    delays[i-1] = (mTraffic(i) < EMIN_DOUBLE) ? 0.0 : N(i)/mTraffic(i);
    //cout << delays[i-1] << endl;
  }

  for (int i = 1; i <= NumIPorts(); ++i) {
    if (delays[i-1] < 0) {
      if (fixNegDelays) {
        delays[i-1]=1.0e6; // arbitrary large value
        continue;
      }
      // calculate adjustment
      if (adj) {
        double totalRate = 0.0;
        for (int p = 1; p <= NumIPorts(); ++p) {
          totalRate += mTraffic(p);
        }
        DASSERT(totalRate*ServiceTime() >= 1.0);
        *adj = totalRate*ServiceTime();
      }
      return 1; // error
    }
  }

  if (adj) *adj = 1.0; // no error
  return 0;
}

//=======================================================================
/*
 * Calculates matrix of forward probabilities, given the input-to-output
 * traffic rates matrix of size NumPorts()*NumPorts().
 * Uses mTraffic matrix (should be precalculated!).
 */

void RouterModel::CalcForwProbMatrix (double * traffic)
{
  for (int i = 0; i < NumIPorts(); ++i) {
    double totalRate = mTraffic(i+1);
    int startIdx = i*NumOPorts();
    for (int o = 0; o < NumOPorts(); ++o) {
      if (i == o) {
        // check that there's no traffic between input and output of the same direction
        /*cout << "input " << i << " output " << o
             << " totalRate " << totalRate << " prob i->o = " << traffic[startIdx+o]/totalRate << endl;
        DASSERT(totalRate < EMIN_DOUBLE | traffic[startIdx+o]/totalRate < EMIN_DOUBLE);*/
        mForwProb[startIdx+o] = 0.0;
      }
      else {
        mForwProb[startIdx+o] =
          (totalRate < EMIN_DOUBLE) ? 0.0 : traffic[startIdx+o]/totalRate;
      }
    }
  }
}

//=======================================================================
/*
 * Calculates traffic matrix, that is a diagonal matrix with traffics
 * for every input port. Calculation is performed using data from
 * extended input-to-output traffic matrix.
 */

void RouterModel::CalcTrafficMatrix (double * traffic)
{
  for (int i = 0; i < NumIPorts(); ++i) {
    double inTraffic = 0.0;
    double * p = traffic + i*NumOPorts();
    for (int o = 0; o < NumOPorts(); ++o, ++p) {
      inTraffic += *p;
    }
    mTraffic(i+1) = inTraffic;
    //cout << "Traffic inport " << i << " = " << inTraffic << "; ";
  }
  //cout << endl;
}

//=======================================================================
/*
 * Calculates traffic matrix, that is a diagonal matrix with traffics
 * for every input port. Calculation is performed using data from
 * extended input-to-output traffic matrix.
 * Fills in the output traffic matrix.
 */

void RouterModel::CalcTrafficMatrix (double * traffic, double * trOut)
{
  for (int i = 0; i < NumIPorts(); ++i) {
    double inTraffic = 0.0;
    double * p = traffic + i*NumOPorts();
    for (int o = 0; o < NumOPorts(); ++o, ++p) {
      inTraffic += *p;
    }
    trOut[i] = inTraffic;
  }
}

//=======================================================================
/*
 * Calculates vector of residual times.
 * Uses mTraffic matrix (should be precalculated!).
 */

void RouterModel::CalcResTimes()
{
  for (int i = 1; i <= NumIPorts(); ++i) {
    resTime(i) = 0.5*mTraffic(i)*ServiceTime()*ServiceTime();
  }
}

//=======================================================================
/*
 * Calculates the contention matrix.
 * The matrix of forward probabilities should be precalculated.
 */

void RouterModel::CalcContMatrix()
{
  for (int i = 0; i < NumIPorts(); ++i) {
    for (int j = 0; j < NumIPorts(); ++j) {
      double prob = 0.0;
      if (i == j) {
        prob = 1.0;
      }
      else {
        for (int o = 0; o < NumOPorts(); ++o) {
          prob += mForwProb[i*NumOPorts()+o]*mForwProb[j*NumOPorts()+o];
        }
      }
      mCont(i+1,j+1) = prob;
    }
  }
}

//=======================================================================
/*
 * Calculates the contention matrix.
 * Fills in the output vector.
 */

void RouterModel::CalcContMatrix(double * traffic, double * conts)
{
  CalcTrafficMatrix(traffic);
  CalcForwProbMatrix(traffic);

  for (int i = 0; i < NumIPorts(); ++i) {
    for (int j = 0; j < NumIPorts(); ++j) {
      double prob = 0.0;
      if (i == j) {
        prob = 1.0;
      }
      else {
        for (int o = 0; o < NumOPorts(); ++o) {
          prob += mForwProb[i*NumOPorts()+o]*mForwProb[j*NumOPorts()+o];
        }
      }
      conts[i*NumOPorts()+j] = prob;
    }
  }
}

//=======================================================================
/*
 * This function calculates utilizations for the input
 * ports of the router, assuming infinite queues.
 */

void RouterModel::CalcUtilizations(double * traffic, double * util)
{
  CalcTrafficMatrix(traffic);
  CalcForwProbMatrix(traffic);
  CalcContMatrix();

  for (int i = 0; i < NumIPorts(); ++i) {
    double stime = 0.0;
    for (int j = 0; j < NumIPorts(); ++j) {
      stime += mCont(i+1,j+1);
    }
    util[i] = stime*ServiceTime()*mTraffic(i+1);
  }
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
