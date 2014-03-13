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

#include <iostream>
#include <string>
#include <sstream>

#include "Function.hpp"
#include "Util.hpp"

using namespace std;

namespace cmpex {

  namespace model {

//=======================================================================
/*
 * Constructors and destructor
 */

Function::Function(FunctionType t) : type_ (t) {}

Function::~Function() {}

Sqrt::Sqrt(double coefficient) :
  Function(Function::FTSQRT), k_ (coefficient) {}

Sqrt::~Sqrt() {}

Powerlaw::Powerlaw(double coefficient, double exponent) :
  Function(Function::FTPOWERLAW), k_ (coefficient), e_ (exponent) {}

Powerlaw::~Powerlaw() {}

Piecewise::Piecewise (const vector<double>& abscissas,
                      const vector<double>& ordinates) :
  Function(Function::FTPIECEWISE)
{
  abs_.assign(abscissas.begin(), abscissas.end());
  ord_.assign(ordinates.begin(), ordinates.end());
}

Piecewise::~Piecewise() {}

Linear::Linear(double coef_1, double coef_0) :
  Function(Function::FTLINEAR), k1_ (coef_1), k0_ (coef_0) {}

Linear::~Linear() {}

Const::Const(double coefficient) :
  Function(Function::FTCONST), c_ (coefficient) {}

Const::~Const() {}

//=======================================================================
/*
 * Read and create a function from string description.
 * Fn string may be changed (for performance issues).
 * Function ownership is transferred to the calling method.
 */

Function * Function::ParseFunction(string &fn)
{
  Function * f = 0;

  Strip(fn);
  if (StartsWith(fn, "sqrt")) {
    fn.erase(0, string("sqrt").size());
    istringstream is(fn);
    double coefficient;
    is >> coefficient;
    f = new Sqrt(coefficient);
  }
  else if (StartsWith(fn, "powerlaw")) {
    fn.erase(0, string("powerlaw").size());
    istringstream is(fn);
    double coefficient, exponent;
    is >> coefficient >> exponent;
    f = new Powerlaw(coefficient, exponent);
  }
  else if (StartsWith(fn, "piecewise")) {
    fn.erase(0, string("piecewise").size());
    LStrip(fn);
    vector<double> abss, ords;
    while(!fn.empty()) { // read next (abs ord) pair
      fn.erase(0, 1); // erase '(' char
      istringstream is(fn);
      double abs, ord;
      is >> abs >> ord;
      abss.push_back(abs);
      ords.push_back(ord);
      fn.erase(0, fn.find(')')+1);
      LStrip(fn);
    }
    f = new Piecewise(abss, ords);
  }
  else if (StartsWith(fn, "linear")) {
    fn.erase(0, string("linear").size());
    istringstream is(fn);
    double k1, k0;
    is >> k1 >> k0;
    f = new Linear(k1, k0);
  }
  else if (StartsWith(fn, "const")) {
    fn.erase(0, string("const").size());
    istringstream is(fn);
    double c;
    is >> c;
    f = new Const(c);
  }
  else {
    cerr << "-W- Unknown function type: " << fn << endl;
  }

  return f;
}

//=======================================================================
/*
 * Evaluates ordinate of the piecewise function.
 * Abscissas to the left of the leftmost interval have the same ordinate,
 * defined by the left point of the interval.
 * Abscissas to the right of the rightmost interval have the same ordinate,
 * defined by the right point of the interval.
 */

double Piecewise::eval(double absciss) const
{
  for (int i = 0; i < abs_.size(); ++i) {
    if (abs_[i] > absciss) {
      if (i) {
        // return linear ordinate between abs_[i-1] and abs_[i]
        return ord_[i-1] +
            (ord_[i]-ord_[i-1])*(absciss-abs_[i-1])/(abs_[i]-abs_[i-1]);
      }
      else {
        // return leftmost point ordinate
        return ord_[i];
      }
    }
  }

  // we're here if absciss is greater than the rightmost point
  // -> return rightmost ordinate
  return ord_[ord_.size()-1];
}

//=======================================================================

  } // namespace model

} //namespace cmpex
