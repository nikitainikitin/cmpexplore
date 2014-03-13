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

#ifndef _ARCH_FUNCTION_H_
#define _ARCH_FUNCTION_H_

#include <string>
#include <vector>
#include <cmath>

#include "Defs.hpp"

using std::string;
using std::vector;

namespace cmpex {

  namespace model {
  
    //======================================================================
    // Function class is the interface for 2D function representation
    // within the program.
    // Functions have the eval() method, returning the value (double)
    // of ordinate, given the absciss value (double).
    //======================================================================

    class Function {

    public:

      enum FunctionType { FTSQRT = 0, FTPOWERLAW, FTPIECEWISE, FTLINEAR, FTCONST, NUMFT };

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Function (FunctionType t);

      virtual ~Function ();
      
      inline FunctionType Type () const;

      virtual double eval (double absciss) const = 0;

      // Creates function from a descriptive string
      static Function * ParseFunction(string& fn);

    private:

      // Deprecated methods: prevent usage
      Function ( const Function& );

      Function& operator = ( const Function& );

      // -------------------------- Attributes -----------------------------

    private:
    
      FunctionType type_;

    };

    //======================================================================
    // Sqrt represents functions in the form k*sqrt(absciss).
    //======================================================================

    class Sqrt : public Function {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Sqrt (double coefficient);

      virtual ~Sqrt ();
      
      inline virtual double eval (double absciss) const;

    private:

      // Deprecated methods: prevent usage
      Sqrt ( const Sqrt& );

      Sqrt& operator = ( const Sqrt& );

      // -------------------------- Attributes -----------------------------

    private:

      double k_;

    };
    
    //======================================================================
    // Powerlaw represents functions in the form of k*absciss^(-e).
    //======================================================================

    class Powerlaw : public Function {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Powerlaw (double coefficient, double exponent);

      virtual ~Powerlaw ();

      inline virtual double eval (double absciss) const;

    private:

      // Deprecated methods: prevent usage
      Powerlaw ( const Powerlaw& );

      Powerlaw& operator = ( const Powerlaw& );

      // -------------------------- Attributes -----------------------------

    private:

      double k_;

      double e_;

    };

    //======================================================================
    // Piecewise represents functions defined as a set of linear intervals.
    // Abscissas to the left of the leftmost interval have the same ordinate,
    // defined by the left point of the interval.
    // Abscissas to the right of the rightmost interval have the same ordinate,
    // defined by the right point of the interval.
    //======================================================================

    class Piecewise : public Function {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Piecewise (const vector<double>& abscissas, const vector<double>& ordinates);

      virtual ~Piecewise ();

      virtual double eval (double absciss) const;

    private:

      // Deprecated methods: prevent usage
      Piecewise ( const Piecewise& );

      Piecewise& operator = ( const Piecewise& );

      // -------------------------- Attributes -----------------------------

    private:

      vector<double> abs_; // vector of abscissas

      vector<double> ord_; // vector of related ordinates

    };

    //======================================================================
    // Linear represents functions in the form k1*absciss+k0.
    //======================================================================

    class Linear : public Function {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Linear (double coef_1, double coef_0);

      virtual ~Linear ();

      inline virtual double eval (double absciss) const;

    private:

      // Deprecated methods: prevent usage
      Linear ( const Linear& );

      Linear& operator = ( const Linear& );

      // -------------------------- Attributes -----------------------------

    private:

      double k1_;

      double k0_;

    };

    //======================================================================
    // Const represents a constant function.
    //======================================================================

    class Const : public Function {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Const (double coefficient);

      virtual ~Const ();

      inline virtual double eval (double absciss) const;

    private:

      // Deprecated methods: prevent usage
      Const ( const Const& );

      Const& operator = ( const Const& );

      // -------------------------- Attributes -----------------------------

    private:

      double c_;

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    Function::FunctionType Function::Type() const {
      return type_;
    }

    double Sqrt::eval(double absciss) const {
      return k_ * sqrt(absciss);
    }

    double Powerlaw::eval(double absciss) const {
      return k_ * pow(absciss, e_);
    }

    double Linear::eval(double absciss) const {
      return k1_ * absciss + k0_;
    }

    double Const::eval(double absciss) const {
      return c_;
    }

  } // namespace model

} // namespace cmpex

#endif // _ARCH_FUNCTION_H_
