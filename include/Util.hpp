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

#ifndef _UTIL_H_
#define _UTIL_H_

#include <string>

#include "../Defs.hpp"

namespace cmpex {
  
  //======================================================================
  // This file contains a collection of utility functions.
  //======================================================================

  // Conversion of numbers to and from strings
  std::string IntToStr ( int i );
  
  std::string DoubleToStr ( double d );
  
  int StrToInt ( const std::string& s );
  
  double StrToDouble ( const std::string& s );
  
  // String utilities:
  // Removes leading spaces and tabs in 's'
  void LStrip ( std::string& s );
  
  // Removes following spaces and tabs in 's'
  void RStrip ( std::string& s );
  
  // Removes leading and following spaces and tabs in 's'
  void Strip ( std::string& s );
  
  // Returns true if 's' starts with 'start_s' string
  bool StartsWith ( const std::string& s, const std::string& start_s );

  // Check file for existence
  bool FileExists ( const std::string& fname );

  // Returns true of two doubles are equal
  bool DoublesAreEqual ( const double& d1, const double& d2 );

} // namespace cmpex

#endif // _UTIL_H_
