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
#include <fstream>
#include <sstream>

#include "Util.hpp"

using std::cout;
using std::endl;
using std::string;
using std::istringstream;
using std::ostringstream;

namespace cmpex {

//=======================================================================
/*
 * Convert int to string.
 */

string IntToStr ( int i )
{
  ostringstream oss;
  oss << i;
  return oss.str();
}

//=======================================================================
/*
 * Convert double to string.
 */

string DoubleToStr ( double d )
{
  ostringstream oss;
  oss << d;
  return oss.str();
}

//=======================================================================
/*
 * Convert string to int.
 */

int StrToInt ( const string& s )
{
  istringstream iss(s);
  int i;
  iss >> i;
  return i;
}

//=======================================================================
/*
 * Convert string to double.
 */

double StrToDouble ( const string& s )
{
  istringstream iss(s);
  double d;
  iss >> d;
  return d;
}

//=======================================================================
/*
 * Removes leading spaces and tabs in 's'.
 */

void LStrip ( string& s )
{
  s.erase(0, s.find_first_not_of(" \t\n\r"));
}

//=======================================================================
/*
 * Removes following spaces and tabs in 's'.
 */

void RStrip ( string& s )
{
  s.erase(s.find_last_not_of(" \t\n\r")+1);
}

//=======================================================================
/*
 * Removes leading and following spaces and tabs in 's'.
 */

void Strip ( string& s )
{
  LStrip(s);
  RStrip(s);
}

//=======================================================================
/*
 * Returns true if 's' starts with 'start_s' string.
 */

bool StartsWith ( const string& s, const string& start_s )
{
  return (s.substr(0, start_s.length()) == start_s) ? true : false;
}

//=======================================================================

} // namespace cmpex
