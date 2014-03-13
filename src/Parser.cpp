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
#include <limits>

#include "Parser.hpp"

using namespace std;

namespace cmpex {

//=======================================================================
/*
 * Constructors and destructor
 */

ValueParser::ValueParser ( void ) {}

ValueParser::~ValueParser () {}

//=======================================================================
/*
 * Split and return the first token of str, separated by comma (or eol).
 * The token and comma are erased from the string.
 */

string ValueParser::SplitToken (string& str)
{
  int cIdx = str.find(',');
  string token = str.substr(0, cIdx);
  str.erase(0, cIdx);
  if (cIdx != string::npos) {
    str.erase(0,1); // erase comma
  }
  return token;
}

//=======================================================================

} //namespace cmpex
