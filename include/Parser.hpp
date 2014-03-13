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

#ifndef _PARSER_H_
#define _PARSER_H_

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "Util.hpp"

using std::string;
using std::vector;

namespace cmpex {

  //======================================================================
  // ValueParser class reads lines assigning an enumeration or a range
  // of values to variable with the following grammar:
  //   <VARIABLE> = <ListOfValues>
  //   <ListOfValues> = <Value> {, <ListOfValues>}
  //   <Value> = Number | [Num-Num]
  //
  //  The following 3 assignments are identical:
  //    Param = 1, 2, 3, 4
  //    Param = [1-4]
  //    Param = 1, [2-3], 4
  //======================================================================

  class ValueParser
  {
    
    // ---------------------------- Methods ------------------------------

  public:

    ValueParser ( void );

    ~ValueParser ();
    
    // Parse the string into vector of point values.
    // String may only contain point values, not intervals.
    template<typename T> static
      void GetVectorOfValues (string& str, vector<T>& vec);
    
    // Parse the string into vector of point values.
    // String may contain point values and intervals.
    template<typename T> static
      void GetVectorOfIValues (string& str, vector<T>& vec);

    // Extract value of type T from string and save it in val.
    // The part of the string until next value is erased.
    template<typename T> static
      void ExtractValue (string& str, T& val);

  protected:

    // Service functions
    
    // Split and return the first token of str,
    // separated by comma (or eol).
    // The token and comma are erased from the string.
    static string SplitToken (string& str);
    
  };

  //----------------------------------------------------------------------
  // Template functions
  //----------------------------------------------------------------------

  //=======================================================================
  /*
   * Parse the string and save values into vector.
   * String may only contain point values, not intervals.
   */

  template<typename T>
    void ValueParser::GetVectorOfValues (string& str, vector<T>& vec)
  {
    string token;
    while (!(token = SplitToken(str)).empty()) {
      std::istringstream is(token);
      T val;
      is >> val;
      vec.push_back(val);
    }
  }

  //=======================================================================
  /*
   * Parse the string and save values into vector.
   * String may contain point values and intervals.
   */

  template<typename T>
    void ValueParser::GetVectorOfIValues (string& str, vector<T>& vec)
  {
    string token;
    bool errorFlag = false;
    
    while (!(token = SplitToken(str)).empty()) {
      int lIdx = token.find('[');
      int rIdx = token.find(']');
      if ( lIdx == string::npos ) {
        if ( rIdx == string::npos ) { // token is a point value
          std::istringstream is(token);
          T val;
          is >> val;
          vec.push_back(val);
        }
        else errorFlag = true;
      }
      else {
        if ( rIdx == string::npos ) errorFlag = true;
        else {
          int dIdx = token.find('-');
          if ( dIdx == string::npos ) errorFlag = true;
          else { // token is an interval
            std::istringstream isMin(token.substr(lIdx+1, dIdx-lIdx-1));
            T valMin;
            isMin >> valMin;
            std::istringstream isMax(token.substr(dIdx+1, rIdx-dIdx-1));
            T valMax;
            isMax >> valMax;
            for (T it = valMin; it <= valMax; ++it) {
              vec.push_back(it);
            }
          }
        }
      }

      if (errorFlag) {
        std::cerr << "-E- Wrong format of input line" << std::endl;
        return;
      }
    }
  }

  //=======================================================================
  /*
   * Extract value of type T from string and save it in val.
   * The part of the string until next value is erased.
   */

  template<typename T>
    void ValueParser::ExtractValue (string& str, T& val)
  {
    LStrip(str);
    int idx = str.find_first_of(" \t");
    std::istringstream is(str);
    is >> val;
    str.erase(0, idx);
  }

  //=======================================================================

} // namespace cmpex

#endif // _PARSER_H_
