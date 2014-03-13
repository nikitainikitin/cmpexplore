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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <iostream>
#include <sstream>
#include <string>

#include "Defs.hpp"

using std::string;

namespace cmpex {

  //======================================================================
  // The Debug class provides debug utilities.
  // Create an object and set minimal debug level. All data sent to the
  // object should be tagged by the debug level and will be printed only
  // if the data level is less or equal than the debug level.
  // Example:
  //   Debug debug;
  //   debug.Level(5);                     // print only messages of level >= 5
  //   debug(1) << "Hi" << endl;           // this will be printed
  //   debug(5) << string("123") << endl;  // this will be printed
  //   debug(6) << 456 << 'a' << endl;     // this will NOT be printed
  //======================================================================

  class Debug
  {
    // ---------------------------- Methods ------------------------------

  public:

    Debug ( void );

    ~Debug ();
    
    inline std::ostream& operator()( int l );
    
    // Accessors
    
    inline int Level () const;
    
    inline void Level (int l);
    
  private:
    
    inline std::ostringstream& Buf ();
    
  private:
    
    int level_; // debug level
    
    std::ostringstream buf_; // buffer for unused symbols (emulates /dev/null)

  };
  
  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int Debug::Level () const {
    return level_;
  }
  
  void Debug::Level (int l) {
    level_ = l;
  }
  
  std::ostringstream& Debug::Buf () {
    return buf_;
  }
  
  std::ostream& Debug::operator() ( int l )
  {
    if (l <= Level()) {
      return std::cout;
    }
    else {
      Buf().str("");
      return Buf();
    }
  }

}; // namespace cmpex

#endif // _DEBUG_H_
