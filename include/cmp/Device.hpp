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

#ifndef _CMP_DEVICE_H_
#define _CMP_DEVICE_H_

#include <vector>
#include <list>

#include "Component.hpp"

namespace cmpex {
  
  namespace cmp {

    //======================================================================
    // Device is a common abstraction for all flat components.
    // It guarantees common interface for extended device types.
    // The constructor is protected, so device itself can't be instantiated.
    //======================================================================

    class Device : public Component {
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Destructor
      virtual ~Device ();
      
      // Accessors
      
      // Implementations of the component interface.

      // Debug methods
      //void Print () const;

    protected:

      Device ( UShort idx, UShort clIdx, CompType type, Component * parent = 0 );

    private:

      // Deprecated methods: prevent usage
      Device ( const Device& );

      Device& operator = ( const Device& );

      // -------------------------- Attributes -----------------------------

    private:
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_DEVICE_H_
