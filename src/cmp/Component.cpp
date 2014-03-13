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

#include "Component.hpp"

#include <iostream>

using std::cout;
using std::endl;

namespace cmpex {

  namespace cmp {
  
//=======================================================================
/*
 * Constructors and destructor
 */

Component::Component ( UShort idx, UShort clIdx, CompType type, Component * parent ) : 
  idx_ (idx), clIdx_ (clIdx), type_ (type), parent_ (parent) {}

Component::~Component () {}

//=======================================================================
/*
 * Print: this is stub function for component.
 */

void Component::Print () const
{
  
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
