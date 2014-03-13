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

#include "MeshIcLink.hpp"
#include "MeshIcTile.hpp"

using std::cout;
using std::endl;

namespace cmpex {
  
  namespace cmp {

//=======================================================================
/*
 * Constructors and destructor
 */

MeshIcLink::MeshIcLink ( int idx, MeshIcTile * src, MeshIcTile * dst,
    bool horizontal, double cap ) :
  GDEdge (idx, src, dst, horizontal), cap_ (cap) {}

MeshIcLink::~MeshIcLink () {}

//=======================================================================
/*
 * Tile accessors
 */

const MeshIcTile * MeshIcLink::Src () const {
  return static_cast<const MeshIcTile*>(GDEdge::Src());
}

MeshIcTile * MeshIcLink::Src () {
  return static_cast<MeshIcTile*>(GDEdge::Src());
}

const MeshIcTile * MeshIcLink::Dst () const {
  return static_cast<const MeshIcTile*>(GDEdge::Dst());
}

MeshIcTile * MeshIcLink::Dst () {
  return static_cast<MeshIcTile*>(GDEdge::Dst());
}

//=======================================================================

  } // namespace cmp
  
} // namespace cmpex
