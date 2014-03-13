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

#ifndef _CMP_MESHICLINK_H_
#define _CMP_MESHICLINK_H_

#include <GDEdge.hpp>

namespace cmpex {
  
  namespace cmp {

    class MeshIcTile;

    //======================================================================
    // MeshIcLink represents a connection between neighboring tiles in MeshIc.
    //======================================================================

    class MeshIcLink : public GDEdge {

      friend class MeshIc;

      // ---------------------------- Methods ------------------------------

    public:

      // Accessors
      inline double Cap () const;
      
      // Tile accessors
      const MeshIcTile * Src () const;

      MeshIcTile * Src ();

      const MeshIcTile * Dst () const;

      MeshIcTile * Dst ();

      // Debug methods
      void Print ();
      
    protected:
      
      // Constructors & destructor
      MeshIcLink ( int idx, MeshIcTile * src, MeshIcTile * dst,
                     bool horizontal, double cap );

      virtual ~MeshIcLink ();
      
    private:

      // Deprecated methods: prevent usage
      MeshIcLink ( const MeshIcLink& );

      MeshIcLink& operator = ( const MeshIcLink& );

      // -------------------------- Attributes -----------------------------

    private:

      double cap_; // link capacity
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    double MeshIcLink::Cap () const {
      return cap_;
    }
    
  } // namespace cmp

} // namespace cmpex

#endif // _CMP_MESHICLINK_H_
