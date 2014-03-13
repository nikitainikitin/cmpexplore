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

#ifndef _CMP_MESHICTILE_H_
#define _CMP_MESHICTILE_H_

#include <GVertex.hpp>
#include "MeshIcLink.hpp"

namespace cmpex {
  
  namespace cmp {
  
    class Component;
    
    //======================================================================
    // MeshIcTile class represents one cell for MeshIc.
    //======================================================================

    class MeshIcTile : public GVertex {
      
      friend class MeshIc;

      // ---------------------------- Methods ------------------------------

    public:

      // Accessors
      inline cmp::Component * Component () const;

      inline void SetComponent ( cmp::Component * c );
      
      // Link accessors
      inline MeshIcLink * NorthIn () const;
      
      inline MeshIcLink * NorthOut () const;
      
      inline MeshIcLink * WestIn () const;
      
      inline MeshIcLink * WestOut () const;
      
      inline MeshIcLink * SouthIn () const;
      
      inline MeshIcLink * SouthOut () const;
      
      inline MeshIcLink * EastIn () const;
      
      inline MeshIcLink * EastOut () const;
      
      // Debug methods
      void Print ();
      
    protected:
      
      // Constructors & destructor
      MeshIcTile ( int idx, int cidx, int ridx );

      virtual ~MeshIcTile ();
      
    private:

      // Deprecated methods: prevent usage
      MeshIcTile ( const MeshIcTile& );

      MeshIcTile& operator = ( const MeshIcTile& );

      // -------------------------- Attributes -----------------------------

    private:
      
      cmp::Component * comp_; // component associated with tile

    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    cmp::Component * MeshIcTile::Component () const {
      return comp_;
    }
    
    void MeshIcTile::SetComponent (cmp::Component * c) {
      comp_ = c;
    }
    
    MeshIcLink * MeshIcTile::NorthIn () const {
      return static_cast<MeshIcLink*>(GVertex::NorthIn());
    }
    
    MeshIcLink * MeshIcTile::NorthOut () const {
      return static_cast<MeshIcLink*>(GVertex::NorthOut());
    }
    
    MeshIcLink * MeshIcTile::WestIn () const {
      return static_cast<MeshIcLink*>(GVertex::WestIn());
    }
    
    MeshIcLink * MeshIcTile::WestOut () const {
      return static_cast<MeshIcLink*>(GVertex::WestOut());
    }
    
    MeshIcLink * MeshIcTile::SouthIn () const {
      return static_cast<MeshIcLink*>(GVertex::SouthIn());
    }
    
    MeshIcLink * MeshIcTile::SouthOut () const {
      return static_cast<MeshIcLink*>(GVertex::SouthOut());
    }
    
    MeshIcLink * MeshIcTile::EastIn () const {
      return static_cast<MeshIcLink*>(GVertex::EastIn());
    }
    
    MeshIcLink * MeshIcTile::EastOut () const {
      return static_cast<MeshIcLink*>(GVertex::EastOut());
    }
    
    /*void Tile::SetClass ( int c ) {
      class_ = c;
    }
    
    void Tile::SetVi ( int vi ) {
      vi_ = vi;
    }
    
    void Tile::SetTci ( TileClassInfo * tci ) {
      ptci_ = tci;
    }*/
  
  } // namespace cmp;

} // namespace cmpex

#endif // _CMP_MESHICTILE_H_
