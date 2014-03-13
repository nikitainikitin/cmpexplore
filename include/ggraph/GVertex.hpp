#ifndef _GVERTEX_H_
#define _GVERTEX_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GVertex.hpp
//   Created May 03, 2011
// ----------------------------------------------------------------------

#include "Defs.hpp"

#include <vector>

namespace cmpex {

  class GGraph;
  class GDEdge;
  
  //======================================================================
  // GVertex represents a cell/vertex of a grid graph.
  //======================================================================

  class GVertex {
    
    friend class GGraph;

    // ---------------------------- Methods ------------------------------

  public:

    // Accessors
    inline int Idx () const;
    
    inline int ColIdx () const;
    
    inline int RowIdx () const;
    
    inline GDEdge * NorthIn () const;
    
    inline GDEdge * NorthOut () const;
    
    inline GDEdge * WestIn () const;
    
    inline GDEdge * WestOut () const;
    
    inline GDEdge * SouthIn () const;
    
    inline GDEdge * SouthOut () const;
    
    inline GDEdge * EastIn () const;
    
    inline GDEdge * EastOut () const;
    
    // Debug methods
    void Print ();
    
  protected:

    // Constructors & destructor
    GVertex ( int idx, int cidx, int ridx );

    virtual ~GVertex ();

    inline void NorthIn ( GDEdge * e );

    inline void NorthOut ( GDEdge * e );

    inline void WestIn ( GDEdge * e );

    inline void WestOut ( GDEdge * e );

    inline void SouthIn ( GDEdge * e );

    inline void SouthOut ( GDEdge * e );

    inline void EastIn ( GDEdge * e );
    
    inline void EastOut ( GDEdge * e );
    
  private:

    // Deprecated methods: prevent usage
    GVertex ( const GVertex& );

    GVertex& operator = ( const GVertex& );

    // -------------------------- Attributes -----------------------------

  private:

    int idx_;
    
    int colIdx_;
    
    int rowIdx_;
    
    GDEdge * northIn_;

    GDEdge * northOut_;

    GDEdge * westIn_;

    GDEdge * westOut_;

    GDEdge * southIn_;

    GDEdge * southOut_;

    GDEdge * eastIn_;
    
    GDEdge * eastOut_;
    
  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int GVertex::Idx () const {
    return idx_;
  }

  int GVertex::ColIdx () const {
    return colIdx_;
  }

  int GVertex::RowIdx () const {
    return rowIdx_;
  }

  GDEdge * GVertex::NorthIn () const {
    return northIn_;
  }

  void GVertex::NorthIn ( GDEdge * e ) {
    northIn_ = e;
  }

  GDEdge * GVertex::NorthOut () const {
    return northOut_;
  }

  void GVertex::NorthOut ( GDEdge * e ) {
    northOut_ = e;
  }

  GDEdge * GVertex::WestIn () const {
    return westIn_;
  }

  void GVertex::WestIn ( GDEdge * e ) {
    westIn_ = e;
  }

  GDEdge * GVertex::WestOut () const {
    return westOut_;
  }

  void GVertex::WestOut ( GDEdge * e ) {
    westOut_ = e;
  }

  GDEdge * GVertex::SouthIn () const {
    return southIn_;
  }

  void GVertex::SouthIn ( GDEdge * e ) {
    southIn_ = e;
  }

  GDEdge * GVertex::SouthOut () const {
    return southOut_;
  }

  void GVertex::SouthOut ( GDEdge * e ) {
    southOut_ = e;
  }

  GDEdge * GVertex::EastIn () const {
    return eastIn_;
  }

  void GVertex::EastIn ( GDEdge * e ) {
    eastIn_ = e;
  }

  GDEdge * GVertex::EastOut () const {
    return eastOut_;
  }

  void GVertex::EastOut ( GDEdge * e ) {
    eastOut_ = e;
  }

}; // namespace cmpex

#endif // _GVERTEX_H_
