#ifndef _GDEDGE_H_
#define _GDEDGE_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GDEdge.hpp
//   Created May 05, 2011
// ----------------------------------------------------------------------

namespace cmpex {

  class GGraph;
  class GVertex;

  //======================================================================
  // GDEdge represents a directed edge between two vertices in grid graph.
  //======================================================================

  class GDEdge {

    friend class GGraph;

    // ---------------------------- Methods ------------------------------

  public:

    // Accessors
    inline int Idx () const;
    
    inline const GVertex * Src () const;

    inline GVertex * Src ();

    inline const GVertex * Dst () const;

    inline GVertex * Dst ();

    // Direction of edge in grid graph:
    // horizontal if grid cells are in the same row,
    // vertical if in the same column.
    inline bool IsHorizontal () const;

    inline bool IsVertical () const;

    // Debug methods
    void Print ();
    
  protected:
    
    // Constructors & destructor
    GDEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal );

    virtual ~GDEdge ();
    
    inline void Src ( GVertex * v );

    inline void Dst ( GVertex * v );

  private:

    // Deprecated methods: prevent usage
    GDEdge ( const GDEdge& );

    GDEdge& operator = ( const GDEdge& );

    // -------------------------- Attributes -----------------------------

  private:

    int idx_;
    
    // Two endpoints: source and destination
    GVertex * src_;
    
    GVertex * dst_;
    
    // Direction
    bool horizontal_;
    
  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int GDEdge::Idx () const {
    return idx_;
  }

  const GVertex * GDEdge::Src () const {
    return src_;
  }

  GVertex * GDEdge::Src () {
    return src_;
  }

  const GVertex * GDEdge::Dst () const {
    return dst_;
  }

  GVertex * GDEdge::Dst () {
    return dst_;
  }

  void GDEdge::Src ( GVertex * v ) {
    src_ = v;
  }
  
  void GDEdge::Dst ( GVertex * v ) {
    dst_ = v;
  }
  
  bool GDEdge::IsHorizontal () const {
    return horizontal_;
  }

  bool GDEdge::IsVertical () const {
    return !horizontal_;
  }

}; // namespace cmpex

#endif // _GDEDGE_H_
