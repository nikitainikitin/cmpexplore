#ifndef _GGRAPH_H_
#define _GGRAPH_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File GGraph.hpp
//   Created May 03, 2011
// ----------------------------------------------------------------------

#include <vector>

namespace cmpex {
  
  class GVertex;
  class GDEdge;

  //======================================================================
  // GGraph is a grid graph class, with two ways of access to its elements:
  // 1. Random index-based access to cells (that are also graph vertices)
  // 2. Pointer-based access to vertices (that are also grid cells) and
  //    sequential navigation over the graph through neighboring edges.
  //======================================================================

  class GGraph {
    
    typedef std::vector<GVertex*> VertexArray;

    typedef std::vector<GDEdge*> EdgeArray;

    // --------------------------- Iterators -----------------------------
    
    typedef VertexArray::iterator VIter;
    
    typedef EdgeArray::iterator EIter;
    
  public:
    
    typedef VertexArray::const_iterator VCIter;
    
    typedef EdgeArray::const_iterator ECIter;
    
    // ---------------------------- Methods ------------------------------

  public:

    // Constructors & destructor
    GGraph ();

    virtual ~GGraph ();

    // Factory method for vertex: creates Tile object
    virtual GVertex * CreateVertex ( int idx );

    // Factory method for edge: creates Link object
    virtual GDEdge * CreateEdge ( int idx, GVertex * src, GVertex * dst, bool horizontal );
    
    // Initializer
    virtual void Init ( int colNum, int rowNum );

    // Accessors
    inline int ColNum () const;

    inline int RowNum () const;
    
    inline int VCnt () const;

    inline int ECnt () const;
    
    inline VCIter VBegin () const;
    
    inline VCIter VEnd () const;
    
    inline ECIter EBegin () const;
    
    inline ECIter EEnd () const;

    inline GVertex * GetVertex ( int x, int y );

    // Debug methods
    void Print () const;

  protected:
    
    inline void AddVertex ( GVertex * v );
    
    inline void AddEdge ( GDEdge * e );
    
  private:

    // Deprecated methods: prevent usage
    GGraph ( const GGraph& );

    GGraph& operator = ( const GGraph& );
    
    // Internal procedure to create edges and connect vertices
    void ConnectVertices();

    // -------------------------- Attributes -----------------------------

  private:

    // Grid sizes
    int colNum_;
    
    int rowNum_;

    // Vertex and edge count
    int vCnt_;
    
    int eCnt_;

    // Vertex and edge containers
    VertexArray vertices_;
    
    EdgeArray edges_;

  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int GGraph::ColNum () const {
    return colNum_;
  }

  int GGraph::RowNum () const {
    return rowNum_;
  }
  
  int GGraph::VCnt () const {
    return vCnt_;
  }

  int GGraph::ECnt () const {
    return eCnt_;
  }
  
  GGraph::VCIter GGraph::VBegin () const {
    return vertices_.begin();
  }

  GGraph::VCIter GGraph::VEnd () const {
    return vertices_.end();
  }

  GGraph::ECIter GGraph::EBegin () const {
    return edges_.begin();
  }

  GGraph::ECIter GGraph::EEnd () const {
    return edges_.end();
  }

  GVertex * GGraph::GetVertex( int x, int y ) {
    return vertices_[x+y*ColNum()];
  }
  
  void GGraph::AddVertex ( GVertex * v ) {
    vertices_.push_back(v);
    ++vCnt_;
  }

  void GGraph::AddEdge ( GDEdge * e ) {
    edges_.push_back(e);
    ++eCnt_;
  }

}; // namespace cmpex

#endif // _GGRAPH_H_
