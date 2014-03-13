#ifndef _EXPLORE_EXPLORATIONENGINE_H_
#define _EXPLORE_EXPLORATIONENGINE_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File ExplorationEngine.hpp
//   Created Sep 09, 2011
// ----------------------------------------------------------------------

#include <string>
#include <vector>

using std::string;
using std::vector;

namespace cmpex {

  namespace explore {

    //======================================================================
    // ExplorationEngine is a main class for cmp exploration process.
    //======================================================================

    class ExplorationEngine {

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      ExplorationEngine ();

      virtual ~ExplorationEngine ();
      
      // Main method that invokes the exploration.
      void Explore() const;
      
    private:

      // Deprecated methods: prevent usage
      ExplorationEngine ( const ExplorationEngine& );

      ExplorationEngine& operator = ( const ExplorationEngine& );

      // -------------------------- Attributes -----------------------------

    private:

    };

  } // namespace explore

} // namespace cmpex

#endif // _EXPLORE_EXPLORATIONENGINE_H_
