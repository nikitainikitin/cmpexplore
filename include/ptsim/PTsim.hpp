#ifndef _PTSIM_CALLHOTSPOT_H_
#define _PTSIM_CALLHOTSPOT_H_

#include <vector>

using std::vector;

namespace cmpex {

  namespace cmp {
    class Component;
    class Interconnect;
    class MeshIc;
    class BusIc;
    class URingIc;
    class BRingIc;
    class XBarIc;
  }

  namespace temperature {

    //======================================================================
    // CallHotSpot is used to get transient and steady state temperatures
    // of CMP configuration with current workload.
    //======================================================================

    class PTsim {

      // ---------------------------- Methods ------------------------------

      // temporary buffers to store transient and steady-state 
      // temperature values;
      // indexed by the component idx (core, router, etc)

      static vector<double> coreTemp_;

      static vector<double> L1DTemp_;

      static vector<double> L1ITemp_;

      static vector<double> L2Temp_;

      static vector<double> L3Temp_;

      static vector<double> MCTemp_;

      static vector<double> MeshRouterTemp_;

      static vector<double> MeshLinkTemp_; // 4 links per router

    public:

      // Calculates transient and steady-state temperature of given configuration
      static int CallHotSpot(cmp::Component * cmp);

    public:

      // Constructors & destructor
      PTsim ();

      virtual ~PTsim ();
      
    private:

      // Deprecated methods: prevent usage
      PTsim ( const PTsim& );

      PTsim& operator = ( const PTsim& );

      // -------------------------- Attributes -----------------------------

    private:
      

    };

  } // namespace temperature

} // namespace cmpex

#endif // _PTSIM_CALLHOTSPOT_H_
