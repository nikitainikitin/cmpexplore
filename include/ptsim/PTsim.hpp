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

      static vector<double> MeshLinkNTemp_; // 4 links per router

      static vector<double> MeshLinkWTemp_; // 4 links per router

      static bool warmupDone_; // flag set to one after hotspot warmup

      static bool initHotspotDone_; // flag set to one after hotspot initialization

      static double timeSim_; // hotspot simulation time

      static int nBlocks_; // number of blocks in hotspot

    public:

      // Calculates transient and steady-state temperature of given configuration
      static int CallHotSpot(cmp::Component * cmp, vector<double> * power_vec = 0,
                             bool silent_mode = false);

      // Terminates HotSpot
      static void EndHotSpot();

      // Saves simulated temperature values to local buffers
      static void SaveSimTemp(cmp::Component * cmp, double * temp_sim);

      // Prints temperature values saved in local buffers
      static void PrintTemp();

      
      // get temperature for one core
      inline static double CoreTemp(int core_idx) { 
	return coreTemp_[core_idx]; 
      }

      // get temperature for one L1D
      inline static double L1DTemp(int l1d_idx) { 
	return L1DTemp_[l1d_idx]; 
      }

      // get temperature for one L1I
      inline static double L1ITemp(int l1i_idx) { 
	return L1ITemp_[l1i_idx]; 
      }

      // get temperature for one L2
      inline static double L2Temp(int l2_idx) { 
	return L2Temp_[l2_idx]; 
      }

      // get temperature for one L3 slice
      inline static double L3Temp(int l3_idx) { 
	return L3Temp_[l3_idx]; 
      }

      // get temperature for one MC
      inline static double MCTemp(int mc_idx) { 
	return MCTemp_[mc_idx]; 
      }

      // get temperature for one router
      inline static double MeshRouterTemp(int rtr_idx) { 
	return MeshRouterTemp_[rtr_idx]; 
      }

      // get temperature for one link North-outbound (South-inbound))
      inline static double MeshLinkNTemp(int ln_idx) { 
	return MeshLinkNTemp_[ln_idx]; 
      }

      // get temperature for one link West-outbound (East-inbound)
      inline static double MeshLinkWTemp(int lw_idx) { 
	return MeshLinkWTemp_[lw_idx]; 
      }

      inline static bool WarmupDone ( ) {
	return warmupDone_;
      }

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
