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

      static vector<double> coreTempEst_;

      static vector<double> L1DTempEst_;

      static vector<double> L1ITempEst_;

      static vector<double> L2TempEst_;

      static vector<double> L3TempEst_;

      static vector<double> MCTempEst_;

      static vector<double> MeshRouterTempEst_;

      static vector<double> MeshLinkNTempEst_; // 4 links per router

      static vector<double> MeshLinkWTempEst_; // 4 links per router

      static bool warmupDone_; // flag set to one after hotspot warmup

      static bool initHotspotDone_; // flag set to one after hotspot initialization

      static double timeSim_; // hotspot simulation time

      static int nBlocks_; // number of blocks in hotspot

      static bool initTracing_; // flag set to one first time PT tracing is called

      static vector<vector<int> > coreTempPredictorIdx_;
      static vector<vector<double> > coreTempPredictorCoeff_;

      static vector<vector<int> > L1DTempPredictorIdx_;
      static vector<vector<double> > L1DTempPredictorCoeff_;

      static vector<vector<int> > L1ITempPredictorIdx_;
      static vector<vector<double> > L1ITempPredictorCoeff_;

      static vector<vector<int> > L2TempPredictorIdx_;
      static vector<vector<double> > L2TempPredictorCoeff_;

      static vector<vector<int> > L3TempPredictorIdx_;
      static vector<vector<double> > L3TempPredictorCoeff_;

      static vector<vector<int> > MCTempPredictorIdx_;
      static vector<vector<double> > MCTempPredictorCoeff_;

      static vector<vector<int> > MeshRouterTempPredictorIdx_;
      static vector<vector<double> > MeshRouterTempPredictorCoeff_;

      static vector<vector<int> > MeshLinkNTempPredictorIdx_; // 4 links per router
      static vector<vector<double> > MeshLinkNTempPredictorCoeff_; // 4 links per router

      static vector<vector<int> > MeshLinkWTempPredictorIdx_; // 4 links per router
      static vector<vector<double> > MeshLinkWTempPredictorCoeff_; // 4 links per router

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

      // Writes power and temperature trace files
      static void WritePowerTempTraces(cmp::Component * cmp, double * power_sim, double * temp_sim);

      // Read temperature predictors from file
      static void ReadTempPredictors(cmp::Component * cmp);

      // Predict next temperatures
      static void PredictTemp(cmp::Component * cmp, vector<double> * power_vec);

      // Maximum estimated temperature among components in the given tile
      static double GetMaxEstTempInTile(int tileIdx);
      
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

      // get temperature estimators ---
      inline static double CoreTempEst(int core_idx) {
        return coreTempEst_[core_idx];
      }

      inline static double L1DTempEst(int l1d_idx) {
        return L1DTempEst_[l1d_idx];
      }

      inline static double L1ITempEst(int l1i_idx) {
        return L1ITempEst_[l1i_idx];
      }

      inline static double L2TempEst(int l2_idx) {
        return L2TempEst_[l2_idx];
      }

      inline static double L3TempEst(int l3_idx) {
        return L3TempEst_[l3_idx];
      }

      inline static double MCTempEst(int mc_idx) {
        return MCTempEst_[mc_idx];
      }

      inline static double MeshRouterTempEst(int rtr_idx) {
        return MeshRouterTempEst_[rtr_idx];
      }

      inline static double MeshLinkNTempEst(int ln_idx) {
        return MeshLinkNTempEst_[ln_idx];
      }

      inline static double MeshLinkWTempEst(int lw_idx) {
        return MeshLinkWTempEst_[lw_idx];
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
