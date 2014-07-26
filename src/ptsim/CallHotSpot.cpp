/* 
This is an example of call to HotSpot C functions from an external C++ code. In this
example random and constant power values are used for a single tile configuration or 
a 2x2 configuration. Temperature traces are saved in a file with extension .ttrace. 
Three functions have to be called in sequence: 1) sim_init, 2) sim_main, and 3) sim_exit.

1) Function sim_init initializes HotSpot by passing five file names. The first file is
the HotSpot floorplan descriptor. The value returned by sim_init is the number of blocks
read in the floorplan descriptor. The second file is the configuration for thermal
analysis. The third file contains initial temperature. The fourth file is the name of
an output file for saving blocks steady state temperatures, computed assuming power is 
the average of values used during simulation. The fifth file is the name of an output 
file for saving grid steady state temperatures. Template of sim_init is as follows

int sim_init(char *flp_file, char *config_file, char *init_file, char *steady_file,
char *gridsteady_file)

2) Function sim_main calls the simulator by passing a pointer to an array of power
values (power_sim), a pointer to an array of temperature values (temp_sim), and the 
current simulation time (time_sim) in seconds. It must be called once with time_sim set
to 0.0 so that initial temperatures are read, and then repeatedly for as many time steps 
as required. It uses the values in power_sim to compute the values in temp_sim. The 
convention for the power and temperature arrays is that the ordering follows the same 
ordering of the floorplan file, in which blocks are listed one per line. Therefore the 
size of power/temp is determined strictly by the number of blocks listed in the 
floorplan file. Template is as follows

void sim_main(double *power_sim, double *temp_sim, double time_sim)

3) Function sim_exit closes the simulation and saves temperature in the steady state
file. Template is simple

void sim_exit();
 */
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

#include "PTsim.hpp"
#include "Component.hpp"
#include "CmpConfig.hpp"
#include "Config.hpp"
#include "MeshIc.hpp"
#include "Cluster.hpp"
#include "Util.hpp"

extern "C" {
#include "PTsim.h"
}

#define MAXSTRING 128

using namespace std;

namespace cmpex {
  
  extern cmp::CmpConfig cmpConfig;
  extern Config config;

  using namespace cmp;

  namespace temperature {

//=======================================================================
/*
 * Constructors and destructor
 */
PTsim::PTsim() {}

PTsim::~PTsim() {}

vector<double> PTsim::coreTemp_;
vector<double> PTsim::L1DTemp_;
vector<double> PTsim::L1ITemp_;
vector<double> PTsim::L2Temp_;
vector<double> PTsim::L3Temp_;
vector<double> PTsim::MCTemp_;
vector<double> PTsim::MeshRouterTemp_;
vector<double> PTsim::MeshLinkTemp_;
vector<double> PTsim::MeshLinkNTemp_;
vector<double> PTsim::MeshLinkWTemp_;

vector<double> PTsim::coreTempEst_;
vector<double> PTsim::L1DTempEst_;
vector<double> PTsim::L1ITempEst_;
vector<double> PTsim::L2TempEst_;
vector<double> PTsim::L3TempEst_;
vector<double> PTsim::MCTempEst_;
vector<double> PTsim::MeshRouterTempEst_;
vector<double> PTsim::MeshLinkNTempEst_;
vector<double> PTsim::MeshLinkWTempEst_;

vector<vector<int> > PTsim::coreTempPredictorIdx_;
vector<vector<double> > PTsim::coreTempPredictorCoeff_;
vector<vector<int> > PTsim::L1DTempPredictorIdx_;
vector<vector<double> > PTsim::L1DTempPredictorCoeff_;
vector<vector<int> > PTsim::L1ITempPredictorIdx_;
vector<vector<double> > PTsim::L1ITempPredictorCoeff_;
vector<vector<int> > PTsim::L2TempPredictorIdx_;
vector<vector<double> > PTsim::L2TempPredictorCoeff_;
vector<vector<int> > PTsim::L3TempPredictorIdx_;
vector<vector<double> > PTsim::L3TempPredictorCoeff_;
vector<vector<int> > PTsim::MCTempPredictorIdx_;
vector<vector<double> > PTsim::MCTempPredictorCoeff_;
vector<vector<int> > PTsim::MeshRouterTempPredictorIdx_;
vector<vector<double> > PTsim::MeshRouterTempPredictorCoeff_;
vector<vector<int> > PTsim::MeshLinkNTempPredictorIdx_; // 4 links per router
vector<vector<double> > PTsim::MeshLinkNTempPredictorCoeff_; // 4 links per router
vector<vector<int> > PTsim::MeshLinkWTempPredictorIdx_; // 4 links per router
vector<vector<double> > PTsim::MeshLinkWTempPredictorCoeff_; // 4 links per router


bool PTsim::warmupDone_;
bool PTsim::initTracing_;
bool PTsim::initHotspotDone_;
double PTsim::timeSim_;
int PTsim::nBlocks_;


//=======================================================================
/*
 * Calculates transient and steady-state temperature of cmp configuration.
 */


string config_filename("./src/ptsim/cmp.config");
string null_filename("(null)");

int PTsim::CallHotSpot(cmp::Component * cmp, vector<double> * power_vec, bool silent_mode)
{
  int i,j;
  double *power_sim,*warmup_power,*temp_sim, time_sim;
  ifstream pin;
  ofstream tout;
  int n_blocks;
  char flp[MAXSTRING];
  char cfg[MAXSTRING];
  char initt[MAXSTRING];
  char stdyt[MAXSTRING];
  char gstdyt[MAXSTRING];
  char nullt[MAXSTRING];
  string line;
  double total_power=0.0;


  // find out mesh size to define file names
  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());
  int meshX = mic->ColNum();
  int meshY = mic->RowNum();

  string cmpSize = "cmp" + IntToStr(meshX) + "x" + IntToStr(meshY);
  string floorplan_filename("./src/ptsim/" + cmpSize + ".flp");
  if (!FileExists(floorplan_filename)) {
    cout << "-E- Floorplan file " << floorplan_filename << " does not exist -> Exiting" << endl;
    return -1;
  }
  string inittemp_filename("./src/ptsim/" + cmpSize + ".init");
  string steadytemp_filename("./src/ptsim/" + cmpSize + ".steady");
  string ttrace_filename("./src/ptsim/" + cmpSize + ".ttrace");
  string gridsteadytemp_filename("./src/ptsim/" + cmpSize + ".grid.steady");

  // adapt C++ strings to C strings
  strcpy(flp,floorplan_filename.c_str());
  strcpy(cfg,config_filename.c_str());
  strcpy(initt,inittemp_filename.c_str());
  strcpy(nullt,null_filename.c_str());
  strcpy(stdyt,steadytemp_filename.c_str());
  strcpy(gstdyt,gridsteadytemp_filename.c_str());

  // initialize hotspot with steady temp target file same as init temp file 
  if (!warmupDone_) {
    nBlocks_=sim_init(flp,cfg,nullt,initt,gstdyt);
    ReadTempPredictors(cmp);
  }
  n_blocks = nBlocks_;

  if (!silent_mode) cout << "Number of blocks = " << n_blocks << endl;

  // create power vector
  power_sim = new double[n_blocks];
  warmup_power = new double[n_blocks];
  // set warmup power:
  if (!warmupDone_) {
    //double max_power = config.MaxPower();
    double tdp = 130.0;
    double max_power = 0.5*tdp; // using half a typical TDP to warm up the system
    double proc_power = max_power * 0.85;
    double uncore_power = max_power * 0.1;
    double mc_power = max_power * 0.05 / cmpConfig.MemCtrlCnt();
    double core_power = proc_power * 0.5 / cmpConfig.ProcCnt();
    double l1i_power = proc_power * 0.3 / cmpConfig.ProcCnt();
    double l1d_power = proc_power * 0.1 / cmpConfig.ProcCnt();
    double l2_power = proc_power * 0.1 / cmpConfig.ProcCnt();
    double l3_power = uncore_power * 0.3 / cmpConfig.MemCnt();
    double rtr_power = uncore_power * 0.7 / cmpConfig.ProcCnt();
    double linkw_power = uncore_power * 0.0 / cmpConfig.ProcCnt();
    double linkn_power = uncore_power * 0.0 / cmpConfig.ProcCnt();
    for(i=0;i<cmpConfig.MemCtrlCnt();i++){
      warmup_power[i]=mc_power;
    }
    int n_tiles = (n_blocks - cmpConfig.MemCtrlCnt()) / 8;
    for(i=0;i<n_tiles;i++) {
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+0]=linkw_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+1]=rtr_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+2]=l3_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+3]=linkn_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+4]=l2_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+5]=core_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+6]=l1d_power;
      warmup_power[i*8+cmpConfig.MemCtrlCnt()+7]=l1i_power;
    }
    //for(i=0;i<n_blocks;i++){
    //if (!power_vec)
    //	warmup_power[i]=0;
    //else
    //	warmup_power[i]=(*power_vec)[i];
    ////warmup_power[i]=block_power[i];
    //}
    // set block_power to warmup_power
    for(i=0;i<n_blocks;i++){
      power_sim[i]=warmup_power[i];
    }
  }



  // create temperature vector
  temp_sim= new double[n_blocks];

  // trace file opening
  tout.open(ttrace_filename.c_str());

  if (warmupDone_) {
    //input power (from CMPexplore
    if (!power_vec) { // no power vector - read values from file
      pin.open("ptsim_power.txt");
      
      i=0;
      while(getline(pin,line)) {
	if ((line == "") || (!line.find("#")))
	  continue;
	stringstream ss;
	ss << line;
	double temp_double;
	while(ss >> temp_double) {
	  if (1) // every tile is ON
	    //if (((i-2)/8)%2==0) // every other tile is ON
	    //if (((i-2)>=(n_blocks-2)/4) && (i-2<(n_blocks-2)/4+n_blocks/2)) // half CMP centered is ON
	    //if ((i-2)>=((n_blocks-2)/2)) // top half CMP is ON
	    power_sim[i] = temp_double;
	  else
	    power_sim[i] = 0;
	  total_power += power_sim[i];
	  if (!silent_mode) cout << power_sim[i] << endl;
	  i++;
	}
      }
      pin.close();
    }
    else { // power vector provided
      for (i = 0; i < power_vec->size(); ++i) {
	power_sim[i] = (*power_vec)[i];
	//cout << power_sim[i] << ' ';
      }
      //cout << endl;
    }
    if (!silent_mode) cout << "TOTAL POWER = " << total_power << endl;
    if(i != n_blocks){
      cout << "\nERROR: power elements /= number of blocks\n\n";
      cout << i << ' ' << n_blocks << endl;
      return -1;
    }
  }
    
    
    // set block power to transient values
    //for(i=0;i<n_blocks;i++){
    //power_sim[i]=block_power[i];
    //}




  if (!warmupDone_) {
    // initialize simulation time
    time_sim = 0.0;
    // initialization of hotspot with 1st call to sim_main, time is 0.0
    sim_main(power_sim,temp_sim,time_sim,silent_mode);
    // increase time and rerun hotspot, save intermediate temp values to trace file
    time_sim += 0.001;
    sim_main(power_sim,temp_sim,time_sim,silent_mode);
    // terminate hotspot and save steady file in init file
    sim_exit();
    timeSim_ = 0.0;
    warmupDone_ = 1;
  }
  else {
    
    
    if (!initHotspotDone_) {
      initHotspotDone_ = 1;
      // re-initialize simulation time
      time_sim = 0.0;
      // re-initialize hotspot with correct init file
      sim_init(flp,cfg,initt,stdyt,gstdyt);
      
      // re-initialization of hotspot with 1st call to sim_main, time is 0.0
      sim_main(power_sim,temp_sim,time_sim,silent_mode);
      
      // increase time and rerun hotspot, save intermediate temp values to trace file
      time_sim += 0.001;
      timeSim_ = time_sim;
    }
    else {
      time_sim = timeSim_;
      sim_main(power_sim,temp_sim,time_sim,silent_mode);
      //output instantaneous temperature trace
      for(j=0;j<n_blocks;j++)
	tout << setiosflags(ios::fixed) << setprecision(2) << temp_sim[j]-273.15 << "\t";
      tout << endl;
      timeSim_ += 0.001;
    }
    
  }
  // closing
  tout.close();


  // convert K to C degrees
  for(j=0;j<n_blocks;j++) {
    temp_sim[j] -= 273.15;
  }

  SaveSimTemp(cmp, temp_sim);
  if (initHotspotDone_) {
    WritePowerTempTraces(cmp, power_sim, temp_sim);
  }
  //PrintTemp();

  delete [] power_sim;
  delete [] temp_sim;
  return 0;
}

void PTsim::EndHotSpot() {
  // terminate hotspot
  if (warmupDone_) {
    sim_exit();
  }
}

//=======================================================================
/*
 * Saves simulated temperature values to local buffers.
 * Assumes specific order of values in temp_sim:
 * - values for MemCtrl,
 * - per tile: LinkW  RTR  L3  LinkN  L2  Core  L1D  L1I (one line per component).
 * The order is the same as in the ptsim_power.txt file.
 */

void PTsim::SaveSimTemp(cmp::Component * cmp, double * temp_sim) {
  coreTemp_.assign(cmpConfig.ProcCnt(), 0.0);
  L1DTemp_.assign(cmpConfig.ProcCnt(), 0.0);
  L1ITemp_.assign(cmpConfig.ProcCnt(), 0.0);
  L2Temp_.assign(cmpConfig.ProcCnt(), 0.0);
  L3Temp_.assign(cmpConfig.MemCnt(), 0.0);
  MCTemp_.assign(cmpConfig.MemCtrlCnt(), 0.0);
  MeshRouterTemp_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkNTemp_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkWTemp_.assign(cmpConfig.ProcCnt(), 0.0);

  int cnt = 0;

  // MC
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    MCTemp_[mc] = temp_sim[cnt];
    ++cnt;
  }

  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());
  for (int tile_id = 0; tile_id < mic->ColNum()*mic->RowNum(); ++tile_id) {

    // LinkW
    /*MeshLinkTemp_[tile_id*4+int(RDWEST)] = temp_sim[cnt];
    if (tile_id%mic->ColNum() != 0) {
      MeshLinkTemp_[(tile_id-1)*4+int(RDEAST)] = temp_sim[cnt];
      }*/
    MeshLinkWTemp_[tile_id] = temp_sim[cnt];
    ++cnt;

    // RTR
    MeshRouterTemp_[tile_id] = temp_sim[cnt++];

    // L3
    L3Temp_[tile_id] = temp_sim[cnt++];

    // LinkN
    /*MeshLinkTemp_[tile_id*4+int(RDNORTH)] = temp_sim[cnt];
    if (tile_id >= mic->ColNum()) { // no north links for tiles in the upper row
      MeshLinkTemp_[(tile_id-mic->ColNum())*4+int(RDSOUTH)] = temp_sim[cnt];
      }*/
    MeshLinkNTemp_[tile_id] = temp_sim[cnt];
    ++cnt;

    // L2
    L2Temp_[tile_id] = temp_sim[cnt++];

    // Core
    coreTemp_[tile_id] = temp_sim[cnt++];

    // L1D
    L1DTemp_[tile_id] = temp_sim[cnt++];

    // L1I
    L1ITemp_[tile_id] = temp_sim[cnt++];
  }

}

//=======================================================================
/*
 * Prints temperature values saved in local buffers
 */

void PTsim::PrintTemp() {

  cout << "  CoreTemp: ";
  for (vector<double>::const_iterator it = coreTemp_.begin(); it != coreTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  L1DTemp: ";
  for (vector<double>::const_iterator it = L1DTemp_.begin(); it != L1DTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  L1ITemp: ";
  for (vector<double>::const_iterator it = L1ITemp_.begin(); it != L1ITemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  L2Temp: ";
  for (vector<double>::const_iterator it = L2Temp_.begin(); it != L2Temp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  L3Temp: ";
  for (vector<double>::const_iterator it = L3Temp_.begin(); it != L3Temp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  MeshRouterTemp: ";
  for (vector<double>::const_iterator it = MeshRouterTemp_.begin(); it != MeshRouterTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  MeshLinkWTemp: ";
  for (vector<double>::const_iterator it = MeshLinkWTemp_.begin(); it != MeshLinkWTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  MeshLinkNTemp: ";
  for (vector<double>::const_iterator it = MeshLinkNTemp_.begin(); it != MeshLinkNTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

  cout << "  MC: ";
  for (vector<double>::const_iterator it = MCTemp_.begin(); it != MCTemp_.end(); ++it) {
    cout << setiosflags(ios::fixed) << setprecision(2) << (*it) << ' ';
  }
  cout << endl;

}


//=======================================================================
/*
 * Save power and temperature transient traces
 */

void PTsim::WritePowerTempTraces(cmp::Component * cmp, double * power_sim, double * temp_sim) {

  string ttrace = "cmp.ttrace";
  string ptrace = "cmp.ptrace";
  ofstream tout, pout;
  int i;

  if (!initTracing_) {
    initTracing_ = 1;
    tout.open(ttrace.c_str());
    pout.open(ptrace.c_str());
  }
  else {
    tout.open(ttrace.c_str(), ios::app);
    pout.open(ptrace.c_str(), ios::app);
  }

  int cnt = 0;

  // MC
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt] << "\t";
    ++cnt;
  }

  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());
  for (int tile_id = 0; tile_id < mic->ColNum()*mic->RowNum(); ++tile_id) {

    // LinkW
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // RTR
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // L3
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // LinkN
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // L2
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // Core
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // L1D
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

    // L1I
    pout << power_sim[cnt] << "\t";
    tout << temp_sim[cnt++] << "\t";

  }
  
  tout << endl;
  pout << endl;
 
  tout.close();
  pout.close();
  

}

//=======================================================================
/*
 * Read Temperature predictors coefficients
 */

void PTsim::ReadTempPredictors(cmp::Component * cmp) {


  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());
  int meshX = mic->ColNum();
  int meshY = mic->RowNum();
  string predictor_filename("./src/ptsim/cmp" + IntToStr(meshX) + "x" + IntToStr(meshY) + ".prd");
  if (!FileExists(predictor_filename)) {
    cout << "-E- Temperature predictors file  " << predictor_filename << " does not exist -> Exiting" << endl;
    exit(1);
  }
  ifstream predin;
  predin.open(predictor_filename.c_str());
  string line;
  int idx;
  double coeff;
  vector<int> idx_vec;
  vector<double> coeff_vec;

  // MC
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      MCTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      MCTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();
  }

  // TILES
  for (int tile_id = 0; tile_id < mic->ColNum()*mic->RowNum(); ++tile_id) {
    // LINK W
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      MeshLinkWTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      MeshLinkWTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // RTR
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      MeshRouterTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      MeshRouterTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // L3
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      L3TempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      L3TempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // LinkN
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      MeshLinkNTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      MeshLinkNTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // L2
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      L2TempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      L2TempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // Core
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      coreTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      coreTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // L1D
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      L1DTempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      L1DTempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();

    // L1I
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> idx) {
	idx_vec.push_back(idx);
      }
      L1ITempPredictorIdx_.push_back(idx_vec);
    }
    if (getline(predin, line)) {
      istringstream iss(line);
      while (iss >> coeff) {
	coeff_vec.push_back(coeff);
      }
      L1ITempPredictorCoeff_.push_back(coeff_vec);
    }
    idx_vec.clear();
    coeff_vec.clear();
  }
  predin.close();
}

//=======================================================================
/*
 * Use actual temperatures and current power to predict next temperatures
 */

void PTsim::PredictTemp(cmp::Component * cmp, vector<double> * power_vec) {

  coreTempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  L1DTempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  L1ITempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  L2TempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  L3TempEst_.assign(cmpConfig.MemCnt(), 0.0);
  MCTempEst_.assign(cmpConfig.MemCtrlCnt(), 0.0);
  MeshRouterTempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkNTempEst_.assign(cmpConfig.ProcCnt(), 0.0);
  MeshLinkWTempEst_.assign(cmpConfig.ProcCnt(), 0.0);

  Cluster * clCmp = static_cast<Cluster*>(cmp);
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  double acctemp;

  double TEXT = 45.0; // ambient temperature

  double maxtemp = 0.0;

  // MC
  for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
    //acctemp = MCTemp_[mc];
    acctemp = TEXT + MCTempPredictorCoeff_[mc][MCTempPredictorCoeff_[mc].size()-1]*(MCTemp_[mc]-TEXT);
    for (int idx = 0; idx < MCTempPredictorIdx_[mc].size(); idx++) {
      acctemp += MCTempPredictorCoeff_[mc][idx]*(*power_vec)[MCTempPredictorIdx_[mc][idx]];
    }
    MCTempEst_[mc]=acctemp;
    maxtemp = max(maxtemp,acctemp);
  }

  // TILES
  for (int tile = 0; tile < mic->ColNum()*mic->RowNum(); ++tile) {

    //LINKW
    //acctemp = MeshLinkWTemp_[tile];
    acctemp = TEXT + MeshLinkWTempPredictorCoeff_[tile][MeshLinkWTempPredictorCoeff_[tile].size()-1]*(MeshLinkWTemp_[tile]-TEXT);
    for (int idx = 0; idx < MeshLinkWTempPredictorIdx_[tile].size(); idx++) {
      acctemp += MeshLinkWTempPredictorCoeff_[tile][idx]*(*power_vec)[MeshLinkWTempPredictorIdx_[tile][idx]];
    }
    MeshLinkWTempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //RTR
    //acctemp = MeshRouterTemp_[tile];
    acctemp = TEXT + MeshRouterTempPredictorCoeff_[tile][MeshRouterTempPredictorCoeff_[tile].size()-1]*(MeshRouterTemp_[tile]-TEXT);
    for (int idx = 0; idx < MeshRouterTempPredictorIdx_[tile].size(); idx++) {
      acctemp += MeshRouterTempPredictorCoeff_[tile][idx]*(*power_vec)[MeshRouterTempPredictorIdx_[tile][idx]];
    }
    MeshRouterTempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //L3
    //acctemp = L3Temp_[tile];
    acctemp = TEXT + L3TempPredictorCoeff_[tile][L3TempPredictorCoeff_[tile].size()-1]*(L3Temp_[tile]-TEXT);
    for (int idx = 0; idx < L3TempPredictorIdx_[tile].size(); idx++) {
      acctemp += L3TempPredictorCoeff_[tile][idx]*(*power_vec)[L3TempPredictorIdx_[tile][idx]];
    }
    L3TempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //LINKN
    //acctemp = MeshLinkNTemp_[tile];
    acctemp = TEXT + MeshLinkNTempPredictorCoeff_[tile][MeshLinkNTempPredictorCoeff_[tile].size()-1]*(MeshLinkNTemp_[tile]-TEXT);
    for (int idx = 0; idx < MeshLinkNTempPredictorIdx_[tile].size(); idx++) {
      acctemp += MeshLinkNTempPredictorCoeff_[tile][idx]*(*power_vec)[MeshLinkNTempPredictorIdx_[tile][idx]];
    }
    MeshLinkNTempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //L2
    //acctemp = L2Temp_[tile];
    acctemp = TEXT + L2TempPredictorCoeff_[tile][L2TempPredictorCoeff_[tile].size()-1]*(L2Temp_[tile]-TEXT);
    for (int idx = 0; idx < L2TempPredictorIdx_[tile].size(); idx++) {
      acctemp += L2TempPredictorCoeff_[tile][idx]*(*power_vec)[L2TempPredictorIdx_[tile][idx]];
    }
    L2TempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //CORE
    //acctemp = coreTemp_[tile];
    acctemp = TEXT + coreTempPredictorCoeff_[tile][coreTempPredictorCoeff_[tile].size()-1]*(coreTemp_[tile]-TEXT);
    for (int idx = 0; idx < coreTempPredictorIdx_[tile].size(); idx++) {
      acctemp += coreTempPredictorCoeff_[tile][idx]*(*power_vec)[coreTempPredictorIdx_[tile][idx]];
    }
    coreTempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //L1D
    //acctemp = L1DTemp_[tile];
    acctemp = TEXT + L1DTempPredictorCoeff_[tile][L1DTempPredictorCoeff_[tile].size()-1]*(L1DTemp_[tile]-TEXT);
    for (int idx = 0; idx < L1DTempPredictorIdx_[tile].size(); idx++) {
      acctemp += L1DTempPredictorCoeff_[tile][idx]*(*power_vec)[L1DTempPredictorIdx_[tile][idx]];
    }
    L1DTempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

    //L1I
    //acctemp = L1ITemp_[tile];
    acctemp = TEXT + L1ITempPredictorCoeff_[tile][L1ITempPredictorCoeff_[tile].size()-1]*(L1ITemp_[tile]-TEXT);
    for (int idx = 0; idx < L1ITempPredictorIdx_[tile].size(); idx++) {
      acctemp += L1ITempPredictorCoeff_[tile][idx]*(*power_vec)[L1ITempPredictorIdx_[tile][idx]];
    }
    L1ITempEst_[tile]=acctemp;
    maxtemp = max(maxtemp,acctemp);

  }

}

//=======================================================================
/*
 * Get maximum estimated temperature among components in the given tile.
 */

double PTsim::GetMaxEstTempInTile(int tileIdx) {
  double maxTemp = 0.0;

  maxTemp = max(CoreTempEst(tileIdx), maxTemp);
  maxTemp = max(L1DTempEst(tileIdx), maxTemp);
  maxTemp = max(L1ITempEst(tileIdx), maxTemp);
  maxTemp = max(L2TempEst(tileIdx), maxTemp);
  maxTemp = max(L3TempEst(tileIdx), maxTemp);
  maxTemp = max(MeshRouterTempEst(tileIdx), maxTemp);
  maxTemp = max(MeshLinkNTempEst(tileIdx), maxTemp);
  maxTemp = max(MeshLinkWTempEst(tileIdx), maxTemp);

  return maxTemp;
}

//=======================================================================
/*
 * Get maximum temperature among components in the given tile.
 */

double PTsim::GetMaxTempInTile(int tileIdx) {
  double maxTemp = 0.0;

  maxTemp = max(CoreTemp(tileIdx), maxTemp);
  maxTemp = max(L1DTemp(tileIdx), maxTemp);
  maxTemp = max(L1ITemp(tileIdx), maxTemp);
  maxTemp = max(L2Temp(tileIdx), maxTemp);
  maxTemp = max(L3Temp(tileIdx), maxTemp);
  maxTemp = max(MeshRouterTemp(tileIdx), maxTemp);
  maxTemp = max(MeshLinkNTemp(tileIdx), maxTemp);
  maxTemp = max(MeshLinkWTemp(tileIdx), maxTemp);

  return maxTemp;
}

//=======================================================================


  } // namespace temperature

} //namespace cmpex
