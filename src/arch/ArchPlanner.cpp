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

#include <iostream>
#include <string>
#include <limits>
#include <stdexcept>
#include <iomanip>
#include <cmath>
#include <sstream>

#include "ArchPlanner.hpp"
#include "ArchConfig.hpp"
#include "Config.hpp"
#include "ParamIterator.hpp"
#include "Util.hpp"
#include "RouterDefs.hpp"

using namespace std;

namespace cmpex {

  extern Config config;

  namespace arch {

//=======================================================================
/*
 * Constructors and destructor
 */

ArchPlanner::ArchPlanner() :
  gMeshDimXIter_(0), gMeshDimYIter_(0), clusterIcTypeIter_(0), numL3Iter_(0)
{
  // initialize internal iterators
  ProcIter().assign(config.ProcCnt(), 0);

  // create parameter iterators
  ParamIters().push_back(new L3SetIter(this));
  ParamIters().push_back(new arch::ProcL1SizeIter(this));
  ParamIters().push_back(new arch::ProcL2SizeIter(this));
  ParamIters().push_back(new ProcSetIter(this));
  ParamIters().push_back(new GMeshDimIter(this));
  ParamIters().push_back(new arch::ClusterIcTypeIter(this));

  for (PCIter it = ParamIters().begin(); it != ParamIters().end(); ++it) {
    if (!(*it)->Init()) {
      std::cerr << "-E- ArchPlanner: no values defined for " << (*it)->Name() << std::endl;
      throw std::out_of_range((*it)->Name());
    }
  }
}

ArchPlanner::~ArchPlanner()
{
  for (PCIter it = ParamIters().begin(); it != ParamIters().end(); ++it)
    delete *it;
}

//=======================================================================
/*
 * The method generates next (according to the current status) ArchConfig.
 * The ArchConfig ownership is transferred to the calling function.
 */

ArchConfig * ArchPlanner::GenerateNextArchConfig()
{
  ArchConfig * ac = 0;

  if (AllItersValid()) {                      // current state produces valid config
    ac = GenerateCurrentArchConfig();
    //ac = reinterpret_cast<ArchConfig*>(0x1);

    // Now advance to the next valid state (if exists).
    // We have to advance first iterator and propagate the advance
    // to the following iterators in case the previous one becomes invalid.
    // Also we have to assure that the obtained state is valid for all iterators.
    // This may not be the case when advancing some later iterator makes the
    // previous one invalid. In the latter case we have to keep repeating the advance
    // of the first iterator until a valid state is obtained or end of iterators is reached.
    bool propagate;
    do {                                      // advance till next valid state or end
      bool end_reached = false;
      for (PCIter it = ParamIters().begin(); it != ParamIters().end(); ++it) { // propagation cycle
        (*it)->Advance();                     // advance current iterator
        if ((*it)->Valid()) {                 // current iterator is valid
          break;                              // advance propagation finished
        }
        else {                                // current iterator is invalid
          if (it == --ParamIters().end()) {   // end reached if at last iterator
            end_reached = true;
          }
          else {
            (*it)->Init();                    // reset current iterator
          }
        }
      }

      propagate = !AllItersValid() && !end_reached; // keep advance propagation?
    }
    while (propagate);
  }

  return ac;
}

//=======================================================================
/*
 * The method generates ArchConfig from the current configuration.
 * The new ArchConfig object is created and its ownership is transferred
 * to the calling function.
 */

ArchConfig * ArchPlanner::GenerateCurrentArchConfig()
{
  const int DUMP_NI = 1; // dump NI into config file: 0 - no, 1 - yes

  // 1. calculate parameters of the configuration =======================================
  UInt x = config.GMeshDimXVec()[GMeshDimXIter()];
  UInt y = config.GMeshDimYVec()[GMeshDimYIter()];

  UInt procPerCluster = 0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    //cout << "p[" << p << "] = " << ProcIter()[p] << ' ';
    procPerCluster += ProcIter()[p];
  }
  //cout << "total = " << procPerCluster << endl;

  UInt memPerCluster = config.NumL3PerCluster()[NumL3Iter()];

  // calc L3 memory size
  double clusterArea = config.MaxArea() / (x*y);
  double procAreaPerCluster = 0.0;
  for (int p = 0; p < config.ProcCnt(); ++p) {
    double l1Size = config.ProcL1Size(p, ProcL1SizeIter()[p]);
    double l2Size = config.ProcL2Size(p, ProcL2SizeIter()[p]);
    double procArea = config.ProcArea(p) +(l1Size+l2Size)*config.MemDensity();
    procAreaPerCluster += procArea*ProcIter()[p];
  }
  //cout << "AP: ProcArea/cluster = " << procAreaPerCluster << endl;
  //cout << "AP: RouterArea = " << RouterArea(config.Tech(), config.LinkWidth()) << endl;
  double memAreaPerCluster = clusterArea - procAreaPerCluster -
      RouterArea(config.Tech(), config.LinkWidth());
  double L3Area = memAreaPerCluster/memPerCluster;
  double L3Size = L3Area/config.MemDensity();
  //cout << "AP: L3Area/cl = " << L3Area << endl;

  double adjL3Size = AdjustL3Size(L3Size);
  //cout << "L3 = " << L3Size << ", adjL3 = " << adjL3Size << ", diff = "
  //     << double(x*y)*(L3Size-adjL3Size)*config.MemDensity() << endl;
  // update area
  double configArea = config.MaxArea()-x*y*(L3Size-adjL3Size)*config.MemDensity();

  L3Size = adjL3Size;

  // effective L3 size
  UInt procCnt = procPerCluster*x*y;
  double L3SizeEff = L3Size*x*y*memPerCluster / (procCnt - config.L3ShareDegreeOfPNum(procCnt) + 1);

  //DASSERT(L3Size >= 0.0);

  if (L3Size < E_DOUBLE)
    memPerCluster = 0;

  const int packetSize = 512; // packet size in bits
  int packetLength = (config.LinkWidth() == 1024) ? 1 :
                       1 + int(ceil(double(packetSize)/double(config.LinkWidth())));

  // 2. write ArchConfig ================================================================
  ArchConfig * ac = new ArchConfig();
  ac->Area(configArea);

  (*ac) << "PARAM UnitLen=1.0e-3\n"
        << "PARAM MemDensity=" << config.MemDensity() << "\n"
        << "PARAM L3LatencyDef=" << config.L3LatencyOfSize(L3Size) << "\n"
        << "PARAM MemReplySize=" << packetLength << "\n"
        << "PARAM LinkWidth=" << config.LinkWidth() << "\n"
        << "PARAM NiDelay=3\n"
        << "PARAM WlFile=" << config.WlFile() << "\n"
        //<< "PARAM L3ClusterSize=0\n"
        << "PARAM UFrequency=" << config.UFreq() << "\n"
        << "PARAM UVoltage=" << config.UVolt() << "\n";

  // ----- start processor defines
  for (int ptype = 0; ptype < config.ProcCnt(); ++ptype) {
    const double L1Size = config.ProcL1Size(ptype, ProcL1SizeIter()[ptype]);
    const double L2Size = config.ProcL2Size(ptype, ProcL2SizeIter()[ptype]);

    // Option 1 (average): initial distribution from the miss curve
    /*const double L1AccProb = 1.0 - config.MissRatioOfMemSize(L1Size);
    const double L2AccProb = config.MissRatioOfMemSize(L1Size) - config.MissRatioOfMemSize(L2Size+L1Size);
    const double L3AccProb = (L3Size < E_DOUBLE) ? 0.0 :
      (config.MissRatioOfMemSize(L2Size+L1Size) - config.MissRatioOfMemSize(L3SizeEff+L2Size+L1Size));
    const double MMAccProb = (L3Size < E_DOUBLE) ? config.MissRatioOfMemSize(L2Size+L1Size) :
      config.MissRatioOfMemSize(L3SizeEff+L2Size+L1Size);*/

    // Option 2 (pessimistic): inclusive cache
    /*const double L1AccProb = 1.0 - config.MissRatioOfMemSize(L1Size);
    const double L2AccProb = config.MissRatioOfMemSize(L1Size) - config.MissRatioOfMemSize(L2Size);
    const double L3AccProb = (L3Size < E_DOUBLE) ? 0.0 :
      (config.MissRatioOfMemSize(L2Size) - config.MissRatioOfMemSize(L3SizeEff));
    const double MMAccProb = (L3Size < E_DOUBLE) ? config.MissRatioOfMemSize(L2Size) :
      config.MissRatioOfMemSize(L3SizeEff);*/

    // Option 3 (optimistic): exclusive cache aka AMAT metric
    /*const double L1AccProb = 1.0 - config.MissRatioOfMemSize(L1Size);
    const double L2AccProb = config.MissRatioOfMemSize(L1Size)*(1.0 - config.MissRatioOfMemSize(L2Size));
    const double L3AccProb = (L3Size < E_DOUBLE) ? 0.0 : config.MissRatioOfMemSize(L1Size)*
      config.MissRatioOfMemSize(L2Size)*(1.0 - config.MissRatioOfMemSize(L3SizeEff));
    const double MMAccProb = (L3Size < E_DOUBLE) ? config.MissRatioOfMemSize(L1Size)*
      config.MissRatioOfMemSize(L2Size) : config.MissRatioOfMemSize(L1Size)*
      config.MissRatioOfMemSize(L2Size)*config.MissRatioOfMemSize(L3SizeEff);*/

    // use the same L3 latency function for all caches
    const UInt L1Lat = config.L3LatencyOfSize(L1Size);
    const UInt L2Lat = config.L3LatencyOfSize(L2Size);

    const double L1Eacc = config.L3AccessEnergyOfSize(L1Size);
    const double L1Pleak = config.L3LeakagePowerOfSize(L1Size);
    const double L2Eacc = config.L3AccessEnergyOfSize(L2Size);
    const double L2Pleak = config.L3LeakagePowerOfSize(L2Size);

    (*ac) << "DEFINE Proc" << ptype << " PROC Type=" << ptype
          << " L1Size=" << L1Size << " L1Lat=" << L1Lat /*<< " L1AccProb=" << L1AccProb*/
          << " L2Size=" << L2Size << " L2Lat=" << L2Lat /*<< " L2AccProb=" << L2AccProb*/
          << " L3SizeEff=" << L3SizeEff
          /*<< " L3AccProb=" << L3AccProb << " MMAccProb=" << MMAccProb*/
          << " OoO=" << config.ProcOoO(ptype) << " Area=" << config.ProcArea(ptype)
          /*<< " Ipc=" << config.ProcIpc(ptype) << " Mpi=" << config.ProcMpi(ptype)*/
          << " Freq=" << config.ProcFreq(ptype) << " Volt=" << config.ProcVolt(ptype) << " Epi=" << config.ProcEpi(ptype)
          << " Pleak=" << config.ProcPleak(ptype) << " L1Eacc=" << L1Eacc
          << " L1Pleak=" << L1Pleak << " L2Eacc=" << L2Eacc << " L2Pleak=" << L2Pleak << "\n";
  }
  // --- end defines

  (*ac) << "MESH Col=" << x << " Row=" << y << " ColWidth=6 RowHeight=5 LinkDelay="
        << config.MeshLinkDelayOfClusterArea(clusterArea) << " RouterDelay=3\n";

  for (int i = 0; i < y; ++i) {
    for (int j = 0; j < x; ++j) {
      if (config.ClusterIcType()[ClusterIcTypeIter()] == "bus") {
        (*ac) << "  BUS CompCnt=" << procPerCluster+memPerCluster+DUMP_NI
              << " AccessTime=" << config.BusTimeOfClusterArea(clusterArea) << "\n";
      }
      else if (config.ClusterIcType()[ClusterIcTypeIter()] == "uring") {
        (*ac) << "  URING CompCnt=" << procPerCluster+memPerCluster+DUMP_NI
              << " LinkDelay=0 RouterDelay=1\n";
      }
      else if (config.ClusterIcType()[ClusterIcTypeIter()] == "bring") {
        (*ac) << "  BRING CompCnt=" << procPerCluster+memPerCluster+DUMP_NI
              << " LinkDelay=0 RouterDelay=1\n";
      }
      else if (config.ClusterIcType()[ClusterIcTypeIter()] == "xbar") {
        (*ac) << "  XBAR CompCnt=" << procPerCluster+memPerCluster+DUMP_NI
              << " Delay=" << config.XBarDelayOfClusterArea(clusterArea) << "\n";
      }
      else {
        cout << "-E- Wrong IC type: " << config.ClusterIcType()[ClusterIcTypeIter()] << endl;
        exit(1);
      }

      for (int ptype = 0; ptype < config.ProcCnt(); ++ptype) {
        for (int p = 0; p < ProcIter()[ptype]; ++p) {
          (*ac) << "    Proc" << ptype << endl;
        }
      }

      for (int m = 0; m < memPerCluster; ++m) {
        (*ac) << "    MEM Size=" << L3Size
              << " Eacc=" << config.L3AccessEnergyOfSize(L3Size)
              << " Pleak=" << config.L3LeakagePowerOfSize(L3Size) << "\n";
      }

      if (DUMP_NI) (*ac) << "    NI\n";
      //(*ac) << "    DIR\n";
    }
  }

  (*ac) << "MEMCTRL Location=West Latency=100 Eacc=" << config.MemCtrlAccessEnergy()
        << " Pleak=" << config.MemCtrlLeakagePower() << "\n"
        << "MEMCTRL Location=North Latency=100 Eacc=" << config.MemCtrlAccessEnergy()
        << " Pleak=" << config.MemCtrlLeakagePower() << "\n"
        << "MEMCTRL Location=South Latency=100 Eacc=" << config.MemCtrlAccessEnergy()
        << " Pleak=" << config.MemCtrlLeakagePower() << "\n"
        << "MEMCTRL Location=East Latency=100 Eacc=" << config.MemCtrlAccessEnergy()
        << " Pleak=" << config.MemCtrlLeakagePower() << "\n";


  // 3. fill in string parameters ==============================================================
  ostringstream os;
  os << setw(2) << x; ac->AddParam("x", os.str()); os.str("");
  os << setw(2) << y; ac->AddParam("y", os.str()); os.str("");
  for (int p = 0; p < config.ProcCnt(); ++p) {
    os << setw(2) << ProcIter()[p]; ac->AddParam(config.ProcName(p), os.str()); os.str("");
    os << setw(3) << config.ProcL1Size(p, ProcL1SizeIter()[p]); ac->AddParam("L1", os.str()); os.str("");
    os << setw(5) << config.ProcL2Size(p, ProcL2SizeIter()[p]); ac->AddParam("L2", os.str()); os.str("");
  }
  os << setw(3) << x*y*procPerCluster; ac->AddParam("P#", os.str()); os.str("");
  //os << setw(2) << memPerCluster; ac->AddParam("NumL3", os.str()); os.str("");
  //os << setw(7) << setprecision(3) << L3Size << "Mb"; ac->AddParam("L3Size", os.str()); os.str("");
  os << setw(5) << setprecision(3) << x*y*memPerCluster*L3Size << "MB";
    ac->AddParam("L3", os.str()); os.str("");
  //os << setw(1) << config.BusTimeOfClusterArea(clusterArea);
  //  ac->AddParam("BusTime", os.str()); os.str("");

  if (config.ClusterIcType()[ClusterIcTypeIter()] == "bus") {
    os << setw(1) << config.BusTimeOfClusterArea(clusterArea);
    ac->AddParam("Ic", "Bus-"+os.str()); os.str("");
  }
  else if (config.ClusterIcType()[ClusterIcTypeIter()] == "uring") {
    ac->AddParam("Ic", "uRing");
  }
  else if (config.ClusterIcType()[ClusterIcTypeIter()] == "bring") {
    ac->AddParam("Ic", "bRing");
  }
  else if (config.ClusterIcType()[ClusterIcTypeIter()] == "xbar") {
    os << setw(1) << config.XBarDelayOfClusterArea(clusterArea);
    ac->AddParam("Ic", "xBar-"+os.str()); os.str("");
  }

  //os << setw(5) << procAreaPerCluster/clusterArea; ac->AddParam("PArea", os.str()); os.str("");
  os << setw(5) << setprecision(4) << ac->Area(); ac->AddParam("A", os.str()); os.str("");

  // 4. fill in vector parameters ==============================================================
  ac->GMeshDimX(x);
  ac->GMeshDimY(y);
  ac->ClusterIcType(config.ClusterIcType()[ClusterIcTypeIter()]);
  for (int p = 0; p < config.ProcCnt(); ++p) {
    ac->ProcCnt().push_back(ProcIter()[p]);
    ac->ProcL1Size().push_back(config.ProcL1Size(p, ProcL1SizeIter()[p]));
    ac->ProcL2Size().push_back(config.ProcL2Size(p, ProcL2SizeIter()[p]));
  }
  ac->NumL3(memPerCluster);

  return ac;
}

//=======================================================================
/*
 * Return true if all parameter iterators are in the valid state.
 */

bool ArchPlanner::AllItersValid() const
{
  for (PCIter it = ParamIters().begin(); it != ParamIters().end(); ++it) {
    if (!(*it)->Valid())
      return false;
  }

  return true;
}

//=======================================================================

  } // namespace arch

} //namespace cmpex
