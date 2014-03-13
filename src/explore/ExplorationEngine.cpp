// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File ExplorationEngine.cpp
//   Created Sep 09, 2011
// ----------------------------------------------------------------------

#include <iostream>
#include <string>
#include <limits>
#include <fstream>
#include <iomanip>
#include <unistd.h>

#include "ExplorationEngine.hpp"
#include "Config.hpp"
#include "arch/ArchConfig.hpp"
#include "cmp/CmpConfig.hpp"
#include <Timer.hpp>
#include "arch/ArchPlanner.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "phys/PhysicalModel.hpp"
#include "power/PowerModel.hpp"

using namespace std;

namespace cmpex {
  
  extern Config config;
  extern cmp::CmpConfig cmpConfig;
  extern stat::Statistics stats;

  using namespace cmp;
  using namespace arch;
  using namespace stat;
  using namespace perf;
  using namespace phys;
  using namespace power;

  namespace explore {

//=======================================================================
/*
 * Constructors and destructor
 */

ExplorationEngine::ExplorationEngine() {}

ExplorationEngine::~ExplorationEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void ExplorationEngine::Explore() const
{
  ArchPlanner ap;
  ArchConfig * pAc;

  int cnt = 0, unRout = 0;

  while (pAc = ap.GenerateNextArchConfig()) {
    if (config.DumpConfigs()) {
      ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
      out << pAc->ToString();
      out.close();
    }

    pAc->AddParam("#conf\t", IntToStr(cnt+1) + "\t");

    cmpConfig.CreateCmp(*pAc);

    // Performance
    IterativePerfModel m;
    StatMetrics * pSm = m.RunBisectionFp();
    //StatMetrics * pSm = m.RunSubgradientFp();

    // Power
    double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
    pSm->Power(power);

    // Routability
    double rout = PhysicalModel::GetRoutabilitySlack(pAc)/config.MaxArea()*100.0;
    pSm->Routability(rout);
    if (rout < 0.0) {
      //cout << "Unroutable config" << endl;
      ++unRout;
    }

    if (power <= config.MaxPower()) {
      StatConfig * sc = new StatConfig(pAc, pSm);
      stats.AddConfig(sc);
    }
    else {
      delete pAc;
      delete pSm;
    }

    cmpConfig.Cleanup();

    ++cnt;
    if (isatty(fileno(stdout))) {
      cout << '\r' << "Configs explored: " << cnt; flush(cout);
    }
  }
  cout << '\r';

  stats.ReportAllConfigs();
  //stats.ReportBestThroughput(50);
  //stats.ReportBestLatency(10000);
  //stats.ReportWorstRoutability(1000);
  //stats.DumpAllConfigs();

  cout << cnt << " configurations explored" << endl;
  cout << unRout << " unroutable configurations found" << endl;
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
