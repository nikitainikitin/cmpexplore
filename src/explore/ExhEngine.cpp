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
#include <fstream>
#include <iomanip>
#include <unistd.h>

#include "ExhEngine.hpp"
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

ExhEngine::ExhEngine() {}

ExhEngine::~ExhEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void ExhEngine::Explore() const
{
  ArchPlanner ap;
  ArchConfig * pAc;

  int cnt = 0;

  while (pAc = ap.GenerateNextArchConfig()) {
    /*if (config.DumpConfigs()) {
      ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
      out << pAc->ToString();
      out.close();
    }*/

    pAc->AddParam("#conf\t", IntToStr(cnt+1) + "\t");

    cmpConfig.CreateCmp(*pAc);

    // *************** HACK: avoid configs with no L3 ********************
    if (config.SimulateCC() && !cmpConfig.MemCnt()) {
      //cout << "Skipping 0-memory" << endl;
      cmpConfig.Cleanup();
      continue;
    }
    // *************** HACK: avoid configs with no L3 ********************

    //ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
    //out << pAc->ToString();
    //out.close();

    StatConfig * sc = new StatConfig(pAc);

    // for every workload
    for (int wlIdx = 0; wlIdx < cmpConfig.GetWlCnt(); ++wlIdx) {
      cmpConfig.SetWlIdx(wlIdx);

      // Performance
      IterativePerfModel m;
      StatMetrics * pSm = m.Run();
      //StatMetrics * pSm = m.RunSubgradientFp();

      // Power
      double power = PowerModel::GetTotalPower(cmpConfig.Cmp());
      pSm->Power(power);

      sc->AddMetrics(pSm);
    }

    if (sc->AggPower() <= config.MaxPower()) {
      stats.AddConfig(sc);
      //ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
      //out << pAc->ToString();
      //out.close();
      ++cnt;
    }
    else {
      delete sc;
    }

    cmpConfig.Cleanup();

    if (isatty(fileno(stdout))) {
      cout << '\r' << "Configs explored: " << cnt; flush(cout);
    }
  }
  cout << '\r';

  stats.ReportAllConfigs();
  if (config.DumpConfigs()) stats.DumpConfigs();

  cout << cnt << " configurations explored" << endl;
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
