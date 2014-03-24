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
#include <ctime>
#include <vector>

#include "Config.hpp"
#include "Debug.hpp"
#include "cmp/CmpConfig.hpp"
#include "cmp/Component.hpp"
#include "explore/ExhEngine.hpp"
#include "explore/EoEngine.hpp"
#include "explore/SaEngine.hpp"
#include "explore/HcEngine.hpp"
#include "explore/RbEngine.hpp"
#include "stat/Statistics.hpp"
#include "stat/StatConfig.hpp"
#include "model/Function.hpp"
#include <Timer.hpp>
#include "perf/IterativePerfModel.hpp"
#include "perf/MatlabPerfModel.hpp"
#include "power/PowerModel.hpp"
#include "phys/PhysicalModel.hpp"
#include "ptsim/PTsim.hpp"
#include "workload/WlConfig.hpp"

using namespace cmpex;
using namespace cmpex::cmp;
using namespace cmpex::explore;
using namespace cmpex::stat;
using namespace cmpex::perf;
using namespace cmpex::power;
using namespace cmpex::temperature;
using namespace cmpex::phys;
using std::cout;
using std::endl;
using std::vector;
using cmpex::Timer;

// global configuration manager
namespace cmpex {
  Config config;
  Debug debug;
  cmp::CmpConfig cmpConfig;
  workload::WlConfig wlConfig;
  stat::Statistics stats(100);
}

//=======================================================================
/*
 * Main procedure
 */

int main( int argc, char ** argv )
{
  srand(0);

  // parse command line
  if (config.ParseCommandLine(argc, argv)) {
    exit(1);
  }

  debug.Level(config.Debug());

  Timer timer;
  timer.Start();

  if (config.Test() != "") {
    if (config.A2wa()) {  // run conversion mode
      cmpConfig.WriteWorkloadArchFromArch(config.Test());
      return 0;
    }
    else if (config.Tmap()) { // mapping mode
      // DEBUG
      wlConfig.CreateTasks(5);
      int tot =  wlConfig.PrintTasks(5);
      cout << endl << "Total Tasks = " << tot << endl;
      wlConfig.Cleanup();
      // end DEBUG
    }
    else {                // run selected testcase
      cmpConfig.CreateCmp(config.Test());

      double area = PhysicalModel::GetCmpArea(cmpConfig.Cmp());
      cout << "Area = " << area << " mm^2" << endl;

      double avgThr = 0, avgLat = 0, avgPow = 0;

      // for every workload
      for (int wlIdx = 0; wlIdx < cmpConfig.GetWlCnt(); ++wlIdx) {
        cmpConfig.SetWlIdx(wlIdx);
        IterativePerfModel m;
        //MatlabPerfModel m;
        StatMetrics * sm = m.Run();
        double power = PowerModel::GetTotalPower(cmpConfig.Cmp());

        // calling PTsim
        if (config.CallPTsim()) PTsim::CallHotSpot(cmpConfig.Cmp());

        avgThr += sm->Throughput();
        avgLat += sm->Latency();
        avgPow += power;

        cout << "WlName = " << cmpConfig.GetWorkload(wlIdx)->shortName
             << ": Power = " << power
             << ", Lat = " << sm->Latency() << ", Thr = " << sm->Throughput() << endl;
        delete sm;
      }

      avgPow /= cmpConfig.GetWlCnt();
      avgLat /= cmpConfig.GetWlCnt();
      avgThr /= cmpConfig.GetWlCnt();

      cout << "Average: Power = " << avgPow
           << ", Lat = " << avgLat << ", Thr = " << avgThr << endl;

    }
  }
  else { // run exploration based on config
    string configFile = (config.ConfigFile() == "") ? "config.txt" :
                         config.ConfigFile();
    cout << "-I- Config file for exploration is " << configFile << endl;
    config.ParseConfigFile(configFile);
    config.Print();

    if (config.ExpMode() == "ex") { // exhaustive exploration
      cout << "-I- Exploration mode is: Exhaustive" << endl;
      ExhEngine ee;
      ee.Explore();
    }
    else if (config.ExpMode() == "eo") {
      cout << "-I- Exploration mode is: Extremal Optimization" << endl;
      EoEngine ee;
      ee.Explore();
    }
    else if (config.ExpMode() == "sa") {
      cout << "-I- Exploration mode is: Simulated Annealing" << endl;
      SaEngine ee;
      ee.Explore();
    }
    else if (config.ExpMode() == "hc") {
      cout << "-I- Exploration mode is: Hill Climbing" << endl;
      HcEngine ee;
      ee.Explore();
    }
    else if (config.ExpMode() == "rb") {
      cout << "-I- Exploration mode is: Random Best" << endl;
      RbEngine ee;
      ee.Explore();
    }
    else {
      cout << "-E- Unknown exploration mode! -> Exiting..." << endl;
    }
  }

  cout << "Total time = " << timer.Current() << endl;

  return 0;
}

//=======================================================================
