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
#include <vector>
#include <map>
#include <ctime>

#include "SaEngine.hpp"
#include "Config.hpp"
#include "arch/ArchConfig.hpp"
#include "cmp/CmpConfig.hpp"
#include <Timer.hpp>
#include "arch/ArchPlanner.hpp"
#include "Util.hpp"
#include "StatConfig.hpp"
#include "Statistics.hpp"
#include "perf/IterativePerfModel.hpp"
#include "PhysicalModel.hpp"
#include "RouterDefs.hpp"
#include "power/PowerModel.hpp"
#include "ExplConf.hpp"
#include "Transform.hpp"

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

SaEngine::SaEngine() {}

SaEngine::~SaEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void SaEngine::Explore()
{
  const bool GRAPH_MODE_PRINT = false;

  stats.NoDuplicates(true);

  ArchPlanner ap;
  ExplConf *curConf, *bestConf;

  Timer timer;
  timer.Start();

  double lambda = 0.5;

  // 1. Initialization
  curConf = bestConf = GetCurConfigCost(ap, lambda);
  cout << "Initial Thr = " << bestConf->thr << endl;

  // 2. Main SA loop

  double tInit = 100;
  double tCur = tInit;
  double alpha = config.SaAlpha();

  int last_impr_iter = 0;

  // number of local iterations
  double lIterCnt = config.GMeshDimXVec().size() + config.GMeshDimYVec().size();
  for (int ptype = 0; ptype < config.ProcCnt(); ++ptype) {
    lIterCnt += config.ProcL1SizeCnt(ptype);
    lIterCnt += config.ProcL2SizeCnt(ptype);
  }

  cout << "Number of local iters = " << lIterCnt << endl;

  double no_impr_limit = lIterCnt*lIterCnt*config.SEffort();
  double last_impr = 0;
  int oIter = 0;

  do { // outer cooling loop

    for (int iter = 0; iter < lIterCnt; ++iter) { // inner cooling loop
      // 2a. Generate new solution (apply transform)
      int trCnt;
      switch(config.ProcCnt()) {
      case 1: trCnt = 22; break;
      case 2: trCnt = 38; break;
      case 3: trCnt = 56; break;
      default: cout << "-E-: SA only supports 1, 2 or 3 types of cores" << endl;
        exit(1);
      }

      int idx = int(RandUDouble()*trCnt);
      Transforms()[idx]->UpdateAP(*curConf, ap);

      // 2b. Estimate cost of new solution
      ExplConf * c = GetCurConfigCost(ap, lambda);

      // 2c. Decide acceptance
      lambda = 0.5*tInit/tCur;
      double curC = curConf->cost;
      double newC = c->cost;

      if (newC > curC) { // accept
        if (curConf != bestConf) delete curConf;
        curConf = c;
      }
      else if ( RandUDouble() < // accept
              std::exp(- curC/newC * curC/newC * tInit/tCur) ) {
        if (curConf != bestConf) delete curConf;
        curConf = c;
      }
      else { // reject
        delete c;
      }

      double x = config.GMeshDimXVec()[curConf->xIdx];
      double y = config.GMeshDimYVec()[curConf->yIdx];
      // 2d. update best cost
      if (curConf->thr > bestConf->thr && fabs(curConf->cost-curConf->thr) < E_DOUBLE && x >= y) {
        last_impr = oIter*lIterCnt + iter;
        delete bestConf;
        bestConf = curConf;
        if (GRAPH_MODE_PRINT) {
          cout << timer.Current() << '\t' << bestConf->thr << '\t' << bestConf->power << endl;
        }
        else {
          cout << "(Time " << timer.Current() << ") "; curConf->Print();
        }
      }

    }

    tCur = tCur*alpha;
    ++oIter;
  } while (oIter*lIterCnt - last_impr < no_impr_limit);

  cout << "Finished search (no improvement during the last "
       << no_impr_limit << " iterations)" << endl;

  stats.ReportAllConfigs();
  if (config.DumpConfigs()) stats.DumpAllConfigs();

  cout << "Best Thr = " << bestConf->thr << endl;
  cout << "Params: tCur = " << tCur << endl;
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
