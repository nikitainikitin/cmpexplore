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

#include "EoEngine.hpp"
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

EoEngine::EoEngine() {}

EoEngine::~EoEngine() {}

//=======================================================================
/*
 * Main method that invokes the exploration.
 */

void EoEngine::Explore()
{
  const bool GRAPH_MODE_PRINT = false;

  stats.NoDuplicates(true);

  const double tau = config.EoTau();

  double lambda = 0.5;

  ArchPlanner ap;
  ExplConf *curConf, *bestConf;

  Timer timer;
  timer.Start();

  // 1. Initialization
  curConf = bestConf = GetCurConfigCost(ap, lambda);
  cout << "Initial Thr = " << bestConf->thr << endl;

  // 2. Main EO loop
  multimap<double, ExplConf*, greater<double> > newConfs;

  // number of local iterations
  double lIterCnt = config.GMeshDimXVec().size() + config.GMeshDimYVec().size();
  for (int ptype = 0; ptype < config.ProcCnt(); ++ptype) {
    lIterCnt += config.ProcL1SizeCnt(ptype);
    lIterCnt += config.ProcL2SizeCnt(ptype);
  }

  cout << "Number of local iters = " << lIterCnt << endl;

  double no_impr_limit = lIterCnt*lIterCnt*config.SEffort();
  double last_impr = 0;
  int iter = 0;

  int trCnt;
  switch(config.ProcCnt()) {
  case 1: trCnt = 22; break;
  case 2: trCnt = 38; break;
  case 3: trCnt = 56; break;
  default: cout << "-E-: EO only supports 1, 2 or 3 types of cores" << endl;
    exit(1);
  }

  do {
    lambda *= pow(1.01, config.EoTau()); // or try 1.02
    //cout << "new lambda = " << lambda << endl;

    // 3. at the beginning of every iter do local search
    for (int liter = 0; liter < lIterCnt; ++liter) {
      int idx = int(RandUDouble()*trCnt);
      Transforms()[idx]->UpdateAP(*curConf, ap);

      ExplConf * c = GetCurConfigCost(ap, lambda);
      if (c->cost > curConf->cost) {
        if (curConf != bestConf) delete curConf;
        curConf = c;
      }
      else {
        delete c;
      }

      double x = config.GMeshDimXVec()[curConf->xIdx];
      double y = config.GMeshDimYVec()[curConf->yIdx];
      if (curConf->thr > bestConf->thr && fabs(curConf->cost-curConf->thr) < E_DOUBLE && x >= y) {
        last_impr = iter*lIterCnt;
        delete bestConf;
        bestConf = curConf;

        if (GRAPH_MODE_PRINT) {
          cout << timer.Current() << '\t' << bestConf->thr << '\t' << bestConf->power << endl;
        }
        else {
          cout <<"(Time " << timer.Current() << ") "; curConf->Print();
        }
      }
    }

    // 4. Generate neighborhood
    for (int idx = 0; idx < trCnt; ++idx) {
      Transforms()[idx]->UpdateAP(*curConf, ap);
      ExplConf * c = GetCurConfigCost(ap, lambda);
      newConfs.insert(make_pair(c->cost, c));
    }

    // 5. accept new config
    int idx = int(floor(newConfs.size()*pow(RandUDouble(),tau)));
    //cout << "Idx = " << idx << endl;
    map<double, ExplConf*, greater<double> >::iterator it = newConfs.begin();
    for (int i = 0; i < newConfs.size(); ++i, ++it) {
      if (i == idx) {
        if (curConf != bestConf) delete curConf;
        curConf = it->second;
        double x = config.GMeshDimXVec()[curConf->xIdx];
        double y = config.GMeshDimYVec()[curConf->yIdx];
        if (curConf->thr > bestConf->thr && fabs(curConf->cost-curConf->thr) < E_DOUBLE && x >= y) {
          last_impr = iter*lIterCnt;
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
      else { // check if any of the neighbors is better than best config (update the latter)
        ExplConf * c = it->second;
        double x = config.GMeshDimXVec()[c->xIdx];
        double y = config.GMeshDimYVec()[c->yIdx];
        if (c->thr > bestConf->thr && fabs(c->cost-c->thr) < E_DOUBLE && x >= y) {
          last_impr = iter*lIterCnt;
          if (bestConf != curConf) delete bestConf;
          bestConf = c;

          if (GRAPH_MODE_PRINT) {
            cout << timer.Current() << '\t' << bestConf->thr << '\t' << bestConf->power << endl;
          }
          else {
            cout << "(Time " << timer.Current() << ") "; bestConf->Print();
          }
        }
        else
          delete it->second;
      }
    }
    newConfs.clear();
    ++iter;
  } while (iter*lIterCnt - last_impr < no_impr_limit);

  cout << "Finished search (no improvement during the last "
       << no_impr_limit << " iterations)" << endl;
  
  stats.ReportAllConfigs();
  if (config.DumpConfigs()) stats.DumpAllConfigs();
}

//=======================================================================

  } // namespace explore

} //namespace cmpex
