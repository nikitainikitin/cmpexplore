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
#include <map>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <cstdlib>

#include "Statistics.hpp"
#include "StatConfig.hpp"
#include "ArchConfig.hpp"
#include "Util.hpp"

using namespace std;

namespace cmpex {

using arch::ArchConfig;

  namespace stat {

//=======================================================================
/*
 * Constructors and destructor
 */

Statistics::Statistics(int maxConfig) : maxConfig_(maxConfig), noDuplicates_ (false) {}

Statistics::~Statistics() {
  for (CCIter it = Configs().begin(); it != Configs().end(); ++it)
    delete it->second;
}

//=======================================================================
/*
 * Report all available configs in sequence.
 */

void Statistics::ReportAllConfigs() const {
  cout << "*************** Stats: ReportAllConfigs ***************" << endl;
  for (CCIter it = Configs().begin(); it != Configs().end(); ++it)
    ReportConfig((*it).second);
}

//=======================================================================
/*
 * Dump all available configs in sequence.
 */

void Statistics::DumpAllConfigs() const {
  int cnt = 0;
  for (CCIter it = Configs().begin(); it != Configs().end(); ++it, ++cnt) {
	ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
    out << (*it).second->Config()->ToString();
    out.close();
  }
}

//=======================================================================
/*
 * Adds config to statistics, assuring not more than maxConfig best
 * configurations.
 */

void Statistics::AddConfig(StatConfig *sc) {
  CCIter lb = configs_.lower_bound(sc->AggThroughput()-E_DOUBLE);
  CCIter ub = configs_.upper_bound(sc->AggThroughput()+E_DOUBLE);

  if (!NoDuplicates() || (lb == ub)) {
    configs_.insert(make_pair(sc->AggThroughput(), sc));
  }
  
  if (configs_.size() > MaxConfig()) { // delete worst config
    delete (*(--configs_.end())).second;
    configs_.erase(--configs_.end());
  }
  assert(configs_.size() <= MaxConfig());
}

//=======================================================================
/*
 * Report cnt configs with worst routability.
 */

void Statistics::ReportWorstRoutability(UInt cnt) const {
  cout << "************* Stats: ReportWorstRoutability *************" << endl;
  typedef multimap<double, StatConfig*, less<double> > RoutMap;
  RoutMap routMap;
  for (CCIter it = Configs().begin(); it != Configs().end(); ++it)
    routMap.insert(make_pair(it->second->GetMetrics(0)->Routability(),it->second));

  for (RoutMap::const_iterator it = routMap.begin();
       it != routMap.end(); ++it, --cnt) {
    ReportConfig(it->second);
    if (cnt <= 1)
      break;
  }
}

//=======================================================================
/*
 * Report selected config.
 */

void Statistics::ReportConfig(StatConfig * sc) const {
  for (ArchConfig::PCIter it = sc->Config()->Params().begin();
       it != sc->Config()->Params().end(); ++it)
    cout << it->first << '=' << it->second << "; ";

  //cout << "\tLat =" << setw(4) << setprecision(3) << sc->AggLatency() << ' ';
  cout << "Pwr = " << setw(5) << setprecision(4) << sc->AggPower() << ' ';
  cout << "Thr =\t" << setw(6) << setprecision(5) << sc->AggThroughput() << '\t';
  //cout << "Rtb =" << setw(5) << setprecision(3) << sc->GetMetrics(0)->Routability() << '%';
  cout << endl;
}

//=======================================================================
/*
 * Dump all configs in sequence.
 */

void Statistics::DumpConfigs() const {
  if (system("mkdir -p test")) {
    cout << "-E- Could not create directory ./test - no configs will be dumped." << endl;
  }

  int cnt = 0;
  for (CCIter it = Configs().begin(); it != Configs().end(); ++it, ++cnt) {
    ofstream out(string("test/cmp" + IntToStr(cnt+1) + ".cmp").c_str());
    out << it->second->Config()->ToString();
    out.close();
  }
}

//=======================================================================

  } // namespace stat

} //namespace cmpex
