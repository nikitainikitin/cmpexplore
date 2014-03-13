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

#include "ExplConf.hpp"
#include "arch/ArchPlanner.hpp"
#include "Config.hpp"

using namespace std;

namespace cmpex {

  extern Config config;

  using namespace arch;

  namespace explore {

//=======================================================================
/*
 * Constructors and destructor
 */

ExplConf::ExplConf (ArchPlanner& ap, double a, double t, double p, double c):
  xIdx(ap.GMeshDimXIter()), yIdx(ap.GMeshDimYIter()),
  procCnt0(ap.ProcIter()[0]), l1_0_idx(ap.ProcL1SizeIter()[0]),
  l2_0_idx(ap.ProcL2SizeIter()[0]), procCnt1(ap.ProcIter()[1]),
  l1_1_idx(ap.ProcL1SizeIter()[1]), l2_1_idx(ap.ProcL2SizeIter()[1]),
  procCnt2(ap.ProcIter()[2]), l1_2_idx(ap.ProcL1SizeIter()[2]),
  l2_2_idx(ap.ProcL2SizeIter()[2]), clType(ap.ClusterIcTypeIter()),
  area (a), thr (t), power(p), cost (c) {}

ExplConf::~ExplConf () {}

//=======================================================================
/*
 * Set ArchPlanner from current config
 */

void ExplConf::SetAp (ArchPlanner& ap) const {
  ap.GMeshDimXIter(xIdx);
  ap.GMeshDimYIter(yIdx);
  ap.ProcIter()[0] = procCnt0;
  ap.ProcL1SizeIter()[0] = l1_0_idx;
  ap.ProcL2SizeIter()[0] = l2_0_idx;
  ap.ProcIter()[1] = procCnt1;
  ap.ProcL1SizeIter()[1] = l1_1_idx;
  ap.ProcL2SizeIter()[1] = l2_1_idx;
  ap.ProcIter()[2] = procCnt2;
  ap.ProcL1SizeIter()[2] = l1_2_idx;
  ap.ProcL2SizeIter()[2] = l2_2_idx;
  ap.ClusterIcTypeIter(clType);
}

//=======================================================================
/*
 * Print configuration
 */

void ExplConf::Print () const {
  cout << "x=" << config.GMeshDimXVec()[xIdx] << "; y=" << config.GMeshDimYVec()[yIdx]
       //<< "; " << ( clType == 0 ? "bus" : (clType == 1 ? "uring" : (clType == 2 ? "bring" : "xbar" )))
       << "; " << config.ClusterIcType()[clType]
       << "; P1=" << procCnt0 << "; L1=" << config.ProcL1Size(0, l1_0_idx)
       << "; L2=" << config.ProcL2Size(0, l2_0_idx);
  if (config.ProcCnt() >= 2)
       cout << "; P2=" << procCnt1
       << "; L1=" << config.ProcL1Size(1, l1_1_idx) << "; L2=" << config.ProcL2Size(1, l2_1_idx);
  if (config.ProcCnt() == 3)
       cout << "; P3=" << procCnt2 << "; L1=" << config.ProcL1Size(2, l1_2_idx)
            << "; L2=" << config.ProcL2Size(2, l2_2_idx);

  cout << ";   A=" << area << "; Pow=" << power << "; Thr=" << thr << endl;
}

//=======================================================================

  } //namespace explore

} //namespace cmpex
