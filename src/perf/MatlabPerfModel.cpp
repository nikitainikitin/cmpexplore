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
#include <cmath>
#include <fstream>
#include <sstream>

#include "MatlabPerfModel.hpp"
#include "Component.hpp"
#include "CmpConfig.hpp"
#include "Cluster.hpp"
#include "Processor.hpp"
#include "Memory.hpp"
#include "Interconnect.hpp"
#include <Timer.hpp>
#include "Debug.hpp"
#include "StatConfig.hpp"
#include "MeshIc.hpp"
#include "RouterModel.hpp"
#include "BusIc.hpp"
#include "BusModel.hpp"

using std::cout;
using std::endl;
using std::ofstream;
using std::ostringstream;

namespace cmpex {

  extern cmp::CmpConfig cmpConfig;
  extern Debug debug;

  using namespace cmp;
  using stat::StatMetrics;

  using model::RouterModel;
  using model::BusModel;

  namespace perf {

  const double thrEps = 0.02; // precision in throughput
  const double latEps = 0.02; // precision in latency

//=======================================================================
/*
 * Constructors and destructor
 */

MatlabPerfModel::MatlabPerfModel () {}

MatlabPerfModel::~MatlabPerfModel () {}

//=======================================================================
/*
 * Main method to estimate CMP performance.
 */

StatMetrics * MatlabPerfModel::Run ()
{
  return RunMatlab();
}

//=======================================================================
/*
 * Dump Matlab model (system of equations) for CMP performance estimation.
 * IMPORTANT NOTE: after the latest changes the model may appear inconsistent.
 * TODO: recheck all the equations.
 */

StatMetrics * MatlabPerfModel::RunMatlab()
{
  Cluster * clCmp = static_cast<Cluster*>(cmpConfig.Cmp());

  for (int c = 0; c < clCmp->CompCnt(); ++c) {
    if (clCmp->GetComponent(c)->Type() == CTCLUSTER) {
      cout << "OptimizeMatlabFlat: Only flat hierarchies are supported!" << endl;
      DASSERT(false);
    }
  }

  double systemThr = 0.0;
  double systemLat = 0.0;
  int iter = 0;

  // init models
  clCmp->Ic()->InitModel();

  // calculate traffic rates
  vector<double> procLat;
  vector<double> procThr;
  vector<double> procRate;
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    double lat = CalculateProcLatency(p, iter ? true : false);
    DEBUG(2, "P" << p << ": avg lat = " << lat << " cycles" << endl);
    systemLat += lat;
    procLat.push_back(lat);

    double thr = proc->Ipc() / (1.0 + (lat*proc->Ipc()-1.0)*proc->Mpi());
    DEBUG(2, "P" << p << ": thr = " << thr << endl);
    systemThr += thr;
    procThr.push_back(thr);

    double totalTraffic = 1.0*thr*proc->Mpi();
    DEBUG(5, "P" << p << ": total memory traffic = " << totalTraffic << endl);
    procRate.push_back(totalTraffic);
  }

  DEBUG(1, "Iter " << iter << ": system throughput = " << systemThr << endl);
  DEBUG(1, "Iter " << iter << ": avg lat = " << systemLat/cmpConfig.ProcCnt() << endl);

  // mark paths
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    // handle L3 memories
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      double trafficToMem = procRate[p]*proc->L3AccessProbability()*proc->L3ProbDistr()[m];

      // global mesh
      if (clCmp->ProcOwner(p) == clCmp->MemOwner(m))
        continue;

      clCmp->Ic()->MarkPathCompToComp(
          clCmp->ProcOwner(p)->ClIdx(), clCmp->MemOwner(m)->ClIdx(), trafficToMem, UShort(MSGNOCC));
      clCmp->Ic()->MarkPathCompToComp(
          clCmp->MemOwner(m)->ClIdx(), clCmp->ProcOwner(p)->ClIdx(), trafficToMem, UShort(MSGNOCC));
    } // L3 memories

    // handle memory controllers
    double trafficToMC =
      procRate[p]*proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt();

    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      // global mesh
      clCmp->Ic()->MarkPathCompToMemCtrl(
          clCmp->ProcOwner(p)->ClIdx(), mc, trafficToMC, UShort(MSGNOCC));
      clCmp->Ic()->MarkPathMemCtrlToComp(
          mc, clCmp->ProcOwner(p)->ClIdx(), trafficToMC, UShort(MSGNOCC));
    } // memory controllers
  }

  // run models
  if (clCmp->Ic()->EstimateBufferDelays())
    return new StatMetrics(0.0, MAX_DOUBLE);

  const double T = (cmpConfig.MemReplySize()+1.0)/2.0;
  MeshIc * mic = static_cast<MeshIc*>(clCmp->Ic());

  // dump matlab file =========================================================

  ofstream out("./solveeqs.m");
  out << "function solveeqs()\n";

  // ============================= DUMP GUESS =======================================
  out << "guess=[";
  // guess for l_p
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    out << procRate[c] << ' ';
  }
  //cout << "Number of l_c vars = " << cmpConfig.ProcCnt() << endl;

  // guess for L_c
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    out << procLat[c] << ' ';
  }
  //cout << "Number of L_c vars = " << cmpConfig.ProcCnt() << endl;

  vector<double> traffics, fprobs, conts;

  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    double * traffic = &(mic->Traffic(0,r,RDNORTH,RDNORTH));

    double * mForwProb = new double[25]; // matrix of forwarding probabilities
    double * mCont     = new double[25]; // contention matrix
    double * mTraffic  = new double[5];  // matrix of input traffic rates

    // calc traffic matrix
    for (int i = 0; i < 5; ++i) {
      double inTraffic = 0.0;
      double * p = traffic + i*5;
      for (int o = 0; o < 5; ++o, ++p) {
        inTraffic += *p;
      }

      mTraffic[i] = inTraffic;

      if (r < mic->ColNum() && i == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && i == 2) continue;

      traffics.push_back(inTraffic);
    }

    // calc forw prob matrix
    for (int i = 0; i < 5; ++i) {
      double totalRate = mTraffic[i];
      int startIdx = i*5;
      for (int o = 0; o < 5; ++o) {
        if (i == o) {
          DASSERT(totalRate < EMIN_DOUBLE | traffic[startIdx+o]/totalRate < EMIN_DOUBLE);
          mForwProb[startIdx+o] = 0.0;
        }
        else {
          mForwProb[startIdx+o] =
            (totalRate < EMIN_DOUBLE) ? 0.0 : traffic[startIdx+o]/totalRate;
        }

        if (r < mic->ColNum() && i == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && i == 2) continue;
        if (r < mic->ColNum() && o == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && o == 2) continue;

        fprobs.push_back(mForwProb[startIdx+o]);
      }
    }

    // calc contention matrix
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 5; ++j) {
        double prob = 0.0;
        if (i == j) {
          prob = 1.0;
        }
        else {
          for (int o = 0; o < 5; ++o) {
            prob += mForwProb[i*5+o]*mForwProb[j*5+o];
          }
        }
        mCont[i*5+j] = prob;

        if (r < mic->ColNum() && i == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && i == 2) continue;
        if (r < mic->ColNum() && j == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && j == 2) continue;

        conts.push_back(prob);
      }
    }

    delete mTraffic;
    delete mCont;
    delete mForwProb;
  }

  // guess for lm_r_p vars
  //DASSERT(traffics.size() == 16);
  for (int i = 0; i < traffics.size(); ++i) {
    out << traffics[i] << ' ';
  }
  //cout << "Number of lm_r_p vars = " << traffics.size() << endl;

  // guess for wm_r_p vars
  //int wm_cnt = 5*mic->TCnt() - 2*mic->ColNum();
  //for (int i = 0; i < wm_cnt; ++i) out << "1 ";
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    RouterModel rm(5, 5, (cmpConfig.MemReplySize()+1.0)/2.0+mic->RouterDelay()-1.0);
    double * traffic = &(mic->Traffic(0,r,RDNORTH,RDNORTH));
    vector<double> delays;
    delays.assign(5, 0.0);
    rm.CalcBufferDelays(traffic, &(delays[0]));
    for (int i = 0; i < 5; ++i) {
      if (r < mic->ColNum() && i == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && i == 2) continue;

      out << delays[i] << ' ';
    }
  }

  // guess for f_r_pi_po vars
  //DASSERT(fprobs.size() == 64);
  for (int i = 0; i < fprobs.size(); ++i) {
    out << fprobs[i] << ' ';
  }

  // guess for c_r_pi_pi2 vars
  //DASSERT(conts.size() == 64);
  for (int i = 0; i < conts.size(); ++i) {
    out << conts[i] << ' ';
  }

  // guess for lm_r_pi_po vars
  int cnt = 0;
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        out << mic->Traffic(0,r,RouteDir(pi),RouteDir(po)) << ' ';
        ++cnt;
      }
    }
  }
  //DASSERT(cnt == 64);

  // guess for l_p_m vars
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    // communication proc <-> mem
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      out << proc->L3AccessProbability()*proc->L3ProbDistr()[m]*procRate[p] << ' ';
    } // L3 memories

    // communication proc <-> mc
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      out << proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt()*procRate[p] << ' ';
    } // memory controllers
  }

  out << "];\n";

  out << "[result, fval, exit, output]=fsolve(@eqns,guess,"
      << "optimset('Algorithm', 'Levenberg-Marquardt', 'TolFun', 0.02, 'TolX', 0.02));\n";
  out << "result\nfval\neqns(guess)\noutput\nend\n\n";
  out << "function fcns=eqns(z)\n";

  // ============================= DUMP VARIABLES ===================================
  int varcnt = 1;
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    out << "l_" << c << "=z(" << varcnt << ");\n";
    ++varcnt;
  }
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    out << "L_" << c << "=z(" << varcnt << ");\n";
    ++varcnt;
  }
  for (int router = 0; router < clCmp->CompCnt(); ++router) {
    for (int port = 0; port < 5; ++port) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (router < mic->ColNum() && port == 0) continue;
      if (router >= mic->TCnt() - mic->ColNum() && port == 2) continue;
      out << "lm_" << router << "_" << port << "=z(" << varcnt << ");\n";
      ++varcnt;
    }
  }
  for (int router = 0; router < clCmp->CompCnt(); ++router) {
    for (int port = 0; port < 5; ++port) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (router < mic->ColNum() && port == 0) continue;
      if (router >= mic->TCnt() - mic->ColNum() && port == 2) continue;
      out << "wm_" << router << "_" << port << "=z(" << varcnt << ");\n";
      ++varcnt;
    }
  }
  // dump vars for \f_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        out << "f_" << r << "_" << pi << "_" << po << "=z(" << varcnt << ");\n";
        ++varcnt;
      }
    }
  }
  // dump vars for \c_r_pi_pi2
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int pi2 = 0; pi2 < 5; ++pi2) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && pi2 == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && pi2 == 2) continue;

        out << "c_" << r << "_" << pi << "_" << pi2 << "=z(" << varcnt << ");\n";
        ++varcnt;
      }
    }
  }
  // dump vars for \lm_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        out << "lm_" << r << "_" << pi << "_" << po << "=z(" << varcnt << ");\n";
        ++varcnt;
      }
    }
  }

  // dump vars for \ln_p_m
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    // communication proc <-> mem
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      out << "ln_p" << p << "_m" << m << "=z(" << varcnt << ");\n";
      ++varcnt;
    } // L3 memories

    // communication proc <-> mc
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      out << "ln_p" << p << "_mc" << mc << "=z(" << varcnt << ");\n";
      ++varcnt;
    } // memory controllers
  }

  // ============================= DUMP EQUATIONS ===================================
  int eqcnt = 1;
  // dump equations for \lambda_c
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    Processor * proc = cmpConfig.GetProcessor(c);
    out << "fcns(" << eqcnt << ")=";
    out << "l_" << c << "+" << proc->Mpi() << "*l_" << c << "*L_" << c
        << "-" << proc->Mpi() << "*l_" << c << "-" << proc->Mpi() << ";\n";
    ++eqcnt;
  }

  // dump equations for \lambda_m_r_p
  for (int router = 0; router < clCmp->CompCnt(); ++router) {
    for (int port = 0; port < 5; ++port) {
      ostringstream os;
      for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
        Processor * proc = cmpConfig.GetProcessor(p);

        // communication proc <-> mem
        for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
          if (clCmp->Ic()->PortOnPathCompToComp(
              UShort(clCmp->ProcOwner(p)->ClIdx()), UShort(clCmp->MemOwner(m)->ClIdx()),
              UShort(router), UShort(port))) {
            os << "+" << proc->L3AccessProbability()*proc->L3ProbDistr()[m] << "*" << "l_" << p;
          }
          else if (clCmp->Ic()->PortOnPathCompToComp(
              UShort(clCmp->MemOwner(m)->ClIdx()), UShort(clCmp->ProcOwner(p)->ClIdx()),
              UShort(router), UShort(port))) {
            os << "+" << proc->L3AccessProbability()*proc->L3ProbDistr()[m] << "*" << "l_" << p; // traffic is the same
          }
        } // L3 memories

        // communication proc <-> mc
        for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
          if (clCmp->Ic()->PortOnPathCompToMemCtrl(
              UShort(clCmp->ProcOwner(p)->ClIdx()), UShort(mc),
              UShort(router), UShort(port))) {
            os << "+" << proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt() << "*" << "l_" << p;
          }
          else if (clCmp->Ic()->PortOnPathMemCtrlToComp(
              UShort(mc), UShort(clCmp->ProcOwner(p)->ClIdx()),
              UShort(router), UShort(port))) {
            os << "+" << proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt() << "*" << "l_" << p; // traffic is the same
          }
        } // memory controllers
      }
      if (!os.str().empty()) {
        out << "fcns(" << eqcnt << ")=";
        out << "-lm_" << router << "_" << port;
        out << os.str();
        out << ";\n";
        ++eqcnt;
      }
    }
  }

  // dump equations for L_c
  for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    Processor * proc = cmpConfig.GetProcessor(c);
    ostringstream os;

    os << "+" << proc->L1AccessProbability() << "*" << proc->L1Lat()
       << "+" << proc->L2AccessProbability() << "*" << proc->L2Lat();

    // communication proc <-> mem
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      clCmp->Ic()->DumpLatencyEqCompToComp(UShort(clCmp->ProcOwner(c)->ClIdx()),
        UShort(clCmp->MemOwner(m)->ClIdx()), proc->L3AccessProbability()*proc->L3ProbDistr()[m], os);
      clCmp->Ic()->DumpLatencyEqCompToComp(UShort(clCmp->MemOwner(m)->ClIdx()),
        UShort(clCmp->ProcOwner(c)->ClIdx()), proc->L3AccessProbability()*proc->L3ProbDistr()[m], os);
      os << "+" << (proc->L3AccessProbability()*proc->L3ProbDistr()[m]) << "*" <<
            (cmpConfig.L3Latency() + cmpConfig.MemReplySize()-1.0);
    } // L3 memories

    // communication proc <-> mc
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      clCmp->Ic()->DumpLatencyEqCompToMemCtrl(UShort(clCmp->ProcOwner(c)->ClIdx()),
        UShort(mc), proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt(), os);
      clCmp->Ic()->DumpLatencyEqMemCtrlToComp(UShort(mc),
        UShort(clCmp->ProcOwner(c)->ClIdx()), proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt(), os);
      os << "+" << (proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt()) << "*" <<
            (cmpConfig.GetMemCtrl(mc)->latency + cmpConfig.MemReplySize()-1.0);
    } // memory controllers

    out << "fcns(" << eqcnt << ")=";
    out << "-L_" << c;
    out << os.str();
    out << ";\n";
    ++eqcnt;
  }

  // dump equations for \wm_r_p
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    RouterModel rm(5, 5, (cmpConfig.MemReplySize()+1.0)/2.0+mic->RouterDelay()-1.0);
    vector<double> conts;
    conts.assign(25, 0.0);
    rm.CalcContMatrix(&(mic->Traffic(0,r,RDNORTH,RDNORTH)), &conts[0]);

    for (int port = 0; port < 5; ++port) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && port == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && port == 2) continue;

      out << "fcns(" << eqcnt << ")=";
      out << "-wm_" << r << "_" << port;
      out << "+" << T << "*lm_" << r << "_" << port << "*wm_" << r << "_" << port;

      for (int port2 = 0; port2 < 5; ++port2) {
        if (port == port2) continue;
        if (r < mic->ColNum() && port2 == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && port2 == 2) continue;

        out << "+" << T << "*c_" << r << "_" << port << "_" << port2
            << "*lm_" << r << "_" << port2 << "*wm_" << r << "_" << port2;
      }

      out << "+" << 0.5*T*T << "*lm_" << r << "_" << port;
      out << ";\n";
      ++eqcnt;
    }
  }

  // dump equations for \f_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        if (pi == po) {
          out << "fcns(" << eqcnt << ")=" << "f_" << r << "_" << pi << "_" << po << ";\n";
        }
        else {
          out << "fcns(" << eqcnt << ")=";
          out << "lm_" << r << "_" << pi << "_" << po
              << "-f_" << r << "_" << pi << "_" << po
              << "*lm_" << r << "_" << pi << ";\n";
        }
        ++eqcnt;
      }
    }
  }

  // dump equations for \c_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int pi2 = 0; pi2 < 5; ++pi2) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && pi2 == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && pi2 == 2) continue;

        if (pi == pi2) {
          out << "fcns(" << eqcnt << ")=" << "c_" << r << "_" << pi << "_" << pi2 << "-1;\n";
        }
        else {
          out << "fcns(" << eqcnt << ")=";
          out << "-c_" << r << "_" << pi << "_" << pi2;
          for (int po = 0; po < 5; ++po) {
            /// TODO: support for arbitrary controller placement, not only east/west
            if (r < mic->ColNum() && po == 0) continue;
            if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

            out << "+f_" << r << "_" << pi << "_" << po
                << "*f_" << r << "_" << pi2 << "_" << po;
          }
          out << ";\n";
        }
        ++eqcnt;
      }
    }
  }

  // dump equations for \lm_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      out << "fcns(" << eqcnt << ")=" << "-lm_" << r << "_" << pi;
      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        if (pi == po) continue;

        out << "+lm_" << r << "_" << pi << "_" << po;
      }

      out << ";\n";
      ++eqcnt;
    }
  }

  // dump equations lm_r_4 = prob(l3+mc)*lc
  /*for (int c=0; c < cmpConfig.ProcCnt(); ++c) {
    Processor * proc = cmpConfig.GetProcessor(c);
    out << "fcns(" << eqcnt << ")=";
    out << "-lm_" << proc->ClIdx() << "_4";
    out << "+l_" << c << "*" << (proc->L3AccessProbability() + proc->MainMemAccessProbability()) << ";\n";
    ++eqcnt;
  }*/

  // dump equations for \lm_r_pi_po
  for (int r = 0; r < clCmp->CompCnt(); ++r) {
    for (int pi = 0; pi < 5; ++pi) {
      /// TODO: support for arbitrary controller placement, not only east/west
      if (r < mic->ColNum() && pi == 0) continue;
      if (r >= mic->TCnt() - mic->ColNum() && pi == 2) continue;

      for (int po = 0; po < 5; ++po) {
        /// TODO: support for arbitrary controller placement, not only east/west
        if (r < mic->ColNum() && po == 0) continue;
        if (r >= mic->TCnt() - mic->ColNum() && po == 2) continue;

        if (pi == po) continue;

        out << "fcns(" << eqcnt << ")=-" << "lm_" << r << "_" << pi << "_" << po;

        for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
          Processor * proc = cmpConfig.GetProcessor(p);

          // communication proc <-> mem
          for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
            if (clCmp->Ic()->PathCompToCompFollowsPorts(
                UShort(clCmp->ProcOwner(p)->ClIdx()), UShort(clCmp->MemOwner(m)->ClIdx()),
                UShort(r), UShort(pi), UShort(po))) {
              out << "+" << "ln_p" << p << "_m" << m;
            }
            else if (clCmp->Ic()->PathCompToCompFollowsPorts(
                UShort(clCmp->MemOwner(m)->ClIdx()), UShort(clCmp->ProcOwner(p)->ClIdx()),
                UShort(r), UShort(pi), UShort(po))) {
              out << "+" << "ln_p" << p << "_m" << m;
            }
          } // L3 memories

          // communication proc <-> mc
          for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
            if (clCmp->Ic()->PathCompToMemCtrlFollowsPorts(
                UShort(clCmp->ProcOwner(p)->ClIdx()), UShort(mc),
                UShort(r), UShort(pi), UShort(po))) {
              out << "+" << "ln_p" << p << "_mc" << mc;
            }
            else if (clCmp->Ic()->PathMemCtrlToCompFollowsPorts(
                UShort(mc), UShort(clCmp->ProcOwner(p)->ClIdx()),
                UShort(r), UShort(pi), UShort(po))) {
              out << "+" << "ln_p" << p << "_mc" << mc;
            }
          } // memory controllers
        }

        out << ";\n";
        ++eqcnt;
      }
    }
  }

  // dump equations for \lambda_m_r_p
  for (int p = 0; p < cmpConfig.ProcCnt(); ++p) {
    Processor * proc = cmpConfig.GetProcessor(p);

    // communication proc <-> mem
    for (int m = 0; m < cmpConfig.MemCnt(); ++m) {
      out << "fcns(" << eqcnt << ")=";
      out << "-ln_p" << p << "_m" << m;
      out << "+" << proc->L3AccessProbability()*proc->L3ProbDistr()[m] << "*" << "l_" << p << ";\n";
      ++eqcnt;
    } // L3 memories

    // communication proc <-> mc
    for (int mc = 0; mc < cmpConfig.MemCtrlCnt(); ++mc) {
      out << "fcns(" << eqcnt << ")=";
      out << "-ln_p" << p << "_mc" << mc;
      out << "+" << proc->MainMemAccessProbability()/cmpConfig.MemCtrlCnt() << "*" << "l_" << p << ";\n";
      ++eqcnt;
    } // memory controllers
  }

  out << "end\n";
  out.close();
//  cout << "Lat = " << systemLat/cmpConfig.ProcCnt() << "; Thr = " << systemThr << endl;
//  cout << "Time = " << timer.Current() << endl;
  return new StatMetrics(systemThr, systemLat/cmpConfig.ProcCnt());
}

//=======================================================================

  } // namespace perf

} // namespace cmpex
