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
#include <cstring>

#include "Config.hpp"
#include "Util.hpp"
#include "Parser.hpp"
#include "Function.hpp"
#include "stat/Statistics.hpp"
#include "cmp/CmpConfig.hpp"

using namespace std;

namespace cmpex {

using model::Function;
using model::Sqrt;
using model::Powerlaw;
using model::Piecewise;
using model::Linear;

extern stat::Statistics stats;
extern cmp::CmpConfig cmpConfig;

//=======================================================================
/*
 * Constructors and destructor
 */

Config::Config ( void ) :
  test_ (""), configFile_ (""), expMode_ ("ex"), a2wa_ (false), tmap_ (false),
  eoTau_ (1.5), saAlpha_ (0.995), sEffort_ (5),
  maxPower_ (1.0e6), maxTemp_ (1.0e6), debug_ (0), dumpConfigs_ (false),
  dumpPTsimPower_ (false), callPTsim_ (false), qos_ (true),
  tech_ (TECH_16NM), uFreq_ (1.6), mcFreq_ (0.8),
  maxProcAreaPerCluster_ (1.0), wlFile_(""), maxArea_ (350.0),
  linkWidth_ (64),  simulateCC_ (false)
{
  numL3PerCluster_.push_back(1);
}

Config::~Config ()
{
  if (missRatioOfMemSize_) delete missRatioOfMemSize_;
  if (l3LatencyOfSize_) delete l3LatencyOfSize_;
  if (busTimeOfClusterArea_) delete busTimeOfClusterArea_;
  if (meshLinkDelayOfClusterArea_) delete meshLinkDelayOfClusterArea_;
  if (l3AccessEnergyOfSize_) delete l3AccessEnergyOfSize_;
  if (l3LeakagePowerOfSize_) delete l3LeakagePowerOfSize_;
}

//=======================================================================
/*
 * Parse command line and fill in the config properties.
 */

int Config::ParseCommandLine ( int argc, char ** argv )
{
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "-test")) {
      ++i;
      Test( string(argv[i]) );
    }
    else if (!strcmp(argv[i], "-config")) {
      ++i;
      ConfigFile( string(argv[i]) );
    }
    else if (!strcmp(argv[i], "-exp_mode")) {
      ++i;
      ExpMode( string(argv[i]) );
    }
    else if (!strcmp(argv[i], "-eo_tau")) {
      ++i;
      EoTau( atof(argv[i]) );
    }
    else if (!strcmp(argv[i], "-sa_a")) {
      ++i;
      SaAlpha( atof(argv[i]) );
    }
    else if (!strcmp(argv[i], "-s_eff")) {
      ++i;
      SEffort( atoi(argv[i]) );
    }
    else if (!strcmp(argv[i], "-debug")) {
      ++i;
      Debug( atoi(argv[i]) );
    }
    else if (!strcmp(argv[i], "-dump_configs")) {
      DumpConfigs(true);
    }
    else if (!strcmp(argv[i], "-print")) {
      ++i;
      stats.MaxConfig( atof(argv[i]) );
    }
    else if (!strcmp(argv[i], "-max_power")) {
      ++i;
      MaxPower( atof(argv[i]) );
    }
    else if (!strcmp(argv[i], "-max_temp")) {
      ++i;
      MaxTemp( atof(argv[i]) );
    }
    else if (!strcmp(argv[i], "-a2wa")) { // conversion mode
      A2wa(true);
    }
    else if (!strcmp(argv[i], "-tmap")) { // mapping mode
      Tmap(true);
    }
    else if (!strcmp(argv[i], "-simcc")) {
      SimulateCC(true);
      cmpConfig.SubnCnt(3); // 3 subnetworks if simulating CC
    }
    else if (!strcmp(argv[i], "-dump_ptsim_power")) {
      DumpPTsimPower(true);
    }
    else if (!strcmp(argv[i], "-call_ptsim")) {
      CallPTsim(true);
    }
    else if (!strcmp(argv[i], "-noqos")) {
      QoS(false);
    }
    else {
      cout << "Wrong argument: " << string(argv[i]) << endl;
      return 1;
    }
  }

  return 0;
}

//=======================================================================
/*
 * Parse configuration file.
 */

int Config::ParseConfigFile (const string& fname)
{
  ifstream in(fname.c_str());
  if (!in.good()) {
    cout << "-E- Can't read from file " << fname.c_str() << " -> Exiting..." << endl;
    exit(1);
  }
  string line;
  
  while (getline(in, line)) {

    LStrip(line);

    // skip empty lines
    if (line.empty()) continue;

    // skip comments
    if (line[0] == '#') continue;

    // read and append multilines (with sign '\' at the end)
    RStrip(line);
    while (line[line.length()-1] == '\\') {
      line.erase(line.length()-1);
      string buf;
      getline(in, buf);
      line += buf;
      RStrip(line);
    }

    int spIdx = line.find_first_of(" \t");
    if (spIdx == string::npos) {
      cerr << "-E- Wrong format of input line" << endl;
      return 1;
    }
    
    // Parse parameter
    string name = line.substr(0, spIdx);
    Strip(name);
    string values = line.substr(spIdx+1);
    ParseParamValues(name, values);
  }  
  
  in.close();
  return 0;
}

//=======================================================================
/*
 * Parse values of parameter by name.
 * Value string may be changed (for performance issues).
 */

void Config::ParseParamValues(const string& name, string& value)
{
  // erase equal sign if present
  int eqIdx = value.find_first_not_of(" \t=");
  if (eqIdx != string::npos) {
    value.erase(0, eqIdx);
  }

  if (name == "uFrequency") {
    UFreq(StrToDouble(value));
  }
  else if (name == "uMaxFrequency") {
    UFreqMax(StrToDouble(value));
  }
  else if (name == "uVoltage") {
    UVolt(StrToDouble(value));
  }
  else if (name == "uMaxVoltage") {
    UVoltMax(StrToDouble(value));
  }
  else if (name == "uMinVoltage") {
    UVoltMin(StrToDouble(value));
  }
  else if (name == "uNomVoltage") {
    UVoltNom(StrToDouble(value));
  }
  else if (name == "mcVoltage") {
    McVolt(StrToDouble(value));
  }
  else if (name == "mcMaxVoltage") {
    McVoltMax(StrToDouble(value));
  }
  else if (name == "mcMinVoltage") {
    McVoltMin(StrToDouble(value));
  }
  else if (name == "mcNomVoltage") {
    McVoltNom(StrToDouble(value));
  }
  else if (name == "mcFrequency") {
    McFreq(StrToDouble(value));
  }
  else if (name == "mcMaxFrequency") {
    McFreqMax(StrToDouble(value));
  }
  else if (name == "procMaxVoltage") {
    ProcVoltMax(StrToDouble(value));
  }
  else if (name == "procMinVoltage") {
    ProcVoltMin(StrToDouble(value));
  }
  else if (name == "procNomVoltage") {
    ProcVoltNom(StrToDouble(value));
  }
  else if (name == "procMaxFrequency") {
    ProcFreqMax(StrToDouble(value));
  }
  else if (name == "procMinVoltFrequency") {
    ProcMinVoltFreq(StrToDouble(value));
  }
  else if (name == "gMeshDimX") {
    ValueParser::GetVectorOfIValues(value, gMeshDimX_);
  }
  else if (name == "gMeshDimY") {
    ValueParser::GetVectorOfIValues(value, gMeshDimY_);
  }
  else if (name == "clusterIcType") {
    ValueParser::GetVectorOfValues(value, clusterIcType_);
  }
  else if (name == "maxArea") {
    MaxArea(StrToDouble(value));
  }
  else if (name == "processor") {
    string pName;
    double area, freq, volt, epi, pleak;
    int ooo;
    ValueParser::ExtractValue(value, pName);
    ValueParser::ExtractValue(value, area);
    ValueParser::ExtractValue(value, ooo);
    ValueParser::ExtractValue(value, freq);
    ValueParser::ExtractValue(value, volt);
    ValueParser::ExtractValue(value, epi);
    ValueParser::ExtractValue(value, pleak);

    // read L1 and L2 sizes
    vector<double> l1Size, l2Size;
    LStrip(value);
    value.erase(0, value.find('=')+1);
    string l1val = value.substr(0, value.find('l'));
    ValueParser::GetVectorOfValues(l1val, l1Size);
    value.erase(0, value.find('=')+1);
    ValueParser::GetVectorOfValues(value, l2Size);

    AddProc(pName, area, ooo, freq, volt, epi, pleak, l1Size, l2Size);
  }
  else if (name == "maxProcAreaPerCluster") {
    double val = StrToDouble(value);
    if (val > 1.0) {
      cout << "-W- maxProcAreaPerCluster adjusted to 1.0" << endl;
      val = 1.0;
    }
    else if (val < 0.1) {
      cout << "-W- maxProcAreaPerCluster adjusted to 0.1" << endl;
      val = 0.1;
    }
    MaxProcAreaPerCluster(val);
  }
  else if (name == "numL3PerCluster") {
    numL3PerCluster_.clear();
    ValueParser::GetVectorOfIValues(value, numL3PerCluster_);
  }
  else if (name == "memDensity") {
    MemDensity(StrToDouble(value));
  }
  else if (name == "wlFile") {
    WlFile(value);
  }
  else if (name == "l3LatencyOfSize") {
    l3LatencyOfSize_ = Function::ParseFunction(value);
  }
  else if (name == "l3ShareDegreeOfPNum") {
    l3ShareDegreeOfPNum_ = Function::ParseFunction(value);
  }
  else if (name == "busTimeOfClusterArea") {
    busTimeOfClusterArea_ = Function::ParseFunction(value);
  }
  else if (name == "meshLinkDelayOfClusterArea") {
    meshLinkDelayOfClusterArea_ = Function::ParseFunction(value);
  }
  else if (name == "xbarDelayOfClusterArea") {
    xbarDelayOfClusterArea_ = Function::ParseFunction(value);
  }
  else if (name == "missRatioOfMemSize") {
    missRatioOfMemSize_ = Function::ParseFunction(value);
  }
  else if (name == "l3AccessEnergyOfSize") {
    l3AccessEnergyOfSize_ = Function::ParseFunction(value);
  }
  else if (name == "l3LeakagePowerOfSize") {
    l3LeakagePowerOfSize_ = Function::ParseFunction(value);
  }
  else if (name == "memCtrlAccessEnergy") {
    MemCtrlAccessEnergy(StrToDouble(value));
  }
  else if (name == "memCtrlLeakagePower") {
    MemCtrlLeakagePower(StrToDouble(value));
  }
  else if (name == "linkWidth") {
    linkWidth_ = StrToInt(value);
  }
  else {
    cerr << "-W- Unknown parameter: " << name << endl;
  }
}
    
//=======================================================================
/*
 * Print configuration file.
 */

void Config::Print() const
{
  cout << "******************   Configuration file   ******************" << endl;

  cout << "gMeshDimX:";
  for (vector<UInt>::const_iterator it = GMeshDimXVec().begin();
         it != GMeshDimXVec().end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << endl;

  cout << "gMeshDimY:";
  for (vector<UInt>::const_iterator it = GMeshDimYVec().begin();
         it != GMeshDimYVec().end(); ++it) {
    cout << ' ' << (*it);
  }
  cout << endl;

  cout << "uFreq = " << UFreq() << ", uVolt = " << UVolt() << endl;

  for (int p = 0; p < ProcCnt(); ++p) {
    cout << "processor " << ProcName(p) << ": area = " << ProcArea(p)
         << "mm^2" /*<< ", ipc = " << ProcIpc(p) << ", mpi = " << ProcMpi(p)*/
         << ", freq = " << ProcFreq(p) << "GHz, volt = "
         << ProcVolt(p) << "V, epi = " << ProcEpi(p)
         << "nJ, pleak = " << ProcPleak(p) << "W" << endl;
  }

  cout << "======= Constraints =======" << endl;
  cout << "maxArea = " << MaxArea() << endl;
  cout << "maxPower = " << MaxPower() << endl;
}
    
//=======================================================================

} //namespace cmpex
