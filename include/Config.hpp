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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <Defs.hpp>
#include <TechDefs.hpp>

#include "Function.hpp"

using std::string;
using std::vector;

namespace cmpex {

  //======================================================================
  // Config is a storage for user-defined parameters.
  //======================================================================

  class Config
  {
    
    // ---------------------------- Methods ------------------------------

  public:

    Config ( void );

    ~Config ();
    
    int ParseCommandLine (int argc, char ** argv);
    
    int ParseConfigFile (const string& fname);
    
    void ParseParamValues(const string& name, string& values);

    // Accessors
    
    inline int Debug ( void ) const;

    inline void Debug ( int d );

    inline bool DumpConfigs ( void ) const;

    inline void DumpConfigs ( bool d );

    inline bool DumpPTsimPower ( void ) const;

    inline void DumpPTsimPower ( bool d );

    inline bool CallPTsim ( void ) const;

    inline void CallPTsim ( bool d );

    inline const string& Test ( void ) const;

    inline void Test ( string s );

    inline bool A2wa ( void ) const;

    inline void A2wa ( bool a );
    
    inline const string& ConfigFile ( void ) const;

    inline void ConfigFile ( string s );

    inline const string& ExpMode ( void ) const;

    inline void ExpMode ( string s );

    inline double EoTau ( void ) const;

    inline void EoTau ( double tau );

    inline double SaAlpha ( void ) const;

    inline void SaAlpha ( double a );

    inline int SEffort ( void ) const;

    inline void SEffort ( int e );

    inline double MaxPower ( void ) const;

    inline void MaxPower ( double p );

    inline Technology Tech () const;

    inline void Tech (Technology t);

    inline double Freq () const;

    inline void Freq (double f);

    inline double McFreq () const;

    inline void McFreq (double f);

    inline const vector<UInt>& GMeshDimXVec() const;

    inline const vector<UInt>& GMeshDimYVec() const;

    inline const vector<string>& ClusterIcType() const;

    inline double MaxProcAreaPerCluster() const;

    inline void MaxProcAreaPerCluster(double a);

    inline const vector<UInt>& NumL3PerCluster() const;

    inline double MemDensity() const;

    inline void MemDensity(double d);

    inline const string& WlFile() const;

    inline void WlFile(const string& f);

    inline double MaxArea() const;

    inline void MaxArea (double a);

    inline int ProcCnt() const;

    inline const string& ProcName (int idx) const;

    inline double ProcArea (int idx) const;

    inline int ProcOoO (int idx) const;

    inline double ProcFreq (int idx) const;

    inline double ProcEpi (int idx) const;

    inline double ProcPleak (int idx) const;

    inline int ProcL1SizeCnt (int idx) const;

    inline double ProcL1Size (int pIdx, int sIdx) const;

    inline int ProcL2SizeCnt (int idx) const;

    inline double ProcL2Size (int pIdx, int sIdx) const;

    inline void AddProc(const string& name, double area, int ooo,
                        double freq, double epi, double pleak,
                        const vector<double>& l1Size,
                        const vector<double>& l2Size);

    inline double MissRatioOfMemSize(double sz) const;

    inline UInt L3LatencyOfSize(double sz) const;

    inline double L3ShareDegreeOfPNum(int pNum) const;

    inline UInt BusTimeOfClusterArea(double area) const;

    inline UInt XBarDelayOfClusterArea(double area) const;

    inline UInt MeshLinkDelayOfClusterArea(double area) const;

    inline int LinkWidth() const;

    inline void LinkWidth (int w);

    inline double L3AccessEnergyOfSize(double sz) const;

    inline double L3LeakagePowerOfSize(double sz) const;

    inline double MemCtrlAccessEnergy () const;

    inline void MemCtrlAccessEnergy (double e);

    inline double MemCtrlLeakagePower () const;

    inline void MemCtrlLeakagePower (double p);

    inline bool SimulateCC () const;

    inline void SimulateCC ( bool s );

    void Print() const;

  private:

    string test_; // test name

    string configFile_; // config file name

    string expMode_; // exploration mode

    bool a2wa_; // conversion mode

    double eoTau_; // tau of extremal optimization

    double saAlpha_; // alpha of simulated annealing

    int sEffort_; // search effort (defines iteratioms before quit)

    double maxPower_; // power constraint

    /*** ============== debug and output options ================ ***/

    int debug_; // debug level

    bool dumpConfigs_; // write generated configurations to files

    bool dumpPTsimPower_; // dump power trace for ptsim simulator

    bool callPTsim_; // call ptsim if true
    
    /*** ============== general parameters ================ ***/

    Technology tech_;

    double freq_; // master clock frequency (NoC and caches) [GHz]

    double mcFreq_; // MC frequency [GHz]

    vector<UInt> gMeshDimX_;
    
    vector<UInt> gMeshDimY_;

    vector<string> clusterIcType_;

    double maxProcAreaPerCluster_;

    vector<UInt> numL3PerCluster_;

    double memDensity_;

    std::string wlFile_;

    model::Function * missRatioOfMemSize_;

    model::Function * l3LatencyOfSize_;

    model::Function * l3ShareDegreeOfPNum_;

    model::Function * busTimeOfClusterArea_;

    model::Function * xbarDelayOfClusterArea_;

    model::Function * meshLinkDelayOfClusterArea_;

    // === Constraints ===

    double maxArea_;

    // ==== Processors ===

    vector<string> procName_;

    vector<double> procArea_;

    vector<int> procOoO_;

    vector<double> procFreq_;

    vector<double> procEpi_;

    vector<double> procPleak_;

    vector<vector<double> > procl1Size_;

    vector<vector<double> > procl2Size_;

    // === Interconnect ===

    int linkWidth_;

    bool simulateCC_; // simulate cache coherence?

    // === Power ===

    model::Function * l3AccessEnergyOfSize_;

    model::Function * l3LeakagePowerOfSize_;

    double memCtrlAccessEnergy_;

    double memCtrlLeakagePower_;

  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  int Config::Debug () const {
    return debug_;
  }
  
  void Config::Debug ( int d ) {
    debug_ = d;
  }
  
  bool Config::DumpConfigs() const {
    return dumpConfigs_;
  }

  void Config::DumpConfigs ( bool d ) {
    dumpConfigs_ = d;
  }

  bool Config::DumpPTsimPower() const {
    return dumpPTsimPower_;
  }

  void Config::DumpPTsimPower( bool d ) {
    dumpPTsimPower_ = d;
  }

  bool Config::CallPTsim() const {
    return callPTsim_;
  }

  void Config::CallPTsim( bool d ) {
    callPTsim_ = d;
  }

  const string& Config::Test () const {
    return test_;
  }
  
  void Config::Test ( string s ) {
    test_ = s;
  }
  
  bool Config::A2wa () const {
    return a2wa_;
  }

  void Config::A2wa ( bool a ) {
    a2wa_ = a;
  }

  const string& Config::ConfigFile () const {
    return configFile_;
  }

  void Config::ConfigFile ( string s ) {
    configFile_ = s;
  }

  const string& Config::ExpMode () const {
    return expMode_;
  }

  void Config::ExpMode ( string s ) {
    expMode_ = s;
  }

  double Config::EoTau () const {
    return eoTau_;
  }

  void Config::EoTau ( double tau ) {
    eoTau_ = tau;
  }

  double Config::SaAlpha() const {
    return saAlpha_;
  }

  void Config::SaAlpha ( double a ) {
    saAlpha_ = a;
  }

  int Config::SEffort() const {
    return sEffort_;
  }

  void Config::SEffort ( int e ) {
    sEffort_ = e;
  }

  double Config::MaxPower() const {
    return maxPower_;
  }

  void Config::MaxPower ( double p ) {
    maxPower_ = p;
  }

  Technology Config::Tech () const {
    return tech_;
  }

  void Config::Tech(Technology t) {
    tech_ = t;
  }

  double Config::Freq() const {
    return freq_;
  }

  void Config::Freq(double f) {
    freq_ = f;
  }

  double Config::McFreq() const {
    return mcFreq_;
  }

  void Config::McFreq(double f) {
    mcFreq_ = f;
  }

  const vector<UInt>& Config::GMeshDimXVec () const {
    return gMeshDimX_;
  }
  
  const vector<UInt>& Config::GMeshDimYVec () const {
    return gMeshDimY_;
  }

  const vector<string>& Config::ClusterIcType() const {
    return clusterIcType_;
  }

  double Config::MaxProcAreaPerCluster() const {
    return maxProcAreaPerCluster_;
  }

  void Config::MaxProcAreaPerCluster(double a) {
    maxProcAreaPerCluster_ = a;
  }

  const vector<UInt>& Config::NumL3PerCluster() const {
    return numL3PerCluster_;
  }

  double Config::MemDensity() const {
    return memDensity_;
  }

  void Config::MemDensity(double d) {
    memDensity_ = d;
  }

  const std::string& Config::WlFile() const {
    return wlFile_;
  }

  void Config::WlFile(const std::string& f) {
    wlFile_ = f;
  }

  double Config::MaxArea() const {
    return maxArea_;
  }

  void Config::MaxArea (double a) {
    maxArea_ = a;
  }

  int Config::ProcCnt() const {
    return procName_.size();
  }

  const string& Config::ProcName(int idx) const {
    return procName_[idx];
  }

  double Config::ProcArea(int idx) const {
    return procArea_[idx];
  }
  
  int Config::ProcOoO(int idx) const {
    return procOoO_[idx];
  }

  double Config::ProcFreq(int idx) const {
    return procFreq_[idx];
  }

  double Config::ProcEpi(int idx) const {
    return procEpi_[idx];
  }

  double Config::ProcPleak(int idx) const {
    return procPleak_[idx];
  }

  int Config::ProcL1SizeCnt(int idx) const {
    return procl1Size_[idx].size();
  }

  double Config::ProcL1Size(int pIdx, int sIdx) const {
    return procl1Size_[pIdx][sIdx];
  }

  int Config::ProcL2SizeCnt(int idx) const {
    return procl2Size_[idx].size();
  }

  double Config::ProcL2Size(int pIdx, int sIdx) const {
    return procl2Size_[pIdx][sIdx];
  }

  void Config::AddProc(const string &name, double area,
                       int ooo, double freq,
                       double epi, double pleak,
                       const vector<double>& l1Size,
                       const vector<double>& l2Size) {
    procName_.push_back(name);
    procArea_.push_back(area);
    procOoO_.push_back(ooo);
    procFreq_.push_back(freq);
    procEpi_.push_back(epi);
    procPleak_.push_back(pleak);
    procl1Size_.push_back(l1Size);
    procl2Size_.push_back(l2Size);
  }

  double Config::MissRatioOfMemSize(double sz) const {
    return missRatioOfMemSize_->eval(sz);
  }

  UInt Config::L3LatencyOfSize(double sz) const {
    return 1 + static_cast<UInt>(l3LatencyOfSize_->eval(sz));
  }

  double Config::L3ShareDegreeOfPNum(int pNum) const {
    return l3ShareDegreeOfPNum_->eval(pNum);
  }

  UInt Config::BusTimeOfClusterArea(double area) const {
    return std::max(UInt(1), static_cast<UInt>(busTimeOfClusterArea_->eval(area)+0.5));
  }

  UInt Config::XBarDelayOfClusterArea(double area) const {
    return std::max(UInt(1), static_cast<UInt>(xbarDelayOfClusterArea_->eval(area)+0.5));
  }

  UInt Config::MeshLinkDelayOfClusterArea(double area) const {
    return std::max(UInt(1), static_cast<UInt>(meshLinkDelayOfClusterArea_->eval(area)+0.5));
  }

  int Config::LinkWidth() const {
    return linkWidth_;
  }

  void Config::LinkWidth(int w) {
    linkWidth_ = w;
  }

  bool Config::SimulateCC () const {
    return simulateCC_;
  }

  void Config::SimulateCC ( bool s ) {
    simulateCC_ = s;
  }

  double Config::L3AccessEnergyOfSize(double sz) const {
    return l3AccessEnergyOfSize_->eval(sz);
  }

  double Config::L3LeakagePowerOfSize(double sz) const {
    return l3LeakagePowerOfSize_->eval(sz);
  }

  double Config::MemCtrlAccessEnergy() const {
    return memCtrlAccessEnergy_;
  }

  void Config::MemCtrlAccessEnergy(double e) {
    memCtrlAccessEnergy_ = e;
  }

  double Config::MemCtrlLeakagePower() const {
    return memCtrlLeakagePower_;
  }

  void Config::MemCtrlLeakagePower(double p) {
    memCtrlLeakagePower_ = p;
  }

} // namespace cmpex

#endif // _CONFIG_H_
