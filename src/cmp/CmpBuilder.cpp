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
#include <fstream>
#include <cmath>
#include <vector>
#include <cstdlib>

#include "CmpBuilder.hpp"
#include "Component.hpp"
#include "Defs.hpp"
#include "Util.hpp"
#include "Processor.hpp"
#include "Memory.hpp"
#include "Cluster.hpp"
#include "MeshIc.hpp"
#include "Config.hpp"
#include "Debug.hpp"
#include "MeshIcTile.hpp"
#include "CmpConfig.hpp"
#include "BusIc.hpp"
#include "arch/ArchConfig.hpp"
#include "URingIc.hpp"
#include "BRingIc.hpp"
#include "XBarIc.hpp"
#include "Parser.hpp"
#include "Function.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::ifstream;
using std::istream;
using std::ofstream;
using std::vector;

namespace cmpex {

  extern Config config;
  extern Debug debug;
  extern cmp::CmpConfig cmpConfig;
  
  using arch::ArchConfig;
  using namespace model;

  namespace cmp {
  
CmpBuilder::StringMap CmpBuilder::Defines;

//=======================================================================
/*
 * Creates Cmp object from file.
 */

Component * CmpBuilder::ReadFromFile (const string& fname)
{
  Component * cmp = 0;
  
  std::ifstream in(fname.c_str());
  while (in.good()) {
    ReadEntry(in, cmp);
  }
  in.close();
  
  cmpConfig.CalcTotalL3Size();
  
  DASSERT(cmp);
  
  DEBUG(5, "Total # of processors = " << cmpConfig.ProcCnt() << endl);
  DEBUG(5, "Total # of memories = " << cmpConfig.MemCnt() << endl);
  DEBUG(5, "Total L3 size = " << cmpConfig.TotalL3Size() << endl);
  
  CmpBuilder::Defines.clear();

  return cmp;
}

//=======================================================================
/*
 * Creates Cmp object from architectural configuration.
 */

Component * CmpBuilder::ReadFromArchConfig (ArchConfig& ac)
{
  Component * cmp = 0;

  // print config
  /*string buf;
  while (getline(ac.IStream(), buf))
    cout << buf << endl;
  return 0;*/

  //std::ifstream in(fname.c_str());
  while (ac.IStream().good()) {
    ReadEntry(ac.IStream(), cmp);
  }

  cmpConfig.CalcTotalL3Size();

  //cout << "Total L3 size = " << cmpConfig.TotalL3Size() << "Mb" << endl;

  DASSERT(cmp);

  DEBUG(5, "Total # of processors = " << cmpConfig.ProcCnt() << endl);
  DEBUG(5, "Total # of memories = " << cmpConfig.MemCnt() << endl);
  DEBUG(5, "Total L3 size = " << cmpConfig.TotalL3Size() << endl);

  CmpBuilder::Defines.clear();

  return cmp;
}

//=======================================================================
/*
 * Converts architectural file to simulator-compatible
 * workload + architecture files
 */

void CmpBuilder::WriteWorkloadArchFromArch (const std::string& fname)
{
  // 1. read in workloads
  std::ifstream in(fname.c_str());

  while (in.good()) {
    string entry;
    std::getline(in, entry);

    LStrip(entry);

    if (StartsWith(entry, "PARAM WlFile")) {
      ReadParameter(entry);
      break;
    }
  }
  in.close();

  if(system("mkdir -p ./test/sim"));

  string fidx = fname.substr(fname.find_last_of("/\\")+4,
                             fname.find_last_of(".") - fname.find_last_of("/\\") - 4);
  // 2. write output files
  for (int wl = 0; wl < cmpConfig.GetWlCnt(); ++wl) {
    cmpConfig.SetWlIdx(wl);
    std::ofstream out(string("test/sim/cmp" + fidx + "_" + IntToStr(wl+1) + ".cmp").c_str());

    int define_cnt = 0;
    std::ifstream in(fname.c_str());
    while (in.good()) {
      string entry;
      std::getline(in, entry);

      string outstr = entry;

      LStrip(entry);

      if (StartsWith(entry, "DEFINE Proc")) {
        // add processor parameters to define and write it to output file
        int typeIdx = outstr.find("Type=");
        int typeEndIdx = outstr.find_first_of(" ", typeIdx);

        int l3Idx = outstr.find("L3SizeEff");
        int l3EndIdx = outstr.find_first_of(" ", l3Idx);

        int l1SizeIdx = outstr.find("L1Size")+7;
        int l1SizeEndIdx = outstr.find_first_of(" ", l1SizeIdx);
        double l1Size = StrToDouble(outstr.substr(l1SizeIdx, l1SizeEndIdx-l1SizeIdx));
        double L1AccProb = 1.0 - cmpConfig.EvalCurWlMissRate(l1Size);

        int l2SizeIdx = outstr.find("L2Size")+7;
        int l2SizeEndIdx = outstr.find_first_of(" ", l2SizeIdx);
        double l2Size = StrToDouble(outstr.substr(l2SizeIdx, l2SizeEndIdx-l2SizeIdx));
        double L2AccProb = cmpConfig.EvalCurWlMissRate(l1Size) -
                           cmpConfig.EvalCurWlMissRate(l2Size+l1Size);

        int l3SizeIdx = outstr.find("L3SizeEff")+10;
        int l3SizeEndIdx = outstr.find_first_of(" ", l3SizeIdx);
        double l3Size = StrToDouble(outstr.substr(l3SizeIdx, l3SizeEndIdx-l3SizeIdx));
        double L3AccProb = (l3Size < E_DOUBLE) ? 0.0 :
                           (cmpConfig.EvalCurWlMissRate(l2Size+l1Size) -
                            cmpConfig.EvalCurWlMissRate(l3Size+l2Size+l1Size));

        double MMAccProb = (l3Size < E_DOUBLE) ?
                           cmpConfig.EvalCurWlMissRate(l2Size+l1Size) :
                           cmpConfig.EvalCurWlMissRate(l3Size+l2Size+l1Size);


        out << outstr.substr(0, typeIdx) << outstr.substr(typeEndIdx+1, l3Idx-(typeEndIdx+1))
            << outstr.substr(l3EndIdx+1)
            << " L1AccProb=" << L1AccProb << " L2AccProb=" << L2AccProb
            << " L3AccProb=" << L3AccProb << " MMAccProb=" << MMAccProb
            << " Ipc=" << cmpConfig.GetWorkload(wl)->ipc[define_cnt]
            << " Mpi=" << cmpConfig.GetWorkload(wl)->mpi[define_cnt] << endl;

        ++define_cnt;
      }
      else if (!StartsWith(entry, "PARAM WlFile")) {
        // copy to output file
        out << outstr << endl;
      }
    }
    in.close();

    out.close();
  }

  CmpBuilder::Defines.clear();
}

//=======================================================================
/*
 * Reads an entry from file specified by the descriptor.
 * Saves pointer to the top component into the argument.
 */

void CmpBuilder::ReadEntry (istream& in, Component *& topComp)
{
  if (!in.good())
    return;

  string entry;
  std::getline(in, entry);
  
  LStrip(entry);

  // skip comments and empty lines
  while (!entry.size() || entry[0] == '#') {
    if (!in.good())
      return;
    std::getline(in, entry);
  }
  
  switch (GetEntryType(entry)) {
    case ETPARAM:
      ReadParameter(entry); break;
    case ETMEMCTRL:
      ReadMemCtrl(entry); break;
    case ETCOMP:
      topComp = ReadComponent(in, entry); break;
    case ETDEFINE:
      ReadDefine(entry); break;
    default:
      cout << "-E-: Undefined entry type in CmpBuilder::ReadEntry()" << endl;
      exit(1);
  }
}

//=======================================================================
/*
 * Retrieve type of entry encoded as a string.
 */

CmpBuilder::EntryType CmpBuilder::GetEntryType (string& entry)
{
  EntryType type = ETCOMP;
  
  if (StartsWith(entry, "PARAM")) {
    type = ETPARAM;
  }
  else if (StartsWith(entry, "MEMCTRL")) {
    type = ETMEMCTRL;
  }
  else if (StartsWith(entry, "DEFINE")) {
    type = ETDEFINE;
  }

  return type;
}

//=======================================================================
/*
 * Reads a component from descriptor (recursively).
 */

Component * CmpBuilder::ReadComponent (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Component * comp = 0;
  
  LStrip(entry);
  
  if (StartsWith(entry, "PROC ")) {
    comp = ReadProcessor(entry, parent, clIdx);
  }
  else if (StartsWith(entry, "MEM ")) {
    comp = ReadMemory(entry, parent, clIdx);
  }
  else if (StartsWith(entry, "MESH ")) {
    comp = ReadMesh(in, entry, parent, clIdx);
  }
  else if (StartsWith(entry, "BUS ")) {
    comp = ReadBus(in, entry, parent, clIdx);
  }
  else if (StartsWith(entry, "URING ")) {
    comp = ReadURing(in, entry, parent, clIdx);
  }
  else if (StartsWith(entry, "BRING ")) {
    comp = ReadBRing(in, entry, parent, clIdx);
  }
  else if (StartsWith(entry, "XBAR ")) {
    comp = ReadXBar(in, entry, parent, clIdx);
  }
  else if (StartsWith(entry, "NI")) {
    // do nothing if NI (dummy component)
  }
  else {
    // try searching defines
    SMCIter it = Defines.find(entry);
    if (it != Defines.end()) { // build component from define
      string e = it->second;
      return ReadComponent(in, e, parent, clIdx);
    }
    cout << "-E-: Undefined entry type in CmpBuilder::ReadComponent()" << endl;
    exit(1);
  }
  
  return comp;
}

//=======================================================================
/*
 * Reads processor from given entry.
 */

Component * CmpBuilder::ReadProcessor (
    string& entry, Component * parent, UShort clIdx)
{
  Processor * p = new Processor(cmpConfig.ProcCnt(), clIdx, parent);
  cmpConfig.AddProcessor(p);
  
  entry.erase(0, entry.find(' ')+1); // erase type
  
  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "Left") {
      p->Left(StrToDouble(arg.value));
    }
    else if (arg.name == "Right") {
      p->Right(StrToDouble(arg.value));
    }
    else if (arg.name == "Top") {
      p->Top(StrToDouble(arg.value));
    }
    else if (arg.name == "Bottom") {
      p->Bottom(StrToDouble(arg.value));
    }
    else if (arg.name == "L1Size") {
      p->SetL1Size(StrToDouble(arg.value));
    }
    else if (arg.name == "L2Size") {
      p->SetL2Size(StrToDouble(arg.value));
    }
    else if (arg.name == "L3SizeEff") {
      p->SetL3SizeEff(StrToDouble(arg.value));
    }
    else if (arg.name == "L1Lat") {
      p->SetL1Lat(StrToInt(arg.value));
    }
    else if (arg.name == "L2Lat") {
      p->SetL2Lat(StrToInt(arg.value));
    }
    else if (arg.name == "OoO") {
      p->SetOoO(StrToInt(arg.value));
    }
    else if (arg.name == "Area") {
      p->SetArea(StrToDouble(arg.value));
    }
    /*else if (arg.name == "Ipc") {
      p->SetIpc(StrToDouble(arg.value));
    }
    else if (arg.name == "Mpi") {
      p->SetMpi(StrToDouble(arg.value));
    }*/
    else if (arg.name == "Type") {
      p->SetType(StrToInt(arg.value));
    }
    else if (arg.name == "Active") {
      p->SetActive(StrToInt(arg.value));
    }
    /*else if (arg.name == "L1AccProb") {
      p->SetL1AccProb(StrToDouble(arg.value));
    }
    else if (arg.name == "L2AccProb") {
      p->SetL2AccProb(StrToDouble(arg.value));
    }
    else if (arg.name == "L3AccProb") {
      p->SetL3AccProb(StrToDouble(arg.value));
    }
    else if (arg.name == "MMAccProb") {
      p->SetMMAccProb(StrToDouble(arg.value));
    }*/
    else if (arg.name == "Freq") {
      p->SetFreq(StrToDouble(arg.value));
    }
    else if (arg.name == "Epi") {
      p->SetEpi(StrToDouble(arg.value));
    }
    else if (arg.name == "Pleak") {
      p->SetPleak(StrToDouble(arg.value));
    }
    else if (arg.name == "L1Eacc") {
      p->SetL1Eacc(StrToDouble(arg.value));
    }
    else if (arg.name == "L1Pleak") {
      p->SetL1Pleak(StrToDouble(arg.value));
    }
    else if (arg.name == "L2Eacc") {
      p->SetL2Eacc(StrToDouble(arg.value));
    }
    else if (arg.name == "L2Pleak") {
      p->SetL2Pleak(StrToDouble(arg.value));
    }
  }
  
  DEBUG(5, "Created a processor" << endl);
  
  return p;
}

//=======================================================================
/*
 * Reads memory from given entry.
 */

Component * CmpBuilder::ReadMemory (
    string& entry, Component * parent, UShort clIdx)
{
  Memory * m = new Memory(cmpConfig.MemCnt(), clIdx, MTL3, parent);
  cmpConfig.AddMemory(m);
  
  entry.erase(0, entry.find(' ')+1); // erase type
  
  bool sizeSet = false;

  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "Left") {
      m->Left(StrToDouble(arg.value));
    }
    else if (arg.name == "Right") {
      m->Right(StrToDouble(arg.value));
    }
    else if (arg.name == "Top") {
      m->Top(StrToDouble(arg.value));
    }
    else if (arg.name == "Bottom") {
      m->Bottom(StrToDouble(arg.value));
    }
    else if (arg.name == "Size") {
      m->SetSize(StrToDouble(arg.value));
      sizeSet = true;
    }
    else if (arg.name == "Latency") {
      m->SetLatency(StrToDouble(arg.value));
    }
    else if (arg.name == "Eacc") {
      m->SetEacc(StrToDouble(arg.value));
    }
    else if (arg.name == "Pleak") {
      m->SetPleak(StrToDouble(arg.value));
    }
  }

  // if size was not specified
  if (!sizeSet) {
    m->SetSize( fabs(m->Right()-m->Left()) * fabs(m->Bottom()-m->Top()) /
                cmpConfig.MemDensity() );
  }

  // if latency was not specified
  if (m->Latency() < E_DOUBLE) {
    m->SetLatency(cmpConfig.L3Latency());
  }
  
  DEBUG(5, "Created a memory" << endl);

  return m;
}

//=======================================================================
/*
 * Reads mesh interconnect from given entry and stream.
 */

Component * CmpBuilder::ReadMesh (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Cluster * c = new Cluster(cmpConfig.ClusterCnt(), clIdx, parent);
  cmpConfig.AddCluster(c);
  MeshIc * mesh = new MeshIc(c, cmpConfig.SubnCnt());
  c->SetIc(mesh);
  
  DEBUG(5, "Created a mesh" << endl);

  int colNum = 0, rowNum = 0;
  
  entry.erase(0, entry.find(' ')+1); // erase type
  
  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "Col") {
      colNum = StrToInt(arg.value);
    }
    else if (arg.name == "Row") {
      rowNum = StrToInt(arg.value);
    }
    else if (arg.name == "ColWidth") {
      mesh->ColWidth(StrToDouble(arg.value));
    }
    else if (arg.name == "RowHeight") {
      mesh->RowHeight(StrToDouble(arg.value));
    }
    else if (arg.name == "LinkDelay") {
      mesh->LinkDelay(StrToInt(arg.value));
      DASSERT(mesh->LinkDelay() > 0);
    }
    else if (arg.name == "RouterDelay") {
      mesh->RouterDelay(StrToInt(arg.value));
      DASSERT(mesh->RouterDelay() > 0);
    }
  }
  
  mesh->Init(colNum, rowNum);

  // recursively read and create subcomponents
  for (int i = 0; i < colNum*rowNum; ++i) {
    entry.clear();
    std::getline(in, entry);
    Component * comp = ReadComponent(in, entry, c, i);
    c->AddComponent(comp);
    mesh->GetTile(i)->SetComponent(comp);
  }

  return c;
}

//=======================================================================
/*
 * Reads bus interconnect from given entry and stream.
 */

Component * CmpBuilder::ReadBus (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Cluster * c = new Cluster(cmpConfig.ClusterCnt(), clIdx, parent);
  cmpConfig.AddCluster(c);
  BusIc * bus = new BusIc(c, cmpConfig.SubnCnt());
  c->SetIc(bus);
  
  DEBUG(5, "Created a bus" << endl);

  int compCnt = 0;
  
  entry.erase(0, entry.find(' ')+1); // erase type
  
  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "CompCnt") {
      compCnt = StrToInt(arg.value);
      bus->CompCnt(compCnt);
    }
    else if (arg.name == "AccessTime") {
      bus->AccessTime(StrToInt(arg.value));
      DASSERT(bus->AccessTime() > 0);
    }
  }
  
  // recursively read and create subcomponents
  for (int i = 0; i < compCnt; ++i) {
    entry.clear();
    std::getline(in, entry);
    Component * comp = ReadComponent(in, entry, c, i);
    if (comp) {
      c->AddComponent(comp);
    }
    else {
      bus->CompCnt(bus->CompCnt()-1);
    }
  }

  return c;
}

//=======================================================================
/*
 * Reads undirectional ring interconnect from given entry and stream.
 */

Component * CmpBuilder::ReadURing (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Cluster * c = new Cluster(cmpConfig.ClusterCnt(), clIdx, parent);
  cmpConfig.AddCluster(c);

  URingIc * ring = new URingIc(c, cmpConfig.SubnCnt());
  c->SetIc(ring);

  DEBUG(5, "Created a u-ring" << endl);

  int compCnt = 0;

  entry.erase(0, entry.find(' ')+1); // erase type

  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "CompCnt") {
      compCnt = StrToInt(arg.value);
      ring->CompCnt(compCnt);
    }
    else if (arg.name == "RouterDelay") {
      ring->RouterDelay(StrToInt(arg.value));
      DASSERT(ring->RouterDelay() > 0);
    }
    else if (arg.name == "LinkDelay") {
      ring->LinkDelay(StrToInt(arg.value));
      //DASSERT(ring->LinkDelay() > 0);
    }
  }

  // recursively read and create subcomponents
  for (int i = 0; i < compCnt; ++i) {
    entry.clear();
    std::getline(in, entry);
    Component * comp = ReadComponent(in, entry, c, i);
    if (comp) {
      c->AddComponent(comp);
    }
    else {
      ring->CompCnt(ring->CompCnt()-1);
    }
  }

  return c;
}

//=======================================================================
/*
 * Reads bidirectional ring interconnect from given entry and stream.
 */

Component * CmpBuilder::ReadBRing (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Cluster * c = new Cluster(cmpConfig.ClusterCnt(), clIdx, parent);
  cmpConfig.AddCluster(c);

  BRingIc * ring = new BRingIc(c, cmpConfig.SubnCnt());
  c->SetIc(ring);

  DEBUG(5, "Created a b-ring" << endl);

  int compCnt = 0;

  entry.erase(0, entry.find(' ')+1); // erase type

  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "CompCnt") {
      compCnt = StrToInt(arg.value);
      ring->CompCnt(compCnt);
    }
    else if (arg.name == "RouterDelay") {
      ring->RouterDelay(StrToInt(arg.value));
      DASSERT(ring->RouterDelay() > 0);
    }
    else if (arg.name == "LinkDelay") {
      ring->LinkDelay(StrToInt(arg.value));
      //DASSERT(ring->LinkDelay() > 0);
    }
  }

  // recursively read and create subcomponents
  for (int i = 0; i < compCnt; ++i) {
    entry.clear();
    std::getline(in, entry);
    Component * comp = ReadComponent(in, entry, c, i);
    if (comp) {
      c->AddComponent(comp);
    }
    else {
      ring->CompCnt(ring->CompCnt()-1);
    }
  }

  return c;
}

//=======================================================================
/*
 * Reads xbar interconnect from given entry and stream.
 */

Component * CmpBuilder::ReadXBar (
    istream& in, string& entry, Component * parent, UShort clIdx)
{
  Cluster * c = new Cluster(cmpConfig.ClusterCnt(), clIdx, parent);
  cmpConfig.AddCluster(c);
  XBarIc * xbar = new XBarIc(c, cmpConfig.SubnCnt());
  c->SetIc(xbar);

  DEBUG(5, "Created an xbar" << endl);

  int compCnt = 0;

  entry.erase(0, entry.find(' ')+1); // erase type

  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "CompCnt") {
      compCnt = StrToInt(arg.value);
      xbar->CompCnt(compCnt);
    }
    else if (arg.name == "Delay") {
      xbar->Delay(StrToInt(arg.value));
      DASSERT(xbar->Delay() > 0);
    }
  }

  // recursively read and create subcomponents
  for (int i = 0; i < compCnt; ++i) {
    entry.clear();
    std::getline(in, entry);
    Component * comp = ReadComponent(in, entry, c, i);
    if (comp) {
      c->AddComponent(comp);
    }
    else {
      xbar->CompCnt(xbar->CompCnt()-1);
    }
  }

  return c;
}

//=======================================================================
/*
 * Reads parameter from given entry.
 * Parameter is assumed to have only one argument pair.
 */

void CmpBuilder::ReadParameter (string& entry)
{
  entry.erase(0, entry.find(' ')+1); // erase type

  DASSERT(HasMoreArgs(entry));
  Argument arg = ExtractNextArg(entry);
  if (arg.name == "UnitLen") {
    cmpConfig.UnitLen(StrToDouble(arg.value));
  }
  else if (arg.name == "MemDensity") {
    cmpConfig.MemDensity(StrToDouble(arg.value));
  }
  else if (arg.name == "Frequency") {
    cmpConfig.Freq(StrToDouble(arg.value));
  }
  else if (arg.name == "McFrequency") {
    cmpConfig.McFreq(StrToDouble(arg.value));
  }
  else if (arg.name == "L3LatencyDef") {
    cmpConfig.L3Latency(StrToDouble(arg.value));
  }
  else if (arg.name == "MemReplySize") {
    cmpConfig.MemReplySize(StrToInt(arg.value));
  }
  else if (arg.name == "LinkWidth") {
    config.LinkWidth(StrToInt(arg.value));
  }
  else if (arg.name == "NiDelay") {
    cmpConfig.NiDelay(StrToDouble(arg.value));
  }
  else if (arg.name == "L3ClusterSize") {
    cmpConfig.L3ClusterSize(StrToInt(arg.value));
  }
  else if (arg.name == "WlFile") {
    if (cmpConfig.WlFile() != arg.value) {
      // only read workload file if it is different from the previous one;
      // otherwise use cached workloads
      for (int i = 0; i < cmpConfig.workloads_.size(); ++i)
        delete cmpConfig.workloads_[i];
      cmpConfig.workloads_.clear();
      ReadWlFile(arg.value);
      cmpConfig.WlFile(arg.value);
    }
  }
  else {
    cout << "-W-: Unknown parameter " << arg.name << endl;
  }
  
  DEBUG(5, "Read parameter " << arg.name << endl);
}

//=======================================================================
/*
 * Reads memory controller from given entry.
 */

void CmpBuilder::ReadMemCtrl (string& entry)
{
  entry.erase(0, entry.find(' ')+1); // erase type
  
  string type;
  double lat, eacc, pleak;
  
  while (HasMoreArgs(entry)) { // handle arguments
    Argument arg = ExtractNextArg(entry);
    if (arg.name == "Location") {
      type = arg.value;
    }
    else if (arg.name == "Latency") {
      lat = StrToDouble(arg.value);
    }
    else if (arg.name == "Eacc") {
      eacc = StrToDouble(arg.value);
    }
    else if (arg.name == "Pleak") {
      pleak = StrToDouble(arg.value);
    }
  }
  
  cmpConfig.AddMemCtrl(type, lat, eacc, pleak);
  
  DEBUG(5, "Added memory controller " << type << ", lat = " << lat
           << ", eacc = " << eacc << ", pleak = " << pleak << endl);
}

//=======================================================================
/*
 * Reads workload file.
 */

void CmpBuilder::ReadWlFile (string& fname)
{
  ifstream in(fname.c_str());

  if (!in.good()) {
    cout << "-E- Can't read from file " << fname.c_str() << " -> Exiting..." << endl;
    exit(1);
  }
  string line;
  CmpConfig::Workload * wl = 0;

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
      return;
    }

    // Parse parameter
    string name = line.substr(0, spIdx);
    Strip(name);
    string value = line.substr(spIdx+1);

    // Parse parameter value
    // erase equal sign if present
    int eqIdx = value.find_first_not_of(" \t=");
    if (eqIdx != string::npos) {
      value.erase(0, eqIdx);
    }

    if (name == "wlName") {
      if (wl) cmpConfig.AddWorkload(wl);
      wl = new CmpConfig::Workload;
      wl->name = value;
    }
    else if (name == "wlShortName") {
      DASSERT(wl);
      wl->shortName = value;
    }
    else if (name == "missRatioOfMemSize") {
      wl->missRatioOfMemSize = Function::ParseFunction(value);
    }
    else if (name == "IPC") {
      vector<double> ipc;
      ValueParser::GetVectorOfValues(value, ipc);
      wl->ipc.assign(ipc.begin(), ipc.end());
    }
    else if (name == "MPI") {
      vector<double> mpi;
      ValueParser::GetVectorOfValues(value, mpi);
      wl->mpi.assign(mpi.begin(), mpi.end());
    }
    else {
      cerr << "-W- Unknown parameter: " << name << endl;
    }
  }

  if (wl) cmpConfig.AddWorkload(wl);

  /*cout << fname << endl;
  cout << "NumWL = " << cmpConfig.workloads_.size() << endl;
  for (int i = 0; i < cmpConfig.workloads_.size(); ++i) {
    cout << "Name = " << cmpConfig.GetWorkload(i)->name << endl;
    cout << "sName = " << cmpConfig.GetWorkload(i)->shortName << endl;
    cout << "Miss(0.064) = " << cmpConfig.GetWorkload(i)->missRatioOfMemSize->eval(0.128) << endl;
    cout << "Miss(128) = " << cmpConfig.GetWorkload(i)->missRatioOfMemSize->eval(128) << endl;
    for (int c = 0; c < 3; ++c) {
        cout << "ipc = " << cmpConfig.GetWorkload(i)->ipc[c]
             << ", mpi = " << cmpConfig.GetWorkload(i)->mpi[c] << endl;
    }
  }*/

  in.close();
}

//=======================================================================
/*
 * Reads define from given entry.
 * It is stored in the 'Defines' map of CmpBuilder.
 */

void CmpBuilder::ReadDefine (string& entry)
{
  entry.erase(0, entry.find(' ')+1); // erase type

  string name = entry.substr(0, entry.find(' '));
  entry.erase(0, entry.find(' ')+1); // erase name
  Defines.insert(make_pair(name, entry));

  DEBUG(5, "Read define " << name << endl);
}

//=======================================================================
/*
 * Returns true if 'entry' has arguments (actually, checks for '=' char).
 */

bool CmpBuilder::HasMoreArgs (const string& entry)
{
  return (entry.find('=') == string::npos) ? false : true;
}

//=======================================================================
/*
 * Extracts and returns next argument from 'entry'.
 */

CmpBuilder::Argument CmpBuilder::ExtractNextArg (string& entry)
{
  Argument arg;
  
  arg.name = entry.substr(entry.find_first_not_of(' '), entry.find('='));
  entry.erase(0, entry.find('=')+1);
  arg.value = entry.substr(0, entry.find(' '));
  entry.erase(0, entry.find(' ')+1);

  return arg;
}

//=======================================================================

  } // namespace cmp

} // namespace cmpex
