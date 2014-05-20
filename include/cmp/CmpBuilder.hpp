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

#ifndef _CMP_CMPBUILDER_H_
#define _CMP_CMPBUILDER_H_

#include <string>
#include <fstream>
#include <map>
#include <string>
#include "Function.hpp"

#include "../Defs.hpp"

namespace cmpex {

  namespace arch {
    class ArchConfig;
  }
  
  namespace cmp {
    
    class Component;

    //======================================================================
    // This class provides methods for creating the Cmp composite structures
    // from files. Builder methods are static.
    //======================================================================

    class CmpBuilder {
      
      enum EntryType { ETUNDEF = -1, ETPARAM, ETFUNCTION, ETMEMCTRL, ETCOMP, ETDEFINE };
      
      // Argument pair: Name-Value
      struct Argument {
        std::string name;
        std::string value;
      };

    public:

      // List of defines
      typedef std::map<std::string, std::string> StringMap;
      typedef StringMap::const_iterator SMCIter;
      static StringMap Defines;
      
      // ---------------------------- Methods ------------------------------

    public:
      
      // Create Cmp object from file.
      static Component * ReadFromFile (const std::string& fname);
      
      // Create Cmp object from architectural configuration.
      static Component * ReadFromArchConfig (arch::ArchConfig& ac);

      // Converts architectural file to simulator-compatible
      // workload + architecture files
      static void WriteWorkloadArchFromArch (const std::string& fname);

      // Reads an entry from descriptor.
      static void ReadEntry (std::istream& in, Component *& topComp);
      
      // Reads a component from descriptor (recursively).
      static Component * ReadComponent (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);
      
      // Reads processor from given entry.
      static Component * ReadProcessor (
          std::string& entry, Component * parent = 0, UShort clIdx = MAX_USHORT);
      
      // Reads memory from given entry.
      static Component * ReadMemory (
          std::string& entry, Component * parent = 0, UShort clIdx = MAX_USHORT);
      
      // Reads mesh interconnect from given entry and stream.
      static Component * ReadMesh (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);

      // Reads bus interconnect from given entry and stream.
      static Component * ReadBus (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);

      // Reads unidirectional ring interconnect from given entry and stream.
      static Component * ReadURing (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);

      // Reads bidirectional ring interconnect from given entry and stream.
      static Component * ReadBRing (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);

      // Reads xbar interconnect from given entry and stream.
      static Component * ReadXBar (
          std::istream& in, std::string& entry, Component * parent = 0,
          UShort clIdx = MAX_USHORT);

      // Reads parameter from given entry.
      static void ReadParameter (std::string& entry);
      
      // Reads memory controller from given entry.
      static void ReadMemCtrl (std::string& entry);
      
      // Reads define from given entry.
      static void ReadDefine (std::string& entry);

      // Reads workload file.
      static void ReadWlFile (std::string& fName);

      // ------------------- Service functions --------------------
      // Retrieve type of entry encoded as a string.
      static EntryType GetEntryType (std::string& entry);
      
      // Returns true is 'entry' has arguments (actually, checks for '=' char).
      static bool HasMoreArgs ( const std::string& entry );
      
      // Extracts and returns next argument from 'entry'.
      static Argument ExtractNextArg ( std::string& entry );
      
      // Reads function from given entry.
      static void ReadFunction (std::string& entry);
      
      static double CoreLeakageOfTemp(double tmp);

      static double L1LeakageOfTemp(double tmp);

      static double L2LeakageOfTemp(double tmp);

      static double L3LeakageOfTemp(double tmp);

      static double RouterLeakageOfTemp(double tmp);

      static double LinksLeakageOfTemp(double tmp);

      static double McLeakageOfTemp(double tmp);

   protected:

    private:

      static model::Function * coreLeakageOfTemp_;
      
      static model::Function * l1LeakageOfTemp_;
      
      static model::Function * l2LeakageOfTemp_;
      
      static model::Function * l3LeakageOfTemp_;
      
      static model::Function * routerLeakageOfTemp_;

      static model::Function * linksLeakageOfTemp_;

      static model::Function * mcLeakageOfTemp_;

      // Deprecated methods: prevent usage
      CmpBuilder ();

      ~CmpBuilder ();
      
      CmpBuilder ( const CmpBuilder& );

      CmpBuilder& operator = ( const CmpBuilder& );

    };




  } // namespace cmp
  
} // namespace cmpex

#endif // _CMP_CMPBUILDER_H_
