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

#ifndef _STAT_STATISTICS_H_
#define _STAT_STATISTICS_H_

#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <algorithm>

#include "Defs.hpp"

using std::string;
using std::vector;
using std::multimap;

namespace cmpex {

  namespace stat {

    class StatConfig;

    //======================================================================
    // Statistics gathers data on explored configurations and generates
    // reports upon user request. Statistics owns all added StatConfigs.
    //======================================================================

    class Statistics {

      //typedef vector<StatConfig*> ConfigVector;
      typedef multimap<double, StatConfig*, std::greater<double> > ConfigVector;

      typedef ConfigVector::const_iterator CCIter;

      // ---------------------------- Methods ------------------------------

    public:

      // Constructors & destructor
      Statistics (int maxConfig); // number of (best) configurations to store

      virtual ~Statistics ();
      
      // Accessors

      inline int MaxConfig () const;

      inline void MaxConfig (int c);
	  
      inline bool NoDuplicates () const;

      inline void NoDuplicates (bool d);

      void AddConfig (StatConfig * sc);

      // Statistics

      void ReportAllConfigs() const;

      void DumpAllConfigs() const;

      void ReportWorstRoutability(UInt cnt) const;

      void ReportConfig(StatConfig * sc) const;

      void DumpConfigs() const;

      // Debug methods
      //void Print ();

    protected:

      inline const ConfigVector& Configs() const;

      inline ConfigVector& Configs();

    private:

      // Deprecated methods: prevent usage
      Statistics ( const Statistics& );

      Statistics& operator = ( const Statistics& );

      // -------------------------- Attributes -----------------------------

    private:

      ConfigVector configs_;

      int maxConfig_;
	  
      bool noDuplicates_;
      
    };

    //----------------------------------------------------------------------
    // Inline functions
    //----------------------------------------------------------------------

    int Statistics::MaxConfig () const {
      return maxConfig_;
    }

    void Statistics::MaxConfig (int c) {
      maxConfig_ = c;
    }

    bool Statistics::NoDuplicates () const {
      return noDuplicates_;
    }

    void Statistics::NoDuplicates (bool d) {
      noDuplicates_ = d;
    }

    const Statistics::ConfigVector& Statistics::Configs() const {
      return configs_;
    }

    Statistics::ConfigVector& Statistics::Configs() {
      return configs_;
    }

  } // namespace stat

} // namespace cmpex

#endif // _STAT_STATISTICS_H_
