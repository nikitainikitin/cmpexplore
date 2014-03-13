#ifndef _TIMER_H_
#define _TIMER_H_

// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File Timer.hpp
//   Created May 24, 2011
// ----------------------------------------------------------------------

#include <sys/resource.h>

namespace cmpex {

  //======================================================================
  // This is a timer object, with the capability of measuring time intervals.
  // To start and stop the timer, appropriate functions exist.
  // Upon stop, the last interval value is added to the total time value
  // that can be further retrieved.
  //
  // Total time stores the sum of all time intervals
  // between Start() and Stop() calls.
  //
  // Example of usage (runtime is shown in brackets):
  //
  //  ===================
  //  Timer t;
  //
  //  t.Start();      // interval 1 started   (0.0)
  //  //...           // code running 2 sec
  //  t.Current();    -> 2.0
  //  //...           // code running 2 sec
  //  t.Stop();       // stop interval 1      (4.0)
  //  t.LastTime();  -> 4.0
  //  //...           // code running 3 sec
  //  t.Start();      // interval 2 started   (7.0)
  //  //...           // code running 5 sec
  //  t.Stop();       // stop interval 2      (12.0)
  //  t.LastTime();  -> 5.0
  //  t.TotalTime(); -> 9.0
  //======================================================================

  class Timer {

    // ---------------------------- Methods ------------------------------

  public:

    Timer ();
    
    virtual ~Timer ();

    // Managing methods
    void Start ();

    void Stop ();
    
    void Reset ();

    // Time retrievers
    double Current ();

    double LastTime ();

    inline double TotalTime () const;

  protected: // methods

    // Converts time from structure rusage to double value (seconds)
    inline double ConvertTime ( struct rusage& r ) const;
    
  private:

    // Deprecated methods: prevent usage
    Timer ( const Timer& );

    Timer& operator = ( const Timer& );

    // -------------------------- Attributes -----------------------------

  private:

    bool running_; // true if timer was started but not finished

    double lastStart_; // last start time

    double lastStop_; // last stop time

    double totalTime_; // accumulated time

  };

  //----------------------------------------------------------------------
  // Inline functions
  //----------------------------------------------------------------------

  double Timer::TotalTime () const {
    return totalTime_;
  }
  
  double Timer::ConvertTime ( struct rusage& r ) const {
    return ( r.ru_utime.tv_sec + r.ru_stime.tv_sec +
             0.000001 * ( r.ru_utime.tv_usec + r.ru_stime.tv_usec ) );
  }
    
}; // namespace cmpex

#endif // _TIMER_H_
