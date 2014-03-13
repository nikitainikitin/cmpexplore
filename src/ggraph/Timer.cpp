// ----------------------------------------------------------------------
//   CMP experimental environment
//   
//   (c) Nikita Nikitin, UPC LSI
//   nikita.i.nikitin@gmail.com
//   
//   File Timer.cpp
//   Created May 24, 2011
// ----------------------------------------------------------------------

#include <iostream>
#include <stdexcept>

#include "Timer.hpp"

using std::cout;
using std::endl;

namespace cmpex {

//=======================================================================
/*
 * Constructors and destructor
 */

Timer::Timer ()
{
  Reset();
}

Timer::~Timer () {}

//=======================================================================
/*
 * Set default parameters.
 */

void Timer::Reset ()
{
  running_ = false;
  lastStart_ = 0.0;
  lastStop_ = 0.0;
  totalTime_ = 0.0;
}

//=======================================================================
/*
 * Start timer.
 */

void Timer::Start ()
{
  if (running_) {
    throw std::runtime_error("Timer: can't start timer that is already running\n");
  }
  
  running_ = true;
  struct rusage start;
  getrusage(RUSAGE_SELF, &start);
  lastStart_ = ConvertTime(start);
}

//=======================================================================
/*
 * Stop timer.
 */

void Timer::Stop ()
{
  if (!running_) {
    throw std::runtime_error("Timer: can't stop timer that is not running\n");
  }
  
  running_ = false;
  struct rusage finish;
  getrusage(RUSAGE_SELF, &finish);
  lastStop_ = ConvertTime(finish);
  
  totalTime_ += (lastStop_ - lastStart_);
}

//=======================================================================
/*
 * Retrieve current time, that has passed from the last(!) start.
 */

double Timer::Current ()
{
  if (!running_) {
    throw std::runtime_error("Timer: can't retrieve current as timer is not running\n");
  }
  
  struct rusage cur;
  getrusage(RUSAGE_SELF, &cur);
  return (ConvertTime(cur) - lastStart_);
}

//=======================================================================
/*
 * Get last time interval.
 */

double Timer::LastTime ()
{
  if (running_) {
    cout << "-WRN- Timer::LastTime() value is 0 when running\n";
    return 0.0;
  }
  
  return (lastStop_ - lastStart_);
}

//=======================================================================

}; // namespace cmpex
