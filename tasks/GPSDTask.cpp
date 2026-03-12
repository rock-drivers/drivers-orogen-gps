#include "GPSDTask.hpp"
#include "gps_base/BaseTypes.hpp"
#include <gps.h>
#include <libgpsmm.h>

using namespace gps;

GPSDTask::GPSDTask(std::string const& name)
    : GPSDTaskBase(name)
    , gpsd_daemon(0)
    , counter_waiting(0)
    , counter_polling(0)
{
}

GPSDTask::~GPSDTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GPSDTask.hpp for more detailed
// documentation about them.

bool GPSDTask::configureHook()
{
  if(!BaseTask::configureHook())
    return false;

#if GPSD_API_MAJOR_VERSION >= 5
  gpsd_daemon = new gpsmm(_hostname.value().c_str(),"2947");
#else
  gpsd_daemon = new gpsmm;
  gps_data_t* pdata = gpsd_daemon->open(_hostname.value().c_str(),"2947");
  if (pdata)
  {
      std::cout << "found gpsd deamon"<< std::endl;
  }
  else
  {
      std::cerr << "Error cannot find gpsd deamon"<< std::endl;
      return false;
  }
#endif
  gpsd_daemon->stream(WATCH_ENABLE);
  return true;
}

bool GPSDTask::startHook()
{
  if(!BaseTask::startHook())
    return false;

  counter_waiting = 0;
  counter_polling = 0;
  return true;
}

int gpsdFixStatus(gps_data_t const& gpsd) {
#if GPSD_API_MAJOR_VERSION >= 10
    return gpsd.fix.status;
#else
    return gpsd.status;
#endif
}

double gpsdFixAltitude(gps_data_t const& gpsd) {
#if GPSD_API_MAJOR_VERSION >= 9
    return gpsd.fix.altMSL;
#else
    return gpsd.fix.altitude;
#endif
}

double gpsdFixGeoidalSeparation(gps_data_t const& gpsd) {
#if GPSD_API_MAJOR_VERSION >= 9
    return gpsd.fix.geoid_sep;
#else
    return gpsd.fix.separation;
#endif
}

#if GPSD_API_MAJOR_VERSION >= 12
gps_base::GPS_SOLUTION_TYPES gpsdFixPositionType(gps_data_t const& gpsd) {
  switch(gpsd.fix.mode) {
    case MODE_NO_FIX:
      return gps_base::NO_SOLUTION;
    case MODE_2D:
      return gps_base::AUTONOMOUS_2D;
  }

  switch(gpsd.fix.status)
  {
  case STATUS_UNK:
    return gps_base::NO_SOLUTION;

  case STATUS_GPS:
    return gps_base::AUTONOMOUS;
  case STATUS_DGPS:
    return gps_base::DIFFERENTIAL;

  case STATUS_RTK_FIX:
    return gps_base::RTK_FIXED;

  case STATUS_RTK_FLT:
    return gps_base::RTK_FLOAT;

  default:
    return gps_base::INVALID;
  }
}
#else
gps_base::GPS_SOLUTION_TYPES gpsdFixPositionType(gps_data_t const& gpsd) {
  auto status = gpsdFixStatus(gpsd);
  switch(status)
  {
  case STATUS_NO_FIX:
    return gps_base::NO_SOLUTION;

  case STATUS_FIX:
    if(gpsd.fix.mode == MODE_2D){
      return gps_base::AUTONOMOUS_2D;
    } else {
      return gps_base::AUTONOMOUS;
    }
  case STATUS_DGPS_FIX:
    return gps_base::DIFFERENTIAL;

#if GPSD_API_MAJOR_VERSION >= 9
  case STATUS_RTK_FIX:
    return gps_base::RTK_FIXED;

  case STATUS_RTK_FLT:
    return gps_base::RTK_FLOAT;
#endif

  default:
    return gps_base::INVALID;
  }
}
#endif

void GPSDTask::updateHook()
{
#if GPSD_API_MAJOR_VERSION >= 5
      if(gpsd_daemon->waiting(0))
      {
          gps_data_t* pdata = gpsd_daemon->read();
#else
      if(gpsd_daemon->waiting())
      {
          counter_waiting = 0;
          gps_data_t* pdata = gpsd_daemon->poll();
#endif
          if (pdata)
          {
            counter_polling = 0;
            if(state() != RUNNING)
                state(RUNNING);

            gps_base::Solution solution;
            solution.altitude = gpsdFixAltitude(*pdata);
            solution.latitude =  pdata->fix.latitude;
            solution.longitude =  pdata->fix.longitude;
            solution.noOfSatellites =  pdata->satellites_used;
            solution.geoidalSeparation = gpsdFixGeoidalSeparation(*pdata);
            solution.deviationLongitude = pdata->fix.epx;
            solution.deviationLatitude= pdata->fix.epy;
            solution.deviationAltitude= pdata->fix.epv;
            solution.time = base::Time::now();
            solution.ageOfDifferentialCorrections = -1;

            solution.positionType = gpsdFixPositionType(*pdata);

            publishSolution(solution);
            // at the moment no constallation info is saved
            //
          }
          else
          {
              if(++counter_polling > _max_error_counter) {
                    std::cerr << "[Error] poll error" << std::endl;
                    return exception(IO_ERROR);
              } else {
                    std::cerr << "[Warning] poll error, got wrong values (" <<
                        counter_polling << "/" << _max_error_counter << " times)" << std::endl;
                    sleep(1);
              }
          }
      } else {
        counter_waiting++;
        if (counter_waiting == _max_error_counter) {
          std::cerr << "[Warning] poll error: gpsd is not responding" << std::endl;
          if(state() != IO_TIMEOUT)
              state(IO_TIMEOUT);
        }
        sleep(1);
      }
}


// void GPSDTask::errorHook()
// {
// }
// void GPSDTask::stopHook()
// {
// }
void GPSDTask::cleanupHook()
{
  delete gpsd_daemon;
  gpsd_daemon = 0;
}

