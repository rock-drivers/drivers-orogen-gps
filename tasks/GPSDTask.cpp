#include "GPSDTask.hpp"
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
#if GPSD_API_MAJOR_VERSION >= 9
            solution.altitude =  pdata->fix.altMSL;
#else
            solution.altitude =  pdata->fix.altitude;
#endif
            solution.latitude =  pdata->fix.latitude;
            solution.longitude =  pdata->fix.longitude;
            solution.noOfSatellites =  pdata->satellites_used;
#if GPSD_API_MAJOR_VERSION >= 9
            solution.geoidalSeparation = pdata->fix.geoid_sep;
#else
            solution.geoidalSeparation = pdata->separation;
#endif
            solution.deviationLongitude = pdata->fix.epx;
            solution.deviationLatitude= pdata->fix.epy;
            solution.deviationAltitude= pdata->fix.epv;
            solution.time = base::Time::now();
            solution.ageOfDifferentialCorrections = -1;

#if GPSD_API_MAJOR_VERSION >= 10
            switch(pdata->fix.status)
#else
            switch(pdata->status)
#endif
            {
            case STATUS_NO_FIX:
              solution.positionType = gps_base::NO_SOLUTION;
              break;
            
            case STATUS_FIX:
              if(pdata->fix.mode == MODE_2D){
                solution.positionType = gps_base::AUTONOMOUS_2D;
              }else{
                solution.positionType = gps_base::AUTONOMOUS;
              }
              break;
            case STATUS_DGPS_FIX:
              solution.positionType = gps_base::DIFFERENTIAL;
              break;

#if GPSD_API_MAJOR_VERSION >= 9
            case STATUS_RTK_FIX:
              solution.positionType = gps_base::RTK_FIXED;
              break;

            case STATUS_RTK_FLT:
              solution.positionType = gps_base::RTK_FLOAT;
              break;
#endif

            default:
              solution.positionType = gps_base::INVALID;
            }
#if GPSD_API_MAJOR_VERSION >= 10
            solution.ageOfDifferentialCorrections = (int) pdata->fix.status;
#else
            solution.ageOfDifferentialCorrections = (int) pdata->status;
#endif
            update(solution);
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

