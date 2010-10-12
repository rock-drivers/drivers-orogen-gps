#include "GPSDTask.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace dgps;


RTT::NonPeriodicActivity* GPSDTask::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


GPSDTask::GPSDTask(std::string const& name, TaskCore::TaskState initial_state)
    : GPSDTaskBase(name, initial_state)
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GPSDTask.hpp for more detailed
// documentation about them.

bool GPSDTask::configureHook()
{
  if(!BaseTask::configureHook())
    return false;

  gps_data_t* pdata = gpsd_daemon.open(_hostname.value().c_str(),"2947");
  if (pdata)
  {
      std::cout << "found gpsd deamon"<< std::endl;
  }
  else
  {
      std::cerr << "Error cannot find gpsd deamon"<< std::endl;
      return false;
  }
  gpsd_daemon.stream(WATCH_ENABLE);
  return true;
}

bool GPSDTask::startHook()
{
  if(!BaseTask::startHook())
    return false;
  counter = 0;
  return true;
}

void GPSDTask::updateHook()
{
  if(gpsd_daemon.waiting())
  {
      gps_data_t* pdata = gpsd_daemon.poll();
      if (pdata)
      {
        counter = 0;
        gps::Solution solution;
        solution.altitude =  pdata->fix.altitude;
        solution.latitude =  pdata->fix.latitude;
        solution.longitude =  pdata->fix.longitude;
        solution.noOfSatellites =  pdata->satellites_used;
        solution.geoidalSeparation = pdata->separation;
        solution.deviationLongitude = pdata->fix.epx;
        solution.deviationLatitude= pdata->fix.epy;
        solution.deviationAltitude= pdata->fix.epv;
        solution.time = base::Time::now();
        solution.ageOfDifferentialCorrections = -1;

        switch(pdata->status)
        {
        case STATUS_NO_FIX:
          solution.positionType = gps::NO_SOLUTION;
          break;

        case STATUS_FIX:
          solution.positionType = gps::RTK_FIXED;
          break;

        case STATUS_DGPS_FIX:
          solution.positionType = gps::DIFFERENTIAL;
          break;
        }
        update(solution);
        // at the moment no constallation info is saved
        //

      }
      else
      {
          std::cerr << "poll error" << std::endl;
          return BaseTask::error();
      }
  }
  else
    counter++;

  if (counter > 100)
  {
      std::cerr << "poll error: gpsd is not responding" << std::endl;
      return BaseTask::error();
  }
}


// void GPSDTask::errorHook()
// {
// }
// void GPSDTask::stopHook()
// {
// }
// void GPSDTask::cleanupHook()
// {
// }

