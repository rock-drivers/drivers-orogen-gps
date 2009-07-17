#include "Task.hpp"
#include <rtt/FileDescriptorActivity.hpp>

#include <iostream>
using namespace std;

using namespace dgps;

RTT::FileDescriptorActivity* Task::getFileDescriptorActivity()
{ return dynamic_cast< RTT::FileDescriptorActivity* >(getActivity().get()); }


Task::Task(std::string const& name)
    : TaskBase(name)
{
}

bool Task::configureHook()
{
    if (!gps.open(_device))
        return false;

    // start device
    getFileDescriptorActivity()->watch(gps.getFileDescriptor());
    return true;
}


bool Task::startHook()
{
    // start GPS information looping
    last_update = DFKI::Time();
    return gps.setPeriodicData(_port, _period);
}

void Task::updateHook()
{
    gps.collectPeriodicData();

    if (last_update < gps.position.timestamp && gps.position.timestamp == gps.errors.timestamp)
    {
        gps::Solution solution;
        solution.timestamp                    = gps.position.timestamp;
        solution.latitude                     = gps.position.latitude;
        solution.longitude                    = gps.position.longitude;
        solution.positionType                 = gps.position.positionType;
        solution.noOfSatellites               = gps.position.noOfSatellites;
        solution.altitude                     = gps.position.altitude;
        solution.geoidalSeparation            = gps.position.geoidalSeparation;
        solution.ageOfDifferentialCorrections = gps.position.ageOfDifferentialCorrections;
        solution.deviationLatitude            = gps.errors.deviationLatitude;
        solution.deviationLongitude           = gps.errors.deviationLongitude;
        solution.deviationAltitude            = gps.errors.deviationAltitude;
        _solution.write(solution);
    }

//if (_satellite.connected())
//        _satellite.write(gps.satellites);
}

// void Task::errorHook()
// {
// }

void Task::stopHook()
{
    gps.stopPeriodicData();
}

void Task::cleanupHook()
{
    gps.close();
}

