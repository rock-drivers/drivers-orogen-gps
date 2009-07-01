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

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

 bool Task::getSatelliteInformation()
 {
	_satellite.write(gps.dataSatellite);
    return true;
 }

 bool Task::isgetSatelliteInformationCompleted()
 {
    return true;
 }

 bool Task::configureHook()
 {
	cerr << "device open on port :" << _portName << endl;
	if (!gps.open(_portName))
	//if (!gps.open("/dev/ttyUSB0"))
		 return false;

	 // start device
	 getFileDescriptorActivity()->watch(gps.getFileDescriptor());
	 return true;
 }


 bool Task::startHook()
 {
	 // start GPS information looping
	 return gps.setPeriodicData();
 }

 void Task::updateHook()
 {
	 gps.collectPeriodicData();
	 if( gps.isOKtoReleaseData()) {
		cerr << "Writing data to output port" << endl;
		_position_and_errors.write(gps.data);
	}

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

