#include "BaseTask.hpp"

#include <rtt/NonPeriodicActivity.hpp>
#include <ogr_spatialref.h>

using namespace dgps;
using namespace RTT;

RTT::NonPeriodicActivity* BaseTask::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


BaseTask::BaseTask(std::string const& name, TaskCore::TaskState initial_state)
    : BaseTaskBase(name, initial_state)
{
  _utm_zone.set(32);
  _utm_north.set(true);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
  // setup conversion from WGS84 to UTM
    OGRSpatialReference oSourceSRS;
    OGRSpatialReference oTargetSRS;
    
    oSourceSRS.SetWellKnownGeogCS( "WGS84" );
    oTargetSRS.SetWellKnownGeogCS( "WGS84" );
    oTargetSRS.SetUTM( _utm_zone, _utm_north );

    coTransform = OGRCreateCoordinateTransformation( &oSourceSRS,
	    &oTargetSRS );

    if( coTransform == NULL )
    {
	RTT::log(Error) << "failed to initialize CoordinateTransform UTM_ZONE:" << _utm_zone << " UTM_NORTH:" << _utm_north << RTT::endlog();
	return false;
    }
    return true;
}
bool BaseTask::startHook()
{
    last_update = base::Time();                
    last_constellation_update = base::Time();
    return true;
}


void BaseTask::updateConstallation(const ConstellationInfo &info)
{
   _constellation.write(info);
   last_constellation_update = base::Time::now();
}

void BaseTask::update(const gps::Solution &solution)
{
    last_update = solution.time;
	
    // if there is a valid reading, then write it to position readings port
    if( solution.positionType != gps::NO_SOLUTION )
    { 
        double la = solution.latitude;
        double lo = solution.longitude;
        double alt = solution.altitude;

        coTransform->Transform(1, &lo, &la, &alt);
        base::samples::RigidBodyState pos;
        pos.time = solution.time;
        pos.position.x() = lo - _origin.value().x();
        pos.position.y() = la - _origin.value().y();
        pos.position.z() = alt - _origin.value().z();
        pos.cov_position(0, 0) = solution.deviationLongitude * solution.deviationLongitude;
        pos.cov_position(1, 1) = solution.deviationLatitude * solution.deviationLatitude;
        pos.cov_position(2, 2) = solution.deviationAltitude * solution.deviationAltitude;
        _position_samples.write(pos);
    }
}

// void BaseTask::updateHook()
// {
// }

// void BaseTask::errorHook()
// {
// }
// void BaseTask::stopHook()
// {
// }
// void BaseTask::cleanupHook()
// {
// }

