#include "BaseTask.hpp"

#include <ogr_spatialref.h>

using namespace gps;

BaseTask::BaseTask(std::string const& name, TaskCore::TaskState initial_state)
    : BaseTaskBase(name, initial_state)
{
  _utm_zone.set(32);
  _utm_north.set(true);
}

BaseTask::BaseTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : BaseTaskBase(name, engine, initial_state)
{
  _utm_zone.set(32);
  _utm_north.set(true);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
    converter.setUTMZone(_utm_zone);
    converter.setUTMNorth(_utm_north);
    converter.setNWUOrigin(_nwu_origin);

    if (! BaseTaskBase::configureHook())
        return false;

    return true;
}
bool BaseTask::startHook()
{
    return true;
}

void BaseTask::update(const gps_base::Solution &solution)
{
    last_update = solution.time;
    _solution.write(solution);

    base::samples::RigidBodyState utm = converter.convertToUTM(solution);
    utm.sourceFrame = _gps_frame.get();
    utm.targetFrame = _utm_frame.get();
    _position_samples.write(utm);

    base::samples::RigidBodyState pos = converter.convertToNWU(utm);
    pos.sourceFrame = _gps_frame.get();
    pos.targetFrame = _nwu_frame.get();
    _nwu_position_samples.write(pos);
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

