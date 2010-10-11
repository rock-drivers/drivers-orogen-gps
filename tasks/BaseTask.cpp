#include "BaseTask.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace dgps;


RTT::NonPeriodicActivity* BaseTask::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


BaseTask::BaseTask(std::string const& name, TaskCore::TaskState initial_state)
    : BaseTaskBase(name, initial_state)
{
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

// bool BaseTask::configureHook()
// {
//     return true;
// }
// bool BaseTask::startHook()
// {
//     return true;
// }

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

