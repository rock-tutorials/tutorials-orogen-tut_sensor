/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace tut_sensor;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }
// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::RigidBodyState local;
    base::samples::RigidBodyState target;

    if (_local_frame.readNewest(local) == RTT::NoData)
        return;
    if (_target_frame.readNewest(target) == RTT::NoData)
        return;

    Eigen::Vector3d v = (target.position - local.position);
    double d = v.norm();

    base::Angle yaw_delta;
    Eigen::Vector3d forward = Eigen::Vector3d::Zero();
    if (d > 0.01)
    {
        forward = local.orientation * base::Vector3d::UnitX();
        Eigen::Vector3d vertical = local.orientation * base::Vector3d::UnitZ();
        yaw_delta = base::Angle::vectorToVector(forward, v, base::Vector3d::UnitZ());
        if (vertical.z() < 0)
            yaw_delta = yaw_delta + base::Angle::fromRad(M_PI);

        rock_tutorial::BearingDistanceSensor result;
        result.time = std::max(local.time, target.time);
        result.bearing = yaw_delta;
        result.distance = d;
        _target_sensor_sample.write(result);
    }

}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

