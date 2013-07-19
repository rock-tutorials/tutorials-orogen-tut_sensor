/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TransformerTask.hpp"

using namespace tut_sensor;

TransformerTask::TransformerTask(std::string const& name)
    : TransformerTaskBase(name)
{
}

TransformerTask::TransformerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : TransformerTaskBase(name, engine)
{
}

TransformerTask::~TransformerTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TransformerTask.hpp for more detailed
// documentation about them.

bool TransformerTask::configureHook()
{
    if (! TransformerTaskBase::configureHook())
        return false;
    return true;
}
bool TransformerTask::startHook()
{
    if (! TransformerTaskBase::startHook())
        return false;
    return true;
}
void TransformerTask::updateHook()
{
    TransformerTaskBase::updateHook();

    base::samples::RigidBodyState target2ref;
    if (!_target2ref.get(base::Time::now(), target2ref, false))
        return; // no transformation available yet
    base::samples::RigidBodyState ref2world;
    if (!_ref2world.get(base::Time::now(), ref2world, false))
        return; // no transformation available yet

    Eigen::Vector3d v = target2ref.position;
    double d = v.norm();

    base::Angle yaw_delta;
    if (d > 0.01)
    {
        yaw_delta = base::Angle::vectorToVector(ref2world.orientation * base::Vector3d::UnitX(), v, base::Vector3d::UnitZ());
        if ((ref2world.orientation * Eigen::Vector3d::UnitZ()).z() < 0)
            yaw_delta = -1 * yaw_delta;

        //std::cout << std::endl;
        //std::cout << ref2world_aligned * base::Vector3d::UnitX() << std::endl;
        //std::cout << v << std::endl;
        //std::cout << yaw_delta << std::endl;

        rock_tutorial::BearingDistanceSensor result;
        result.time = std::max(target2ref.time, ref2world.time);
        result.bearing = yaw_delta;
        result.distance = d;
        _target_sensor_sample.write(result);
    }
}

void TransformerTask::errorHook()
{
    TransformerTaskBase::errorHook();
}
void TransformerTask::stopHook()
{
    TransformerTaskBase::stopHook();
}
void TransformerTask::cleanupHook()
{
    TransformerTaskBase::cleanupHook();
}
