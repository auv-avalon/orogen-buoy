/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Detector.hpp"

using namespace buoy;

Detector::Detector(std::string const& name, TaskCore::TaskState initial_state)
    : DetectorBase(name, initial_state)
{
}

Detector::Detector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DetectorBase(name, engine, initial_state)
{
}

Detector::~Detector()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Detector.hpp for more detailed
// documentation about them.

// bool Detector::configureHook()
// {
//     if (! DetectorBase::configureHook())
//         return false;
//     return true;
// }
// bool Detector::startHook()
// {
//     if (! DetectorBase::startHook())
//         return false;
//     return true;
// }
// void Detector::updateHook()
// {
//     DetectorBase::updateHook();
// }
// void Detector::errorHook()
// {
//     DetectorBase::errorHook();
// }
// void Detector::stopHook()
// {
//     DetectorBase::stopHook();
// }
// void Detector::cleanupHook()
// {
//     DetectorBase::cleanupHook();
// }

