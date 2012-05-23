/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Survey.hpp"

using namespace buoy;

Survey::Survey(std::string const& name, TaskCore::TaskState initial_state)
    : SurveyBase(name, initial_state)
{
}

Survey::Survey(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : SurveyBase(name, engine, initial_state)
{
}

Survey::~Survey()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Survey.hpp for more detailed
// documentation about them.

// bool Survey::configureHook()
// {
//     if (! SurveyBase::configureHook())
//         return false;
//     return true;
// }
// bool Survey::startHook()
// {
//     if (! SurveyBase::startHook())
//         return false;
//     return true;
// }
// void Survey::updateHook()
// {
//     SurveyBase::updateHook();
// }
// void Survey::errorHook()
// {
//     SurveyBase::errorHook();
// }
// void Survey::stopHook()
// {
//     SurveyBase::stopHook();
// }
// void Survey::cleanupHook()
// {
//     SurveyBase::cleanupHook();
// }

