/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef BUOY_DETECTOR_TASK_HPP
#define BUOY_DETECTOR_TASK_HPP

#include "opencv/cv.h"
#include "buoy/DetectorBase.hpp"
#include "visual_detectors/buoy_detector.h"
#include "visual_detectors/buoy_estimation_filter.h"
#include "visual_detectors/buoy_paradise_filter.h"
#include "visual_detectors/buoy_pos_estimation.h"
#include "visual_detectors/command_creation.h"
#include <base/pose.h>

namespace buoy {

    /*! \class Detector 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * time in seconds to wait in moving_to_cutting_distance before cutting.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','buoy::Detector')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Detector : public DetectorBase
    {
	friend class DetectorBase;
    protected:
		avalon::HSVColorBuoyDetector detector;
		avalon::BuoyPosEstimator posestimator;
    	avalon::BuoyParadiseFilter filter;
    	base::samples::frame::Frame frame;
    	IplImage image;
    	States previous_state;
    	States current_state;
    	RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> fp;//to read inputframes


    public:
        /** TaskContext constructor for Detector
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Detector(std::string const& name = "buoy::Detector", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Detector 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Detector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Detector
         */
	~Detector();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();



    };
}

#endif

