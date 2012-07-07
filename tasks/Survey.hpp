/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef BUOY_SURVEY_TASK_HPP
#define BUOY_SURVEY_TASK_HPP

#include "buoy/SurveyBase.hpp"
#include "visual_detectors/command_creation.h"
#include <base/pose.h>

namespace buoy {

    /*! \class Survey 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * time in seconds to wait in moving_to_cutting_distance before cutting.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','buoy::Survey')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Survey : public SurveyBase
    {
	friend class SurveyBase;
    protected:
    	avalon::CommandCreator commander;
    	States previous_state;
    	States current_state;
    	base::samples::RigidBodyState ot;//to get aktuell RBS
    	base::samples::RigidBodyState servoing_rbs; //start-RBS of buoy-servoing
    	bool started_servoing;
    	bool started_cutting;
    	bool strafed_over_180_degrees;
		bool new_state;
		bool strafe_to_angle;
		bool angle_arrived;
		bool strafe_finished_bool;
		double target_heading;
		base::Time cutting_start;
		base::Time re_search_start;

		double winkelspiel;


    public:
        /** TaskContext constructor for Survey
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Survey(std::string const& name = "buoy::Survey", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Survey 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Survey(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Survey
         */
	~Survey();

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

		bool did180degrees();

        base::AUVPositionCommand createPositionCommand(avalon::feature::Buoy buoys);

		base::AUVPositionCommand buoy_search(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand buoy_detected(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand researching_buoy(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand buoy_lost(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand buoy_arrived(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand strafing(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand strafe_finished(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand moving_to_cutting_distance(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand cutting(avalon::feature::Buoy buoy, bool buoyfound);
		base::AUVPositionCommand cutting_success(avalon::feature::Buoy buoy, bool buoyfound);
    };
}

#endif

