/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ServoingOnWall.hpp"

using namespace buoy;

ServoingOnWall::ServoingOnWall(std::string const& name, TaskCore::TaskState initial_state)
    : ServoingOnWallBase(name, initial_state)
{
}

ServoingOnWall::ServoingOnWall(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ServoingOnWallBase(name, engine, initial_state)
{
}

ServoingOnWall::~ServoingOnWall()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ServoingOnWall.hpp for more detailed
// documentation about them.

bool ServoingOnWall::configureHook()
{
    if (! ServoingOnWallBase::configureHook())
        return false;
    return true;
}
bool ServoingOnWall::startHook()
{
    if (! ServoingOnWallBase::startHook())
        return false;
    last_buoy_validation = 0;

    return true;
}
void ServoingOnWall::updateHook()
{
    ServoingOnWallBase::updateHook();

    avalon::feature::Buoy buoy;
    sonar_detectors::Wall wall;
    base::samples::RigidBodyState orientation;
    
    if (_buoy_samples.readNewest(buoy) == RTT::NoData){
        error(WAIT_FOR_BUOY_SAMPLE);
        return;
    }
    
    if (_wall_samples.readNewest(wall) == RTT::NoData){
        error(WAIT_FOR_WALL_SAMPLE);
        return;
    }
    
    if (_orientation_samples.readNewest(orientation) == RTT::NoData){
        error(WAIT_FOR_ORIENTATION_SAMPLE);
        return;
    }

    base::LinearAngular6DCommand world_cmd, aligned_cmd;
    world_cmd.time = base::Time::now();
    aligned_cmd.time = base::Time::now();
    
    //TODO: Schwellwert als property
    if(buoy.validation > 0.8){
        if(last_buoy_validation < 0.8 && state() != BUOY_SERVOING && state() != ALIGNED){
            wall_on_buoy_detection = wall;
                start_heading = base::getYaw(orientation.orientation);
            state(BUOY_SERVOING);
        }

        double heading_step = base::Angle::normalizeRad(base::getYaw(orientation.orientation) + _heading_step_size.get());
        if(heading_step > _target_heading.get()){
            heading_step = _target_heading.get();
        }


        Eigen::Vector3d buoy_rel_pos_in_world = orientation.orientation * buoy.world_coord;
        Eigen::Vector3d buoy_offset = orientation.orientation.inverse() * (Eigen::AngleAxisd(heading_step, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(_distance_to_buoy.get(),0,0));
        
        buoy.world_coord[0] = buoy.world_coord[0] - 0.8;

        aligned_cmd.linear = buoy.world_coord + buoy_offset;
        world_cmd.angular = Eigen::Vector3d::Zero();
        world_cmd.angular[2] = atan2(buoy_rel_pos_in_world(1), buoy_rel_pos_in_world(0));

        std::cout << "\n\nworld coord:   " << buoy.world_coord.transpose() << std::endl;
        std::cout << "offset:      + " << buoy_offset.transpose() << std::endl;
        std::cout << "------------------------------------------------------" << std::endl;
        std::cout << "result:      + " << aligned_cmd.linear.transpose() << std::endl;
        
        if(world_cmd.linear.norm() < _aligned_distance.get() && base::getYaw(orientation.orientation) + 0.1 >= _target_heading.get()  && state() != ALIGNED){
            state(ALIGNED);
        }

    } else {
        if(state() != PASSIVE_BUOY_SEARCHING){
            //state(PASSIVE_BUOY_SEARCHING);
        }
    }
    _world_cmd.write(world_cmd);
    _aligned_position_cmd.write(aligned_cmd);

    last_buoy_validation = buoy.validation;
}

void ServoingOnWall::errorHook()
{
    ServoingOnWallBase::errorHook();
    
    base::LinearAngular6DCommand world_cmd, aligned_cmd;
    world_cmd.time = base::Time::now();
    aligned_cmd.time = base::Time::now();
    _world_cmd.write(world_cmd);
    _aligned_position_cmd.write(aligned_cmd);
    
    std::cout << "error Hook() - " <<state()  << std::endl; 
    if (state() == WAIT_FOR_BUOY_SAMPLE){
    std::cout << "error Hook() BUOY"  << std::endl; 
        avalon::feature::Buoy buoy;
        if(_buoy_samples.readNewest(buoy) == RTT::NewData){
            recover();
        }
    }else if( state() == WAIT_FOR_WALL_SAMPLE){
    std::cout << "error Hook() WALL"  << std::endl; 
        sonar_detectors::Wall wall;
        if (_wall_samples.readNewest(wall) == RTT::NewData){
            recover();
        }
    }else if( state() == WAIT_FOR_ORIENTATION_SAMPLE){
    std::cout << "error Hook() Orientation"  << std::endl; 
        base::samples::RigidBodyState orientation;
        if (_orientation_samples.readNewest(orientation) == RTT::NewData){
            std::cout << "recover" << std::endl;
            recover();
        }
    }
}
void ServoingOnWall::stopHook()
{
    ServoingOnWallBase::stopHook();
}
void ServoingOnWall::cleanupHook()
{
    ServoingOnWallBase::cleanupHook();
}
