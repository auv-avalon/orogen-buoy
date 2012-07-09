/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Survey.hpp"
//#include "visual_detectors/buoy_pos_estimation.h"
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <iostream>

bool new_cutting = true;
double buoy_depth;
base::Time mov_to_cut_time;

using namespace avalon;
using namespace buoy;


Survey::Survey(std::string const& name, TaskCore::TaskState initial_state)
    : SurveyBase(name, initial_state)
{
	started_servoing=false;
}

Survey::Survey(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : SurveyBase(name, engine, initial_state)
{
	started_servoing=false;
}

Survey::~Survey()
{
	started_servoing=false;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Survey.hpp for more detailed
// documentation about them.

bool Survey::configureHook()
{
    if (! SurveyBase::configureHook())
        return false;
    return true;
}


bool Survey::startHook()
{
    if (! SurveyBase::startHook()){
   		std::cout<<"startHook: FALSE"<<std::endl;
		return false;
	}else{
		std::cout<<"startHook: TRUE"<<std::endl;
	}
    commander = CommandCreator(_max_buoy_distance.get());

    previous_state=BUOY_SEARCH;
    current_state=BUOY_SEARCH;
    started_servoing=false;
    started_cutting=false;
    strafed_over_180_degrees=false;
	new_state=true;
	strafe_to_angle=false;
	angle_arrived=false;
	strafe_finished_bool=false;
	target_heading=0;
	winkelspiel=0.08;	//=0.4

    return true;
}


void Survey::updateHook()
{
	commander.setGoodDist(_max_buoy_distance.get());
    SurveyBase::updateHook();

	_orientation_samples.read(ot);

    base::AUVPositionCommand command;
    command.heading=command.x=command.y=0;
	command.z=ot.getPose().position[2];

	//if the property is true, change to STRAFE_FINISHED-state
	bool force=false;									/**** ACHTUNG!!!! Chris hat hier _force_cutting mit irgend einem bool connected!!!  ****/
/*    if(_force_cutting.read(force) == RTT::NewData && force){
        strafed_over_180_degrees=false;
        started_servoing=false;
        previous_state=current_state;
        current_state=STRAFE_FINISHED;
		new_state=true;
	}*/
	//wenn ein target-winkel übergeben wird, gehe zu diesem winkel
	if(_target_angle_input.read(target_heading) == RTT::NewData){
		strafe_to_angle=true;
	}
	
/*
 *	zu jeder zeit müssen das heading und der state des lichtes per modem übertragen werden
 *
 *  - herum fahren und abei daten senden...
 *  - ...bis der befehl kommt auf zu hören
 *  - das vorgeschlagene heading ansteuern
 */
	//if light is on handle it
	bool light = false;
	if(_light.read(light) == RTT::NewData && light){
		//hier muss dem modem so einiges erzählt werden
	}


	
//HIER DIE BOJE EINLESEN UM SIE ZU BENUTZEN
	feature::Buoy buoy;
	_input_buoy.read(buoy);
	bool buoyfound=false;
	if(buoy.image_radius>=0){
		buoyfound=true;								//#########   ACHTUNG, NOCH BEARBEITEN!!!!!
	}

	/*
     *  =================  Starting state-machine  =================
     */
    switch(current_state)
    {
    case BUOY_SEARCH:
        if (buoyfound) {
            command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor);
            previous_state=current_state;
            current_state=BUOY_DETECTED;
	    	new_state=true;
        }else{
		    command.heading = 0;  //ot.getYaw();
		    command.x = 0;
		    command.y = 0;
		    command.z = (double)_buoy_depth;
		}
    break;
    case BUOY_DETECTED:
        if(buoyfound){
            //check if the auv is in the right place
            double x_diff = sqrt((buoy.world_coord[0]-_max_buoy_distance)*(buoy.world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(buoy.world_coord[1]*buoy.world_coord[1]);
            //double z_diff = sqrt(buoy.world_coord[2]*buoy.world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z){
                previous_state=current_state;
                current_state=BUOY_ARRIVED;
				new_state=true;
            }
            command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor);
        }else{
            previous_state=current_state;
	    	re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
	    	new_state=true;
        }
    break;
    case RE_SEARCHING_BUOY:
        if(buoyfound){
            previous_state=current_state;
	    	new_state=true;
            if(started_cutting) current_state=MOVING_TO_CUTTING_DISTANCE;
	      	else if(strafe_finished_bool & !strafe_to_angle) current_state=STRAFE_FINISHED;
			else current_state=BUOY_DETECTED;
            command=commander.centerBuoy(buoy,ot,_buoy_depth, _maxX, _headingFactor);
        }else{
	    	base::Time time = base::Time::now()-re_search_start;
	    	if(time.toSeconds()>_lost_timeout){  //boje verloren
	        	previous_state=current_state;
				current_state=BUOY_LOST;
				new_state=true;
	    	}else{
	    	    //TODO: hier läßt sich noch eine bessere Lösung finden!!
	    	    base::Time time=base::Time::now()-re_search_start;
				if(time.toSeconds()<_lost_timeout/2){
			    	command.heading=0;
			    	command.x=-_maxX/4;
			    	command.y=0;
			    	command.z=_buoy_depth;
				}
	    	}
        }
    break;
    case BUOY_LOST:
                
    break;    
    case BUOY_ARRIVED:
        new_state=true;
        if(buoyfound){
            double x_diff = sqrt((buoy.world_coord[0]-_max_buoy_distance)*(buoy.world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(buoy.world_coord[1]*buoy.world_coord[1]);
           // double z_diff = sqrt(buoy.world_coord[2]*buoy.world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z){ //wenn die Buoy noch zentral ist
                if(strafed_over_180_degrees && !_strafe_around) command=commander.strafeBuoy(buoy,ot,-_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
				else command=commander.strafeBuoy(buoy,ot,_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);

                if(!started_servoing){
                    started_servoing=true;
                    servoing_rbs=ot;
                }
                //last_command.clear();
                previous_state=current_state;
                if(strafe_to_angle)		//überprüfen ob er in normales strafing geht, oder zu einem bestimmten zielheading strafen soll
					current_state=STRAFE_TO_ANGLE;					
				else
					current_state=STRAFING;
                //wenn mehr als 180° erreicht wurden merke dies
                if(!strafed_over_180_degrees && started_servoing && did180degrees()){
                    strafed_over_180_degrees=true;
                }
                //wenn bereits ein halbkreis gedreht wurde wird zurück gedreht u (überprüft ob...) abgeschlossen
                if(started_servoing && strafed_over_180_degrees){  
                    double x=ot.getYaw()-servoing_rbs.getYaw();
                    if(x>M_PI) x-=2*M_PI;
                    if(x<-M_PI) x+=2*M_PI;
                    if(x<winkelspiel && x>-winkelspiel){//überprüfen ob er zurück am ursprungsort ist und ob er endlos strafen soll oder nicht
                        strafed_over_180_degrees=false;
                        started_servoing=false;
                        previous_state=current_state;
                        current_state=STRAFE_FINISHED;
                    }
                }
            }else{
                previous_state=current_state;
                current_state=BUOY_DETECTED;
                command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor);
            }
        }else{
            previous_state=current_state;
	    	re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
        }
    break;
    case STRAFING:
        if(buoyfound){
            double x_diff = sqrt((buoy.world_coord[0]-_max_buoy_distance)*(buoy.world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(buoy.world_coord[1]*buoy.world_coord[1]);
           // double z_diff = sqrt(buoy.world_coord[2]*buoy.world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z*1.5 && !strafe_to_angle){ //wenn die Buoy noch zentral ist und nicht zwischenzeitig ein befehl eingegangen ist, wo hin gestrafed werden soll
				if(strafed_over_180_degrees && !_strafe_around) command=commander.strafeBuoy(buoy,ot,-_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
				else command=commander.strafeBuoy(buoy,ot,_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
				//wenn mehr als 180° erreicht wurden merke dies
                if(!strafed_over_180_degrees && started_servoing && did180degrees()){
                    strafed_over_180_degrees=true;
                }
                //wenn bereits ein halbkreis gedreht wurde wird zurück gedreht u abgeschlossen
                if(started_servoing && strafed_over_180_degrees){  
                    double x=ot.getYaw()-servoing_rbs.getYaw();
                    if(x>M_PI) x-=2*M_PI;
                    if(x<-M_PI) x+=2*M_PI;
                    if(x<winkelspiel && x>-winkelspiel){
                        strafed_over_180_degrees=false;
                        started_servoing=false;
                        previous_state=current_state;
                        current_state=STRAFE_FINISHED;
                    }
                }
		//======================================================================	
            }else{
                previous_state=current_state;
                current_state=BUOY_DETECTED;
				new_state=true;
                command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor );
            }
        }else{
            previous_state=current_state;
		    re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
		    new_state=true;
        }
    break;
    case STRAFE_FINISHED:
		strafe_finished_bool=true;
        started_servoing = false;
        started_cutting = true;
        strafed_over_180_degrees = false;
        previous_state=current_state;
		new_state=true;
        if(buoyfound){
//            current_state=MOVING_TO_CUTTING_DISTANCE;
//            command=commander.centerBuoy(buoy,ot,_buoy_depth, _maxX, _headingFactor );
			if(strafe_to_angle){		//wenn ein strafe-to-angle-befehl rein gekommen ist, so strafe zu diesem winkel...
				previous_state=current_state;
                current_state=BUOY_DETECTED;
				new_state=true;
                command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor );
			} else {
				command=commander.centerBuoyHeadingFixed(buoy, ot, _buoy_depth, _maxX, servoing_rbs.getYaw(), _headingFactor);
			}
        }else{
		    re_search_start=base::Time::now();
		    current_state=RE_SEARCHING_BUOY;
		}    
    break;
    case MOVING_TO_CUTTING_DISTANCE:
        if(buoyfound){
	    	if(new_cutting){
	    	    mov_to_cut_time = base::Time::now();
				new_cutting=false;
	    	}
	    	base::Time time = base::Time::now()-mov_to_cut_time;
	    	int seconds = time.toSeconds();
            double x_diff = sqrt((buoy.world_coord[0]-_max_buoy_distance)*(buoy.world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(buoy.world_coord[1]*buoy.world_coord[1]);
            double z_diff = buoy.world_coord[2];
	//wenn die Buoy noch zentral ist und die Zeit abgelaufen ist
	    	cout << "moving_to_cutting_distance - Zeit: " << seconds;
            if(x_diff<_good_x*0.7 && y_diff<_good_y_z*0.5 && z_diff>0 && seconds>_cutting_wait_time){ 
				base::Pose p = ot.getPose();
				buoy_depth = p.position[2];
                command=commander.cutBuoy(buoy,ot,buoy_depth, _cutting_hight);
				previous_state=current_state;
				cutting_start=base::Time::now();
				current_state=CUTTING;
				new_state=true;
	    	}else{
	    	    command=commander.centerBuoy(buoy,ot,_buoy_depth, _maxX, _headingFactor);
				//Lieber ein bischen tiefer halten als die Boje eigentlich ist
				command.z-=0.1;
	    	}
		}else{
			new_cutting=true;
			previous_state=current_state;
			re_search_start=base::Time::now();
			current_state=RE_SEARCHING_BUOY;
			new_state=true;
		}
    break;
    case CUTTING:
        command = cutting(buoy, buoyfound);
    break;
    case CUTTING_SUCCESS:
		
    break;
	case STRAFE_TO_ANGLE:
		if(buoyfound){
            double x_diff = sqrt((buoy.world_coord[0]-_max_buoy_distance)*(buoy.world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(buoy.world_coord[1]*buoy.world_coord[1]);
			//strafe-richtung bestimmen
			double heading = ot.getYaw();
			double heading_diff = target_heading-heading;
			if(heading_diff<-M_PI)
				heading_diff+=2*M_PI;
			if(heading_diff>M_PI)
				heading_diff-=2*M_PI;
			double strafe_int = 0;
			if(heading_diff<0){		//wenn kleiner, dann nach links strafen, das führt zu drehen nach rechts
				strafe_int = _strafe_intensity * _strafe_intensity;
			} else{					//wenn größer 0 is, dann nach rechts strafen, das führt zu drehen nach links
				strafe_int = -1 * (_strafe_intensity * _strafe_intensity);
			}
			
            if(x_diff<_good_x && y_diff<_good_y_z*1.5){ //wenn die Buoy noch zentral ist
				
				if(heading_diff<winkelspiel && heading_diff>-winkelspiel)	//wenn das target-heading bereits erreicht wurde zentriere die boje mit fixem heading
				{
					angle_arrived=true;
					previous_state=current_state;
					current_state=ANGLE_ARRIVED;
					new_state=true;
				}
				if(!angle_arrived){					
					command=commander.strafeBuoy(buoy,ot,strafe_int, _buoy_depth, _headingFactor, _headingModulation);
				}
            }else{
                previous_state=current_state;
                current_state=BUOY_DETECTED;
				new_state=true;
                command=commander.centerBuoy(buoy,ot, _buoy_depth, _maxX, _headingFactor );
            }
        }else{
            previous_state=current_state;
		    re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
		    new_state=true;
        }
	break;
	case ANGLE_ARRIVED:
		if(buoyfound){
			command=commander.centerBuoyHeadingFixed(buoy, ot, _buoy_depth, _maxX, target_heading, _headingFactor);
		} else {
			previous_state=current_state;
		    re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
		    new_state=true;
		}
	break;
    default:
        current_state=BUOY_SEARCH;

    }
    //if there is a new state, write it out
    if(new_state){
        state(current_state);
    }
    new_state=false;
	
	//command-z_offset dazu addieren
	command.z+=_z_offset;
    //if(buoyfound) last_command.push_back(command);;
        _relative_position.write(command);
}


// void Survey::errorHook()
// {
//     SurveyBase::errorHook();
// }
// void Survey::stopHook()
// {
//     SurveyBase::stopHook();
// }
void Survey::cleanupHook()
{
	SurveyBase::cleanupHook();
}



bool Survey::did180degrees()
{
    double start=servoing_rbs.getYaw();
    double now=ot.getYaw();
    double div=now-start;
    _strafed_angle.write(div);	//schreiben des zurückgelegten winkels auf den output-port
    if(div>M_PI) div-=2*M_PI;
    if(div<-M_PI) div+=2*M_PI;
    if(div>M_PI*_strafe_angle || div<-M_PI*_strafe_angle) return true;
    return false;
}


base::AUVPositionCommand Survey::buoy_search(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::buoy_detected(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::researching_buoy(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::buoy_lost(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::buoy_arrived(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::strafing(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::strafe_finished(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::moving_to_cutting_distance(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
base::AUVPositionCommand Survey::cutting(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	if(buoyfound){
		command=commander.cutBuoy(buoy,ot,buoy_depth, _cutting_hight);
	   	base::Time time=base::Time::now()-cutting_start;
	   	if(time.toSeconds()>_cutting_time){    //wenn die vergangene zeit einer Property entspricht
            previous_state=current_state;
			current_state=CUTTING_SUCCESS;
			new_state=true;
	    }
	}else{
	   	command=commander.cutBuoy(ot,buoy_depth, _cutting_hight);
	   	base::Time time=base::Time::now()-cutting_start;
	   	if(time.toSeconds()>_cutting_time){    //wenn die vergangene zeit einer Property entspricht
            previous_state=current_state;
			current_state=CUTTING_SUCCESS;
			new_state=true;
	   	}
	}
	return command;
}
base::AUVPositionCommand Survey::cutting_success(feature::Buoy buoy, bool buoyfound){
	base::AUVPositionCommand command;
	return command;
}
