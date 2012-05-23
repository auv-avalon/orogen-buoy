/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Detector.hpp"
#include "base/samples/frame.h"
#include "frame_helper/FrameHelper.h"
//#include "visual_detectors/buoy_pos_estimation.h"
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <iostream>

#define GUI_WINDOW_NAME "Buoy Monitor"

int houghTh = 0, edgeTh = 0;
int buoys_buffer_size=5,buoys_buffer_size_min=3,startvalidation=100,mindist=100,maxage=150;
int vValue = 0;
int hValue = 0;
int sValue = 0;
int buoy_depth = 0;
bool debug_gui = true;
bool new_cutting = true;
base::Time mov_to_cut_time;
double good_heading=10, good_distance=3.0, good_depth=0.2;
int sum[5], counter=0;

using namespace avalon;
using namespace buoy;
frame_helper::FrameHelper frameHelper;


Detector::Detector(std::string const& name, TaskCore::TaskState initial_state)
    : DetectorBase(name, initial_state)
{
	started_servoing=false;
}

Detector::Detector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DetectorBase(name, engine, initial_state)
{
	started_servoing=false;
}

Detector::~Detector()
{
	started_servoing=false;
}

bool Detector::did180degrees()
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

void on_change_Hvalue(int th){
	hValue=th;
}
void on_change_Vvalue(int th){
	vValue=th;
}
void on_change_Svalue(int th){
	sValue=th;
}
void on_change_hough_th(int th) {
	houghTh = th;
}
void on_change_edge_th(int th) {
	edgeTh = th;
}

void on_change_bufferSize(int th) {
	buoys_buffer_size = th;
}
void on_change_bufferSizeMin(int th) {
	buoys_buffer_size_min = th;
}
void on_change_startVal(int th) {
	startvalidation = th;
} 
void on_change_mindist(int th) {
	mindist = th;
}
void on_change_maxage(int th) {
	maxage = th;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Detector.hpp for more detailed
// documentation about them.

bool Detector::configureHook()
{
     if (! DetectorBase::configureHook())
         return false;
     return true;
}
bool Detector::startHook()
{
     	if (!DetectorBase::startHook())
		return false;
        double r = _buoy_radius;
        posestimator = BuoyPosEstimator(r);
        commander = CommandCreator(_max_buoy_distance.get());

        previous_state=BUOY_SEARCH;
        current_state=BUOY_SEARCH;

        started_servoing=false;
        started_cutting=false;
        strafed_over_180_degrees=false;
	new_state=true;
	buoy_depth = _buoy_depth;
        

	houghTh = _houghTh;
	edgeTh = _edgeTh;
	hValue= _hValue;
	sValue= _sValue;
	vValue= _vValue;
	edgeTh = _edgeTh;
	debug_gui = _debug_gui;
//	detector.configureLowHue(_hue_low);
//	detector.configureHighHue(_hue_high);
	detector.configureHoughThreshold(houghTh);
//	detector.configureEdgeThreshold(edgeTh);servoing_rbs
	filter.setBufferSize(buoys_buffer_size);
        filter.setMinSize(buoys_buffer_size_min);
        filter.setStartval(startvalidation);
        filter.setMindist(mindist);
        filter.setMaxage(maxage,true);

if(_debug_gui){
	cvNamedWindow(GUI_WINDOW_NAME, CV_WINDOW_NORMAL);
	cvResizeWindow(GUI_WINDOW_NAME, 800, 600);
	cvNamedWindow("Trackbars",CV_WINDOW_NORMAL);


	cvCreateTrackbar("Hue Threshold", "Trackbars", &hValue, 255, on_change_Hvalue);
	cvCreateTrackbar("Saturation Threshold", "Trackbars", &sValue, 255, on_change_Svalue);
	cvCreateTrackbar("Hough Threshold", "Trackbars", &houghTh, 256, on_change_hough_th);
	cvCreateTrackbar("Sobel Threshold", "Trackbars", &edgeTh, 256, on_change_edge_th);

	//cvCreateTrackbar("bufferSize", "Trackbars", &buoys_buffer_size, 30, on_change_bufferSize);
	//cvCreateTrackbar("bufferSizeMin", "Trackbars", &buoys_buffer_size_min, 15, on_change_bufferSizeMin);
	//cvCreateTrackbar("startValidation", "Trackbars", &startvalidation, 200, on_change_startVal);
	//cvCreateTrackbar("mindist", "Trackbars", &mindist, 300, on_change_mindist);
	cvCreateTrackbar("maxage in 10ms", "Trackbars", &maxage, 500, on_change_maxage);

}
	return true;
}


void Detector::updateHook()
{
	DetectorBase::updateHook();
	if (_frame.read(fp) != RTT::NewData) {
		return;
	}

    //if incoming image is nocht RGB change it to RGB. this is
    //because of different image-Formats between Simulation and Hardware-Cam
    if( !fp->isRGB()) {
	frame.init(fp->getWidth(),fp->getHeight(),8,base::samples::frame::MODE_RGB);
        frameHelper.convertColor(*fp, frame);
        image = IplImage(frame.convertToCvMat());
    } else {
        frame.init(*fp,true);
        image = IplImage(fp->convertToCvMat());
    }
	bool testMode = false;

    if(_debug_gui){
	detector.configureHoughThreshold(houghTh);
	detector.configureEdgeThreshold(edgeTh);

	filter.setBufferSize(buoys_buffer_size);
        filter.setMinSize(buoys_buffer_size_min);
        filter.setStartval(startvalidation);
        filter.setMindist(mindist);
        filter.setMaxage(maxage,true);
	testMode = true;
    }else{filter.setMaxage((double)_filter_timeout);}

	int temp[5];
	if(counter<4){
		counter++;
	}
	int sumValue=0;
	for(int i=1; i<=counter && i< 5;i++){
		temp[i-1] = sum[i];
		sumValue+=sum[i];
	}
	sumValue = sumValue/counter;
	BuoyFeatureVector result = detector.buoyDetection(&image, hValue, sValue, _debug_gui);
	filter.feed(result);

if(_debug_gui){
        for(unsigned int i=0;i<result.size();i++)
        {
            BuoyDetector::draw(&image, result[i], CV_RGB(0,0,255));
        }
}

feature::Buoy ground_truth_buoy;
_orientation_samples.read(ot);
// GROUND TRUTH BEGINS
double d;
double angle_from_Cam;
if(_run_in_simulation){

    base::Position buoy_pos;
    buoy_pos[0] = 40.0;
    buoy_pos[1] = 5;
    buoy_pos[2] = -7.5;
    
    base::Vector3d auv2buoy = buoy_pos - ot.position;
    double buoy_yaw = ot.getYaw()-atan2(auv2buoy[1], auv2buoy[0]);
    double buoy_pitch = -atan2(auv2buoy[2], auv2buoy[0]);
   
    ground_truth_buoy.image_x = 128*(1+buoy_yaw/M_PI*6.0);
    ground_truth_buoy.image_y = 128*(1+buoy_pitch/M_PI*9.0);
    ground_truth_buoy.image_radius = 90.0/pow(auv2buoy.norm(),1.3);                              
   
    if(ground_truth_buoy.image_x < 0)   ground_truth_buoy.image_x = 0;
    if(ground_truth_buoy.image_x > 255) ground_truth_buoy.image_x = 255;
    
    if(ground_truth_buoy.image_y < 0)   ground_truth_buoy.image_y = 0;
    if(ground_truth_buoy.image_y > 255) ground_truth_buoy.image_y = 255;
      
    if(ground_truth_buoy.image_radius < 2)   ground_truth_buoy.image_radius = 2;
    if(ground_truth_buoy.image_radius > 100) ground_truth_buoy.image_radius = 100;

  
    double x = buoy_pos[0] - ot.position[0];
    double y = buoy_pos[1] - ot.position[1];
    double angle = -ot.getYaw();
    ground_truth_buoy.world_coord[0] = x * cos(angle) - y * sin(angle);
    ground_truth_buoy.world_coord[1] = x * sin(angle) + y * cos(angle) ;
    ground_truth_buoy.world_coord[2] = buoy_pos[2] - ot.position[2];
    d = sqrt(ground_truth_buoy.world_coord[0]*ground_truth_buoy.world_coord[0] + ground_truth_buoy.world_coord[1]*ground_truth_buoy.world_coord[1] + ground_truth_buoy.world_coord[2]*ground_truth_buoy.world_coord[2]);
    angle_from_Cam = acos(ground_truth_buoy.world_coord[0] / d);
}
    //buoy at 40m, 0, -1.5m with diameter of 30cm

// GROUND TRUTH ENDS

    BuoyFeatureVector vector = filter.process();
//        BuoyFeatureVector vector = result;
    _orientation_samples.read(ot);
    bool buoyfound=false;

    if(_run_in_simulation){
        vector.clear();
        if(d < 5.0 && angle_from_Cam < 40.0)
        {
        vector.push_back(ground_truth_buoy);         
        }
    }
    if (vector.size() > 0 && vector.front().validation>-1) {
        feature::Buoy& buoy=vector.front();
        if(!_run_in_simulation) posestimator.estimateAuvKoordinates(buoy, frame);
        buoyfound=true;
        vector.front()=buoy;
        if(_debug_gui){
            BuoyDetector::draw(&image, buoy,CV_RGB(255, 0, 0));
        }
        _buoy.write(buoy);
    }
    base::AUVPositionCommand command;
    command.heading=command.x=command.y=command.z=0;
    
    //if the property is true, change to STRAFE_FINISHED-state
	bool force=false;
    if(_force_cutting.read(force) == RTT::NewData && force){
        strafed_over_180_degrees=false;
        started_servoing=false;
        previous_state=current_state;
        current_state=STRAFE_FINISHED;
	new_state=true;
    }
    /*
     *  =================  Starting state-machine  =================
     */
    switch(current_state)
    {
    case BUOY_SEARCH:
        if (buoyfound) {
            command=commander.centerBuoy(vector.front(),ot, _buoy_depth, _maxX, _headingFactor);
            previous_state=current_state;
            current_state=BUOY_DETECTED;
	    new_state=true;
        }
	else
	{
	    command.heading = 0;  //ot.getYaw();
	    command.x = 0;
	    command.y = 0;
	    command.z = (double)_buoy_depth;
	}
    break;
    case BUOY_DETECTED:
        if(buoyfound){
            //check if the auv is in the right place
            double x_diff = sqrt((vector.front().world_coord[0]-_max_buoy_distance)*(vector.front().world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(vector.front().world_coord[1]*vector.front().world_coord[1]);
            //double z_diff = sqrt(vector.front().world_coord[2]*vector.front().world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z){
                previous_state=current_state;
                current_state=BUOY_ARRIVED;
		new_state=true;
            }
            command=commander.centerBuoy(vector.front(),ot, _buoy_depth, _maxX, _headingFactor);
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
	      else current_state=BUOY_DETECTED;
            command=commander.centerBuoy(vector.front(),ot,_buoy_depth, _maxX, _headingFactor);
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
            double x_diff = sqrt((vector.front().world_coord[0]-_max_buoy_distance)*(vector.front().world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(vector.front().world_coord[1]*vector.front().world_coord[1]);
           // double z_diff = sqrt(vector.front().world_coord[2]*vector.front().world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z){ //wenn die Buoy noch zentral ist
                if(strafed_over_180_degrees && !_strafe_around) command=commander.strafeBuoy(vector.front(),ot,-_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
		else command=commander.strafeBuoy(vector.front(),ot,_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);

                if(!started_servoing){
                    started_servoing=true;
                    servoing_rbs=ot;
                }
                //last_command.clear();
                previous_state=current_state;
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
                    if(x<0.4 && x>-0.4){//überprüfen ob er zurück am ursprungsort ist
                        strafed_over_180_degrees=false;
                        started_servoing=false;
                        previous_state=current_state;
                        current_state=STRAFE_FINISHED;
                    }
                }
            }else{
                previous_state=current_state;
                current_state=BUOY_DETECTED;
                command=commander.centerBuoy(vector.front(),ot, _buoy_depth, _maxX, _headingFactor);
            }
        }else{
            previous_state=current_state;
	    re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
        }
    break;
    case STRAFING:
        if(buoyfound){
            double x_diff = sqrt((vector.front().world_coord[0]-_max_buoy_distance)*(vector.front().world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(vector.front().world_coord[1]*vector.front().world_coord[1]);
           // double z_diff = sqrt(vector.front().world_coord[2]*vector.front().world_coord[2]);
            if(x_diff<_good_x && y_diff<_good_y_z*1.5){ //wenn die Buoy noch zentral ist
		if(strafed_over_180_degrees && !_strafe_around) command=commander.strafeBuoy(vector.front(),ot,-_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
		else command=commander.strafeBuoy(vector.front(),ot,_strafe_intensity, _buoy_depth, _headingFactor, _headingModulation);
		
		//========================================================EXPERIMENTELL
		//wenn mehr als 180° erreicht wurden merke dies
                if(!strafed_over_180_degrees && started_servoing && did180degrees()){
                    strafed_over_180_degrees=true;
                }
                //wenn bereits ein halbkreis gedreht wurde wird zurück gedreht u abgeschlossen
                if(started_servoing && strafed_over_180_degrees){  
                    double x=ot.getYaw()-servoing_rbs.getYaw();
                    if(x>M_PI) x-=2*M_PI;
                    if(x<-M_PI) x+=2*M_PI;
                    if(x<0.4 && x>-0.4){
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
                command=commander.centerBuoy(vector.front(),ot, _buoy_depth, _maxX, _headingFactor );
            }
        }else{
            previous_state=current_state;
	    re_search_start=base::Time::now();
            current_state=RE_SEARCHING_BUOY;
	    new_state=true;
        }
    break;
    case STRAFE_FINISHED:
        started_servoing = false;
        started_cutting = true;
        strafed_over_180_degrees = false;
        previous_state=current_state;
	new_state=true;
        if(buoyfound){
            current_state=MOVING_TO_CUTTING_DISTANCE;
            command=commander.centerBuoy(vector.front(),ot,_buoy_depth, _maxX, _headingFactor );
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
            double x_diff = sqrt((vector.front().world_coord[0]-_max_buoy_distance)*(vector.front().world_coord[0]-_max_buoy_distance));
            double y_diff = sqrt(vector.front().world_coord[1]*vector.front().world_coord[1]);
            double z_diff = vector.front().world_coord[2];
	//wenn die Buoy noch zentral ist und die Zeit abgelaufen ist
	    cout << "moving_to_cutting_distance - Zeit: " << seconds;
            if(x_diff<_good_x*0.7 && y_diff<_good_y_z*0.5 && z_diff>0 && seconds>_cutting_wait_time){ 
		base::Pose p = ot.getPose();
		buoy_depth = p.position[2];
                command=commander.cutBuoy(vector.front(),ot,buoy_depth, _cutting_hight);
		previous_state=current_state;
		cutting_start=base::Time::now();
		current_state=CUTTING;
		new_state=true;
	    }else{
	        command=commander.centerBuoy(vector.front(),ot,_buoy_depth, _maxX, _headingFactor);
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
        if(buoyfound){
	    command=commander.cutBuoy(vector.front(),ot,buoy_depth, _cutting_hight);
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
    break;
    case CUTTING_SUCCESS:

    break;
    default:
        current_state=BUOY_SEARCH;

    }
    //if there is a new state, write it out
    if(new_state){
        state(current_state);
    }
    new_state=false;

    if(_debug_gui){
	cvCvtColor(&image, &image, CV_RGB2BGR);
	char command_string[100];
	if(vector.size()>0) sprintf(command_string, "relBuoyPos-xyz: %f %f %f", vector.front().world_coord[0], vector.front().world_coord[1], vector.front().world_coord[2]);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0, 1, CV_AA);
	cvPutText(&image, command_string, cvPoint(20, image.height-20), &font, cvScalar(255, 255, 255, 0));
	cvShowImage(GUI_WINDOW_NAME, &image);
	cvWaitKey(25);
    }
    //if(buoyfound) last_command.push_back(command);;
        _relative_position.write(command);
}

void Detector::cleanupHook() {
	DetectorBase::cleanupHook();

    if(_debug_gui){
	cvDestroyWindow(GUI_WINDOW_NAME);
    }
}

