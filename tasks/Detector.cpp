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
bool debug_gui = true;

using namespace avalon;
using namespace buoy;
frame_helper::FrameHelper frameHelper;


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

    previous_state=NO_BUOY_FOUND;
    current_state=NO_BUOY_FOUND;
	state(current_state);

    houghTh = _houghTh;
	edgeTh = _edgeTh;
	hValue= _hValue;
	sValue= _sValue;
	vValue= _vValue;
	edgeTh = _edgeTh;
    
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
    }else{
		filter.setMaxage((double)_filter_timeout);
	}

	BuoyFeatureVector result = detector.buoyDetection(&image, hValue, sValue, _debug_gui);
	filter.feed(result);

	if(_debug_gui){
        for(unsigned int i=0;i<result.size();i++)
        {
            BuoyDetector::draw(&image, result[i], CV_RGB(0,0,255));
        }
	}

    BuoyFeatureVector vector = filter.process();
		//wenn die boje gefunden wurde schreibe sie raus
    if (vector.size() > 0 && vector.front().validation>-1) {
        feature::Buoy& buoy=vector.front();
        //vector.front()=buoy;
        if(_debug_gui){
            BuoyDetector::draw(&image, buoy,CV_RGB(255, 0, 0));
        }
        _buoy.write(buoy);
		current_state = BUOY_FOUND;
    }else { //wenn keine boje gefunden wurde, schreibe eine neue mit negativem Radius
		feature::Buoy* buoy = new feature::Buoy(0,0,-1);
		_buoy.write(*buoy);
		current_state = NO_BUOY_FOUND;
	}

	if(current_state!=previous_state){
		state(current_state);
		previous_state=current_state;
	}
}

void Detector::cleanupHook() {
	DetectorBase::cleanupHook();

    if(_debug_gui){
		cvDestroyWindow(GUI_WINDOW_NAME);
    }
}

