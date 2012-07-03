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
int buoys_buffer_size=5,buoys_buffer_size_min=3,startvalidation=100,mindist=100;

using namespace avalon;
using namespace buoy;



Detector::Detector(std::string const& name, TaskCore::TaskState initial_state)
    : DetectorBase(name, initial_state)
{
//	h_frame = new base::samples::frame::Frame();
//	s_frame = new base::samples::frame::Frame();
//	h_frame.init(0,0,8,base::samples::frame::MODE_GRAYSCALE);
//	s_frame.init(0,0,8,base::samples::frame::MODE_GRAYSCALE);
	hframe.reset(new base::samples::frame::Frame());
	sframe.reset(new base::samples::frame::Frame());
}

Detector::Detector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DetectorBase(name, engine, initial_state)
{
//	h_frame = new base::samples::frame::Frame();
//	s_frame = new base::samples::frame::Frame();
}

Detector::~Detector()
{
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
   	posestimator = BuoyPosEstimator();

    previous_state=NO_BUOY_FOUND;
    current_state=NO_BUOY_FOUND;
	state(current_state);
    
	detector.configureHoughThreshold(_houghTh);
//	detector.configureEdgeThreshold(edgeTh);servoing_rbs

	filter.setBufferSize(buoys_buffer_size);
    filter.setMinSize(buoys_buffer_size_min);
    filter.setStartval(startvalidation);
    filter.setMindist(mindist);
    filter.setMaxage((double)_filter_timeout,true);

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

	filter.setMaxage((double)_filter_timeout);

	BuoyFeatureVector result = detector.buoyDetection(&image, _hValue.get(), _sValue.get());
	filter.feed(result);
 	
	BuoyFeatureVector vector = filter.process();

	//licht, boje und state auf initial stellen (boje nicht gefunden)
	bool light_on = false;
	
	feature::Buoy buoy = feature::Buoy(0,0,-1);
	current_state = NO_BUOY_FOUND;
	//wenn die boje gefunden wurde schreibe sie raus
    if (vector.size() > 0 && vector.front().validation>-1) {
        buoy=vector.front();
		posestimator.estimateAuvKoordinates(buoy, frame, _buoy_radius.get());
		current_state = BUOY_FOUND;
		light_on = detector.findWhiteLight(&image, buoy, feature::WhiteLightSettings(_roi_x,_roi_y,_roi_width,_roi_height,_val_th,_sat_th));
    }
	_buoy.write(buoy);
	_light.write(light_on);

	if(current_state!=previous_state){
		state(current_state);
		previous_state=current_state;
	}
	
	if(_debug){
		frame_helper::FrameHelper fh = frame_helper::FrameHelper();
		base::samples::frame::Frame* h_p = hframe.write_access();
		base::samples::frame::Frame* s_p = sframe.write_access();
		frame_helper::FrameHelper::copyMatToFrame(detector.getHshaded(),*h_p);
		frame_helper::FrameHelper::copyMatToFrame(detector.getSplane(),*s_p);
		hframe.reset(h_p);
		sframe.reset(s_p);
		_h_image.write(hframe);
		_s_image.write(sframe);
		_other_buoys.write(vector);
	}
}

void Detector::cleanupHook() {
	DetectorBase::cleanupHook();
}

