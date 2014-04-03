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
    std::cout <<"init" << std::endl;
//	h_frame = new base::samples::frame::Frame();
//	s_frame = new base::samples::frame::Frame();
//	h_frame.init(0,0,8,base::samples::frame::MODE_GRAYSCALE);
//	s_frame.init(0,0,8,base::samples::frame::MODE_GRAYSCALE);
	hframe.reset(new base::samples::frame::Frame());
	sframe.reset(new base::samples::frame::Frame());
	vframe.reset(new base::samples::frame::Frame());
	grayframe.reset(new base::samples::frame::Frame());
	houghframe.reset(new base::samples::frame::Frame());
	debugframe.reset(new base::samples::frame::Frame());
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
    std::cout <<"configure" << std::endl;
     if (! DetectorBase::configureHook())
         return false;
     return true;
}
bool Detector::startHook()
{
    std::cout <<"start" << std::endl;
    if (!DetectorBase::startHook()){
   		std::cout<<"startHook: FALSE"<<std::endl;
		return false;
	}else{
		std::cout<<"startHook: TRUE"<<std::endl;
	}
   	double r = _buoy_radius;
   	posestimator = BuoyPosEstimator();

    previous_state=NO_BUOY_FOUND;
    current_state=NO_BUOY_FOUND;
	state(current_state);


	filter.setBufferSize(buoys_buffer_size);
    filter.setMinSize(buoys_buffer_size_min);
    filter.setStartval(startvalidation);
    filter.setMindist(mindist);
    filter.setMaxage((double)_filter_timeout,true);

	return true;
}


void Detector::updateHook()
{
    std::cout <<"update" << std::endl;
    std::cout <<"configure" << std::endl;
	detector.configureHoughAccumulatorThresholdH(_hHoughAccumulatorThreshold.get());
	detector.configureHoughAccumulatorThresholdS(_sHoughAccumulatorThreshold.get());
	detector.configureHoughAccumulatorThresholdV(_vHoughAccumulatorThreshold.get());
	detector.configureHoughEdgeThresholdH(_hHoughEdgeThreshold.get());
	detector.configureHoughEdgeThresholdS(_sHoughEdgeThreshold.get());
	detector.configureHoughEdgeThresholdV(_vHoughEdgeThreshold.get());

	detector.configureHoughCircleMin(_houghMinCircle.get());
	detector.configureHoughCircleMax(_houghMaxCircle.get());

        detector.configureHValueMin(_hValueMin.get());
        detector.configureSValueMin(_sValueMin.get());
        detector.configureVValueMin(_vValueMin.get());

        detector.configureHValueMax(_hValueMax.get());
        detector.configureSValueMax(_sValueMax.get());
        detector.configureVValueMax(_vValueMax.get());

        //if a smooth value % 2 = 0 you get an ocv error so we catch it hear.
        if(!(_hSmooth.get() % 2)){
            std::cout << "Gerader Wert für hSmooth!" << std::endl;
            _hSmooth.set(_hSmooth.get()+1);
        }
        if(! (_sSmooth.get() % 2)){
            _sSmooth.set(_sSmooth.get()+1);
        }
        if(! (_vSmooth.get() % 2)){
            _vSmooth.set(_vSmooth.get()+1);
        }

        detector.configureHSmooth(_hSmooth.get());
        detector.configureSSmooth(_sSmooth.get());
        detector.configureVSmooth(_vSmooth.get());
        
        detector.configureDebug(_debug.get());
        detector.configureDebugGray(_hsv_gray.get());
        detector.configureDebugHough(_hough_debug_h.get(), _hough_debug_s.get(), _hough_debug_v.get());
	DetectorBase::updateHook();
        if (_frame.read(fp) != RTT::NewData) {
		return;
	}

    //if incoming image is nocht RGB change it to RGB. this is
    //because of different image-Formats between Simulation and Hardware-Cam
    if( !fp->isRGB()) {
	frame.init(fp->getWidth(),fp->getHeight(),8,base::samples::frame::MODE_RGB);
        frameHelper.convertColor(*fp, frame);
        image = IplImage(frame_helper::FrameHelper::convertToCvMat(frame));
    } else {
        frame.init(*fp,true);
        image = IplImage(frame_helper::FrameHelper::convertToCvMat(*fp));
    }

	filter.setMaxage((double)_filter_timeout);

	BuoyFeatureVector result = detector.buoyDetection(&image);
        std::cout << "H-Value-Min: " << _hValueMin.get() << "          S-Value-Min: " << _sValueMin.get() << std::endl;
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
		//light_on = detector.findWhiteLight(&image, buoy, feature::WhiteLightSettings(_roi_x,_roi_y,_roi_width,_roi_height,_val_th,_sat_th));
                light_on = false;
    }
	_buoy.write(buoy);
	_light.write(light_on);

	if(current_state!=previous_state){
		state(current_state);
		previous_state=current_state;
	}

	if(_debug.get()){
    std::cout <<"writeout" << std::endl;
    std::cout << "1" << std::endl;
		base::samples::frame::Frame* h_p = hframe.write_access();
		base::samples::frame::Frame* s_p = sframe.write_access();
		base::samples::frame::Frame* v_p = vframe.write_access();
		base::samples::frame::Frame* hsv_debug_p = grayframe.write_access();
		base::samples::frame::Frame* hough_debug_p = houghframe.write_access();
		base::samples::frame::Frame* debug_p = debugframe.write_access();

    std::cout << "2" << std::endl;
    std::cout << "2.1" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getHplane(),*h_p);
    std::cout << "2.2" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getSplane(),*s_p);
    std::cout << "2.3" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getVplane(),*v_p);
    std::cout << "2.4" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getHSVDebug(),*hsv_debug_p);
    std::cout << "2.5" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getHoughDebug(),*hough_debug_p);
    std::cout << "2.6" << std::endl;
		frame_helper::FrameHelper::copyMatToFrame(detector.getDebugImage(),*debug_p);

    std::cout << "3" << std::endl;
		hframe.reset(h_p);
		sframe.reset(s_p);
                vframe.reset(v_p);
                grayframe.reset(hsv_debug_p);
                houghframe.reset(hough_debug_p);
		debugframe.reset(debug_p);

    std::cout << "4" << std::endl;
		_h_image.write(hframe);
		_s_image.write(sframe);
                _v_image.write(vframe);
                if(_hsv_gray.get() == 0){
                    _binary_debug_image.write(hframe);
                } else if (_hsv_gray.get() == 1){
                    _binary_debug_image.write(sframe);
                }else{
                    _binary_debug_image.write(vframe);
                }
                _debug_image.write(debugframe);
                _gray_debug_image.write(grayframe);
                _hough_debug_image.write(houghframe);
		_other_buoys.write(vector);
	}
}

void Detector::cleanupHook() {
	DetectorBase::cleanupHook();
}

