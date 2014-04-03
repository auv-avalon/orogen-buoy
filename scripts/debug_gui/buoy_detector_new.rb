require 'vizkit'


class BuoyDetector
  RADTODEG = 180.0/Math::PI
  Bla = "0"
  #float.round(d) is not available for ruby 1.8
  def round(n)
      (n*10**4).round.to_f/10**4
  end

  def initialize(camera_front, buoy_task)
#  def initialize()
    @buoy_task = buoy_task
    @camera_front = camera_front

    @window = Vizkit.load(File.join(File.dirname(__FILE__),"buoy_detector_new.ui"))
    
    @camera_front.frame.connect_to @window.image_front_camera, :type=>:buffer,:size=>1
    @buoy_task.binary_debug_image.connect_to @window.image_binary, :type=>:buffer,:size=>1
    @buoy_task.gray_debug_image.connect_to @window.image_grayscale, :type=>:buffer,:size=>1
    @buoy_task.hough_debug_image.connect_to @window.image_debug, :type=>:buffer,:size=>1
#	@h_frame_port.connect_to @window.h_image, :type=>:buffer,:size=>1
#	@buoy_task.light_image.connect_to @window.light_image, :type=>:buffer,:size=>1
#    @buoy_task.buoy.connect_to self.method(:display)
#	@buoy_task.other_buoys.connect_to self.method(:display_debug)
#	@buoy_task.light.connect_to self.method(:display_light)

    refresh = Qt::Timer.new
    refresh.connect(SIGNAL('timeout()')) do 
        if @buoy_task.reachable?
    	   #@window.SpinBoxSpeed.setValue(@pipeline_task.default_x)
           #@window.SpinBoxColorChannel.setValue(@pipeline_task.use_channel)
    	   #@window.SpinBoxDepth.setValue(@pipeline_task.depth)
        end
    end
    refresh.start(2000)
##
## Detektion
##
    @window.hValueMinSpinBox.setValue(@buoy_task.hValueMin)
    @window.hValueMinSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
        @buoy_task.hValueMin = value
    end
    @window.hValueMaxSpinBox.setValue(@buoy_task.hValueMax)
    @window.hValueMaxSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
        @buoy_task.hValueMax = value
    end
    @window.sValueMinSpinBox.setValue(@buoy_task.sValueMin)
    @window.sValueMinSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
        @buoy_task.sValueMin = value
    end
    @window.sValueMaxSpinBox.setValue(@buoy_task.sValueMax)
    @window.sValueMaxSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
        @buoy_task.sValueMax = value
    end
    @window.vValueMinSpinBox.setValue(@buoy_task.vValueMin)
    @window.vValueMinSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
        @buoy_task.vValueMin = value
    end
    @window.vValueMaxSpinBox.setValue(@buoy_task.vValueMax)
    @window.vValueMaxSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.vValueMax = value
    end
    @window.hSmoothSpinBox.setValue(@buoy_task.hSmooth)
    @window.hSmoothSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.hSmooth = value
    end
    @window.sSmoothSpinBox.setValue(@buoy_task.sSmooth)
    @window.sSmoothSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.sSmooth = value
    end
    @window.vSmoothSpinBox.setValue(@buoy_task.vSmooth)
    @window.vSmoothSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.vSmooth = value
    end
    @window.hEdgeThresholdSpinBox.setValue(@buoy_task.hHoughEdgeThreshold)
    @window.hEdgeThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.hHoughEdgeThreshold = value
    end
    @window.sEdgeThresholdSpinBox.setValue(@buoy_task.sHoughEdgeThreshold)
    @window.sEdgeThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.hHoughEdgeThreshold = value
    end
    @window.vEdgeThresholdSpinBox.setValue(@buoy_task.vHoughEdgeThreshold)
    @window.vEdgeThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.hHoughEdgeThreshold = value
    end
    @window.hAccuThresholdSpinBox.setValue(@buoy_task.hHoughAccumulatorThreshold)
    @window.hAccuThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.hHoughAccumulatorThreshold = value
    end
    @window.sAccuThresholdSpinBox.setValue(@buoy_task.sHoughAccumulatorThreshold)
    @window.sAccuThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.sHoughAccumulatorThreshold = value
    end
    @window.vAccuThresholdSpinBox.setValue(@buoy_task.vHoughAccumulatorThreshold)
    @window.vAccuThresholdSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.vHoughAccumulatorThreshold = value
    end
    @window.houghMinCircleSpinBox.setValue(@buoy_task.houghMinCircle)
    @window.houghMinCircleSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.houghMinCircle = value
    end
    @window.houghMaxCircleSpinBox.setValue(@buoy_task.houghMaxCircle)
    @window.houghMaxCircleSpinBox.connect(SIGNAL('valueChanged(int)')) do |value|
       @buoy_task.houghMaxCircle = value
    end
    @window.hDebugCheckBox.setChecked(@buoy_task.hough_debug_h)
    @window.hDebugCheckBox.connect(SIGNAL('clicked(bool)')) do |value|
       @buoy_task.hough_debug_h = value
    end
    @window.sDebugCheckBox.setChecked(@buoy_task.hough_debug_s)
    @window.sDebugCheckBox.connect(SIGNAL('clicked(bool)')) do |value|
       @buoy_task.hough_debug_s = value
    end
    @window.vDebugCheckBox.setChecked(@buoy_task.hough_debug_v)
    @window.vDebugCheckBox.connect(SIGNAL('clicked(bool)')) do |value|
       @buoy_task.hough_debug_v = value
    end
    @window.debugTypeComboBox.connect(SIGNAL('currentIndexChanged(int)')) do |value|
       @buoy_task.hsv_gray = value
    end
  end

  def show
      @window.show
  end
end

