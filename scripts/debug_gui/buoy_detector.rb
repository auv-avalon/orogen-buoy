require 'vizkit'


class BuoyDetector
  RADTODEG = 180.0/Math::PI
  Bla = "0"
  #float.round(d) is not available for ruby 1.8
  def round(n)
      (n*10**4).round.to_f/10**4
  end

  def initialize(main_frame_port, s_frame_port, h_frame_port,buoy_task)
    @buoy_task = buoy_task
    @frame_port = main_frame_port
	@s_frame_port = s_frame_port
	@h_frame_port = h_frame_port

    @window = Vizkit.load(File.join(File.dirname(__FILE__),"buoy_detector.ui"))
    @frame_port.connect_to @window.main_image, :type=>:buffer,:size=>1
	@s_frame_port.connect_to @window.s_image, :type=>:buffer,:size=>1
	@h_frame_port.connect_to @window.h_image, :type=>:buffer,:size=>1
    @buoy_task.buoy.connect_to self.method(:display)
	@buoy_task.other_buoys.connect_to self.method(:display_debug)
	@buoy_task.light.connect_to self.method(:display_light)

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
	@window.hValue.setValue(@buoy_task.hValue)
	@window.hValue.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.hValue = value
    end
	@window.sValue.setValue(@buoy_task.sValue)
	@window.sValue.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.sValue = value
    end
	@window.buoy_radius.setValue(@buoy_task.buoy_radius)
	@window.buoy_radius.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.buoy_radius = value
    end
##
## Licht
##


	@window.roi_x.setValue(@buoy_task.roi_x)
	@window.roi_x.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.roi_x = value
    end
	@window.roi_y.setValue(@buoy_task.roi_y)
	@window.roi_y.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.roi_y = value
    end
	@window.roi_width.setValue(@buoy_task.roi_width)
	@window.roi_width.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.roi_width = value
    end
	@window.roi_height.setValue(@buoy_task.roi_height)
	@window.roi_height.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.roi_height = value
    end
	@window.val_th.setValue(@buoy_task.val_th)
	@window.val_th.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.val_th = value
    end
	@window.sat_th.setValue(@buoy_task.sat_th)
	@window.sat_th.connect(SIGNAL('valueChanged(double)')) do |value|
		@buoy_task.sat_th = value
    end
##
## Survey
##



    @lineX = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
    @lineX.openGL true
	@lineY = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
	@lineY.openGL true

	#debug-linien
	@lines = Array.new
	#licht-linien
#	@light_top = @window.main_image.addLine(0,0,1,Qt::Color.new(0,0,0),0,0);
#	@light_top.openGL true
#	@light_left = @window.main_image.addLine(0,0,1,Qt::Color.new(0,0,0),0,0);
#	@light_left.openGL true
#	@light_right = @window.main_image.addLine(0,0,1,Qt::Color.new(0,0,0),0,0);
#	@light_right.openGL true
#	@light_buttom = @window.main_image.addLine(0,0,1,Qt::Color.new(0,0,0),0,0);
#	@light_buttom.openGL true


    @pen = Qt::Pen.new
    @pen.setColor(Qt::Color.new(255,0,0))
  end


  def display_light(sample,_)
	@white_light_detected= sample.to_s=="1"
	@window.light_on.setText(sample.to_s)
  end

  def display_debug(sample,_)
	#löschen der alten linien
	@lines.each do |line|
		@window.main_image.removeItem(line, true)
	end
	@lines.clear
    #einzeichnen der debug-samples
	sample.each do |buoy|
		start_x = buoy.image_x-buoy.image_radius
		end_x = buoy.image_x+buoy.image_radius
		start_y = buoy.image_y
		end_y = buoy.image_y
		@lines << @window.main_image.addLine(start_x,start_y,2,Qt::Color.new(0,255,0),end_x,end_y)
		start_x = buoy.image_x
		end_x = buoy.image_x
		start_y = buoy.image_y-buoy.image_radius
		end_y = buoy.image_y+buoy.image_radius
		@lines << @window.main_image.addLine(start_x,start_y,2,Qt::Color.new(0,255,0),end_x,end_y)
	end
  end

  def display(sample,_)
      @window.x.setText(sample.image_x.to_s)
	  @window.y.setText(sample.image_y.to_s)
      @window.r.setText(sample.image_radius.to_s)
      @window.val.setText(sample.validation.to_s)
      @window.world_x.setText(round(sample.world_coord[0]).to_s)
      @window.world_y.setText(round(sample.world_coord[1]).to_s)
      @window.world_z.setText(round(sample.world_coord[2]).to_s)


	if sample.image_radius!=-1
		@lineX.setPosX(sample.image_x-sample.image_radius)
		@lineX.setEndX(sample.image_x+sample.image_radius)
		@lineX.setPosY(sample.image_y)
		@lineX.setEndY(sample.image_y)
		@lineY.setPosX(sample.image_x)
		@lineY.setEndX(sample.image_x)
		@lineY.setPosY(sample.image_y-sample.image_radius)
		@lineY.setEndY(sample.image_y+sample.image_radius)


		roi_x =  @buoy_task.roi_x
		roi_y = @buoy_task.roi_y
		roi_width = @buoy_task.roi_width
		roi_height = @buoy_task.roi_height

		p1_x = sample.image_x-(roi_x * sample.image_radius) - (roi_width*sample.image_radius)/2
		p1_y = sample.image_y- sample.image_radius + (roi_y*sample.image_radius) -(roi_height*sample.image_radius)
		p2_x = p1_x+(roi_width*sample.image_radius)
		p2_y = p1_y
		p3_x = p2_x
		p3_y = p1_y + (roi_height*sample.image_radius)
		p4_x = p1_x
		p4_y = p3_y
		
		cR =255
		cG = 0
		if  @white_light_detected
		cR =0
		cG = 255
		end

		@lines << @window.main_image.addLine(p1_x,p1_y,2,Qt::Color.new(cR,cG,0),p2_x,p2_y)
		@lines << @window.main_image.addLine(p2_x,p2_y,2,Qt::Color.new(cR,cG,0),p3_x,p3_y)
		@lines << @window.main_image.addLine(p3_x,p3_y,2,Qt::Color.new(cR,cG,0),p4_x,p4_y)
		@lines << @window.main_image.addLine(p4_x,p4_y,2,Qt::Color.new(cR,cG,0),p1_x,p1_y)
		

	else
		@lineX.setPosX(0)
		@lineX.setEndX(0)
		@lineX.setPosY(0)
		@lineX.setEndY(0)
		@lineY.setPosX(0)
		@lineY.setEndX(0)
		@lineY.setPosY(0)
		@lineY.setEndY(0)
	end


      @window.main_image.update2
  end

  def show
      @window.show
  end
end

