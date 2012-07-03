require 'vizkit'


class BuoyDetector
  RADTODEG = 180.0/Math::PI

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

    refresh = Qt::Timer.new
    refresh.connect(SIGNAL('timeout()')) do 
        if @buoy_task.reachable?
    	   #@window.SpinBoxSpeed.setValue(@pipeline_task.default_x)
           #@window.SpinBoxColorChannel.setValue(@pipeline_task.use_channel)
    	   #@window.SpinBoxDepth.setValue(@pipeline_task.depth)
        end
    end
    refresh.start(2000)

	@window.hValue.connect(SIGNAL('valueChanged(double)')) do |value|
		#@buoy_task.hValue = value
        #@pipeline_task.forceAngle(value/180*Math::PI)
    end

    @lineX = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
    @lineX.openGL true
	@lineY = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
	@lineY.openGL true

	#debug-linien
	@lines = Array.new
#	@lineX1 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#    @lineX1.openGL true
#	@lineY1 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#	@lineY1.openGL true
#	@lineX2 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#    @lineX2.openGL true
#	@lineY2 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#	@lineY2.openGL true
#	@lineX3 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#    @lineX3.openGL true
#	@lineY3 = @window.main_image.addLine(0,0,1,Qt::Color.new(255,255,0),0,0);
#	@lineY3.openGL true

    @pen = Qt::Pen.new
    @pen.setColor(Qt::Color.new(255,0,0))
  end

  def display_debug(sample,_)
	#löschen der alten linien
	@lines.each do |line|
		@window.main_image.removeItem(line, true)
	end
	@lines.clear
    #einzeichnen der debug-samples
	sample.each do |buoy|
		puts "ein each..."
#		start_x = buoy.image_x-buoy.image_radius
#		end_x = buoy.image_x+bouy.image_radius
#		start_y = buoy.image_y
#		end_y = buoy_image_y
#		@lines.add(@window.main_image.addLine(start_x,start_y,2,Qt::Color.new(0,255,0),end_x,end_y))
#		start_x = buoy.image_x
#		end_x = buoy.image_x
#		start_y = buoy.image_y-buoy.image_radius
#		end_y = buoy_image_y+bouy.image_radius
#		@lines.add(@window.main_image.addLine(start_x,start_y,2,Qt::Color.new(0,255,0),end_x,end_y))
	end
  end

  def display(sample,_)
      @window.x.setText(sample.image_x.to_s)
	  @window.y.setText(sample.image_y.to_s)
      @window.r.setText(sample.image_radius.to_s)
      @window.val.setText(sample.validation.to_s)
      @window.world_x.setText(sample.world_coord[0].to_s)
      @window.world_y.setText(sample.world_coord[1].to_s)
      @window.world_z.setText(sample.world_coord[2].to_s)


	if sample.image_radius!=-1
		@lineX.setPosX(sample.image_x-sample.image_radius)
		@lineX.setEndX(sample.image_x+sample.image_radius)
		@lineX.setPosY(sample.image_y)
		@lineX.setEndY(sample.image_y)
		@lineY.setPosX(sample.image_x)
		@lineY.setEndX(sample.image_x)
		@lineY.setPosY(sample.image_y-sample.image_radius)
		@lineY.setEndY(sample.image_y+sample.image_radius)
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

