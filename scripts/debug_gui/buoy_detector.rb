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

    @window.SpinBoxSpeed.setEnabled false
    @window.SpinBoxColorChannel.setEnabled false
    @window.SpinBoxDepth.setEnabled false

    refresh = Qt::Timer.new
    refresh.connect(SIGNAL('timeout()')) do 
        if @buoy_task.reachable?
    	   #@window.SpinBoxSpeed.setValue(@pipeline_task.default_x)
           #@window.SpinBoxColorChannel.setValue(@pipeline_task.use_channel)
    	   #@window.SpinBoxDepth.setValue(@pipeline_task.depth)
        end
    end
    refresh.start(2000)

#    @line1 = @window.ImageView.addLine(0,0,1,Qt::Color.new(0,0,255),0,0);
#    @line2 = @window.ImageView.addLine(0,0,1,Qt::Color.new(0,0,255),0,0);
#    @line3 = @window.ImageView.addLine(0,0,1,Qt::Color.new(255,0,0),0,0);
#    @line4 = @window.ImageView.addLine(0,0,1,Qt::Color.new(255,0,0),0,0);
#    @line1.openGL true
#    @line2.openGL true
#    @line3.openGL true
#    @line4.openGL true

    @pen = Qt::Pen.new
    @pen.setColor(Qt::Color.new(255,0,0))
  end

#  def display_debug(sample,_)
#      #not working
     # @window.PlotWidget.clearCurveData(1)
#      @window.PlotWidget.clearAll()
#      @window.PlotWidget.registerCurve(2,@pen,"pos",1)

#      values = sample.rays.to_a
#      array_x = 0.upto(values.size()-1).to_a.map{|v|v.to_f}
#      @window.PlotWidget.addPoints(array_x,values,1) # plot column

#      values= [0.0,values.max]
#      array_x = [sample.max_ray_pos.to_f,sample.max_ray_pos.to_f]
#      @window.PlotWidget.addPoints(array_x,values,2)
#  end

  def display(sample,_)
      @window.x.setText(sample.image_x.to_s)
	  @window.y.setText(sample.image_y.to_s)
      @window.r.setText(sample.image_radius.to_s)
#      @window.prop.setText(sample.propability.to_s)
      @window.val.setText(sample.validation.to_s)
      @window.world_x.setText(sample.world_coord[0].to_s)
      @window.world_y.setText(sample.world_coord[1].to_s)
      @window.world_z.setText(sample.world_coord[2].to_s)

#      center_x = @window.ImageView.getWidth()/2
#      center_y = @window.ImageView.getHeight()/2

#      if sample.accepted
#          #update image overlay
#          x = sample.x + sample.width*Math.cos(sample.angle)*0.5 +center_x
#          y = sample.y - sample.width*Math.sin(sample.angle)*0.5 +center_y 
#          @line1.setPosX(x - 640*Math.sin(sample.angle))
#          @line1.setPosY(y - 640*Math.cos(sample.angle))
#          @line1.setEndX(x + 640*Math.sin(sample.angle))
#          @line1.setEndY(y + 640*Math.cos(sample.angle))

#          x = sample.x - sample.width*Math.cos(sample.angle)*0.5 + center_x
#          y = sample.y + sample.width*Math.sin(sample.angle)*0.5 + center_y 
#          @line2.setPosX(x + 640*Math.sin(sample.angle))
#          @line2.setPosY(y + 640*Math.cos(sample.angle))
#          @line2.setEndX(x - 640*Math.sin(sample.angle))
#          @line2.setEndY(y - 640*Math.cos(sample.angle))


 #         @line4.setPosX(sample.x + center_x)
 #         @line4.setPosY(sample.y + center_y)
 #         @line4.setEndX(sample.gap_pos_x + center_x)
 #         @line4.setEndY(sample.gap_pos_y + center_y)

  #        @line3.setPosX(0)
  #        @line3.setPosY(0)
  #        @line3.setEndX(0)
   #       @line3.setEndY(0)
 #     elsif sample.confidence > 0.1
 #         @line1.setPosX(0)
 #         @line1.setPosY(0)
 #         @line1.setEndX(0)
 #         @line1.setEndY(0)
 #         @line2.setPosX(0)
 #         @line2.setPosY(0)
 #         @line2.setEndX(0)
 #         @line2.setEndY(0)

 #         @line3.setPosX(x - 640*Math.sin(sample.angle))
 #         @line3.setPosY(y - 640*Math.cos(sample.angle))
 #         @line3.setEndX(x + 640*Math.sin(sample.angle))
 #         @line3.setEndY(y + 640*Math.cos(sample.angle))
#      else
#          @line1.setPosX(0)
#          @line1.setPosY(0)
#          @line1.setEndX(0)
#          @line1.setEndY(0)
#          @line2.setPosX(0)
#          @line2.setPosY(0)
#          @line2.setEndX(0)
#          @line2.setEndY(0)
#          @line3.setPosX(0)
#          @line3.setPosY(0)
#          @line3.setEndX(0)
#          @line3.setEndY(0)

#      end
#      @window.ImageView.update2
  end

  def show
      @window.show
  end
end

