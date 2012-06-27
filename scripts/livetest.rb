require 'orocos'
require 'vizkit'
include Orocos

Orocos::CORBA.name_service = "192.168.128.51"


Orocos.initialize

#Orocos.run "buoydetector" do 
Orocos.run("buoy_test" => nil, :wait => 20) do
  survey = Orocos::TaskContext.get "buoy_survey"
  detector = Orocos::TaskContext.get "buoy_detector"
  camera = Orocos::TaskContext.get "front_camera"

 #Vizkit.display detector.buoy
  #Vizkit.display detector.relative_position

  camera.frame.connect_to detector.frame
  detector.buoy.connect_to survey.input_buoy

  detector.configure
  detector.start

  survey.configure
  survey.start

   Vizkit.display camera.frame

  begin
     puts "busy"
     Vizkit.exec
  rescue Interrupt => e
     puts "Ende"
  end
end


