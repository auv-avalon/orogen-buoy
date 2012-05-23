require 'orocos'
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"
Orocos::CORBA.name_service = "127.0.0.1"

Orocos.initialize

#Orocos.run "buoydetector" do 
Orocos.run("buoydetector_test" => nil, :wait => 20) do
  detector = TaskContext.get "buoydetector_test"
  #camera = Orocos::TaskContext.get "front_camera"
  camera = TaskContext.get 'front_camera_simulation'
#  camera.start

  buoy_monitor = Vizkit.display detector.buoy

  Vizkit.display detector.relative_position

  camera.frame.connect_to detector.frame
  detector.configure
  detector.start

#   Vizkit.display camera.frame

  begin
     puts "busy"
     Vizkit.exec
  rescue Interrupt => e
     puts "Ende"
  end

end


