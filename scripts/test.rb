require "vizkit"

Orocos::CORBA.name_service = "127.0.0.1"
Orocos::CORBA.max_message_size = 80000000
Orocos.initialize

if ARGV.size < 1
  puts "No Log File in use"
  exit 1
end

#Orocos.run do
Orocos.run("buoydetector_test", :wait => 20) do
  log = Orocos::Log::Replay.open(ARGV[0], Typelib::Registry.new)

  detector = Orocos::TaskContext.get "buoydetector_test"

  log.front_camera.frame.connect_to detector.frame

  #Vizkit.display detector.buoy
  #Vizkit.display detector.relative_position

  detector.configure
  
  detector.start

  begin
    Vizkit.control log
    Vizkit.exec
  rescue Interrupt => e
     puts "Ende"
  end
end


