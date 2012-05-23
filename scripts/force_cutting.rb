require 'orocos'
require 'vizkit'
include Orocos

Orocos::CORBA.name_service = "192.168.128.51"
Orocos.initialize

  detector = Orocos::TaskContext.get "buoy_detector"
  detector.force_cutting.writer.write(true)
