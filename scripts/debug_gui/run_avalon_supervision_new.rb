#! /usr/bin/env ruby
# -*- coding: utf-8 -*-

require './buoy_detector_new.rb'
#require 'type_specialize'

include Orocos


Orocos::CORBA.name_service = "192.168.128.51"
#puts Orocos.register_pkgconfig_path('../build')

Orocos.initialize
Orocos.run do
  #Orocos.log_all_ports
  detector = Orocos::TaskContext.get "buoy_detector"
  front_camera = Orocos::TaskContext.get "front_camera"  

  gui = BuoyDetector.new(front_camera, detector)
  gui.show

  Vizkit.display detector.buoy
  Vizkit.exec

  STDERR.puts "shutting down"
end

