#! /usr/bin/env ruby
# -*- coding: utf-8 -*-

require './buoy_detector_new.rb'
require 'orocos/log'
#require 'type_specialize'

include Orocos


Orocos::CORBA.name_service = "192.168.128.51"
#puts Orocos.register_pkgconfig_path('../build')

Orocos.initialize
if ARGV.size < 1
  puts "No Log File in use"
  exit 1
end
Orocos.run ("buoy_test") do
  #Orocos.log_all_ports
  detector = Orocos::TaskContext.get "buoy_detector"
  

  log.front_camera.frame.connect_to detector.frame

  detector.configure
  detector.start

  sleep 5

  STDERR.puts "configuration done"

  gui = BuoyDetector.new(log.front_camera, detector)
  gui.show

  Vizkit.display detector.buoy
  Vizkit.control log
  Vizkit.exec

  STDERR.puts "shutting down"
end

