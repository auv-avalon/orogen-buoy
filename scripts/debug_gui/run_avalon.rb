#! /usr/bin/env ruby
# -*- coding: utf-8 -*-

require './buoy_detector_new.rb'
require 'orocos/log'
#require 'type_specialize'

include Orocos


Orocos::CORBA.name_service = "192.168.128.51"
Orocos.initialize

Orocos.run ("buoy_test") do
  #Orocos.log_all_ports
  detector = Orocos::TaskContext.get "buoy_detector"
  camera = Orocos::TaskContext.get "front_camera"

  camera.frame.connect_to detector.frame

  detector.configure
  detector.start

  sleep 5

  STDERR.puts "configuration done"

  gui = BuoyDetector.new(camera, detector)
  gui.show

  Vizkit.display detector

  Vizkit.exec

  STDERR.puts "shutting down"
end

