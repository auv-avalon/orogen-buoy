
#! /usr/bin/env ruby
# -*- coding: utf-8 -*-

require './buoy_detector_new.rb'
require 'orocos/log'
#require 'type_specialize'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"
#puts Orocos.register_pkgconfig_path('../build')

Orocos.initialize
Orocos.run do
  #Orocos.log_all_ports
  detector = Orocos::TaskContext.get "buoy_detector"
  camera = Orocos::TaskContext.get "front_camera"
  controller = Orocos::TaskContext.get "buoy_wall_servoing"
  imu = Orocos::TaskContext.get "imu"
  wall_detector = Orocos::TaskContext.get "servoing_wall_detector"

  camera.frame.connect_to detector.frame

  detector.configure
  detector.start

  imu.pose_samples.connect_to controller.orientation_samples
  detector.buoy.connect_to controller.buoy_samples
  wall_detector.wall.connect_to controller.wall_samples

  controller.configure
  controller.start



  sleep 5

  STDERR.puts "configuration done"

  gui = BuoyDetector.new(camera, detector)
  gui.show

  Vizkit.display detector
  Vizkit.display controller

  Vizkit.exec

  STDERR.puts "shutting down"
end

