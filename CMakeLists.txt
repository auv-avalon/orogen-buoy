PROJECT(buoy)
cmake_minimum_required(VERSION 2.6)

find_package(Rock)
rock_activate_cxx11()

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(buoyBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

