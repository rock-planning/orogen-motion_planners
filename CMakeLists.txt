PROJECT(motion_planners)
cmake_minimum_required(VERSION 2.6)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(motion_plannersBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

