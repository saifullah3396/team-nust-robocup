# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-planning-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/Make/cmake/common.cmake)
include_directories(include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

#find_package(PythonInterp REQUIRED)
#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON_INCLUDE_DIRS})

file (GLOB TNRS_PLANNING_MODULE_SRC 
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/RobotStartup/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/RobotStartup/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/ExternalInterface/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/ExternalInterface/Types/*.cpp"
      #"${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/Robocup/*.cpp"
      #"${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/Robocup/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/KickSequence/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/KickSequence/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/KickSequence/Types/*.c")
      
add_library (tnrs-planning-module SHARED ${TNRS_PLANNING_MODULE_SRC})
qi_use_lib(tnrs-planning-module
  ALCOMMON
  ALPROXIES
  BOOST
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_CALIB3D
  OPENCV2_OBJDETECT
  OPENCV2_VIDEO
  PTHREAD
)

target_link_libraries(tnrs-planning-module
  libtnrs-utils.so
  libtnrs-base.so
  libtnrs-behavior-manager.so
  -ljsoncpp
)
  
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-planning-module
          libtnrs-utils.so
          libtnrs-base.so
          libtnrs-behavior-manager.so
          -ljsoncpp
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)

#add_library(PyRobotStartup SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/PlanningBehaviors/RobotStartup/PyTest/PyRobotStartup.cpp")
#qi_use_lib(PyRobotStartup
#  BOOST
#  BOOST_PYTHON
#)
#target_link_libraries(PyRobotStartup 
#  ${PYTHON_LIBRARIES} 
#  libtnrs-utils.so
#  libtnrs-config-manager.so
#  libtnrs-base.so
#  libtnrs-behavior-manager.so
#  libtnrs-motion-module.so
#  libtnrs-sb-module.so
#  libtnrs-vision-module.so
#  libtnrs-planning-module.so
#  -ljsoncpp)
#add_dependencies(PyRobotStartup Boost)
#
#set_target_properties(PyRobotStartup PROPERTIES SUFFIX .so)
#set_target_properties(PyRobotStartup PROPERTIES PREFIX "")  
