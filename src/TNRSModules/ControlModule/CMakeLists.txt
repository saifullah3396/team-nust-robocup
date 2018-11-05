# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-control-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (TNRS_CONTROL_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_CONTROL_MODULE_SRC "${TNRS_CONTROL_MODULE_DIR}/src/*.cpp")
add_library (tnrs-control-module SHARED ${TNRS_CONTROL_MODULE_SRC})

qi_use_lib(tnrs-control-module
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

target_link_libraries(tnrs-control-module
  libtnrs-utils.so
  libtnrs-base.so
)
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${TNRS_CONTROL_MODULE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${TNRS_CONTROL_MODULE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} 
          tnrs-control-module 
          libtnrs-utils.so
          libtnrs-base.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
