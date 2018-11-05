# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-behavior-manager)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (TNRS_BEHAVIOR_MANAGER_DIR ${CMAKE_CURRENT_SOURCE_DIR})

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_BEHAVIOR_MANAGER_SRC "${TNRS_BEHAVIOR_MANAGER_DIR}/src/*.cpp")
add_library (tnrs-behavior-manager SHARED ${TNRS_BEHAVIOR_MANAGER_SRC})

qi_use_lib(tnrs-behavior-manager
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

target_link_libraries(tnrs-behavior-manager
  libtnrs-utils.so
  libtnrs-base.so
)
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${TNRS_BEHAVIOR_MANAGER_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${TNRS_BEHAVIOR_MANAGER_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-behavior-manager
          libtnrs-utils.so
          libtnrs-base.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
