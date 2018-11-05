# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-base)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(tnrs-base SHARED ${SRC})

qi_use_lib(tnrs-base
  BOOST
  OPENCV2_CORE
  PTHREAD
)

target_link_libraries(tnrs-base
  libtnrs-utils.so)
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} 
          tnrs-base
          libtnrs-utils.so)
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
