# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-utils)

include($ENV{PATH_TO_TEAM_NUST_DIR}/Make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (TNRS_UTILS_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/Src/Utils")

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_UTILS_SRC "${TNRS_UTILS_DIR}/src/*.cpp")
add_library (tnrs-utils SHARED ${TNRS_UTILS_SRC})

qi_use_lib(tnrs-utils
  BOOST
  OPENCV2_CORE
  PTHREAD
)

if (BUILD_TNRS_TESTS)
    set (test_DIR "${TNRS_UTILS_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${TNRS_UTILS_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-utils)
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
