# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-random-lib)

find_package(qibuild)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (RANDOM_LIB_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/resources/RandomLib")

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB RANDOM_LIB_SRC "${RANDOM_LIB_DIR}/src/*.cpp")
add_library (tnrs-random-lib SHARED ${RANDOM_LIB_SRC})
