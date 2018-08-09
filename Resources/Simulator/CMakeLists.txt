cmake_minimum_required(VERSION 2.8)

# Project name
project(VREP-Naoqi-Sim)

# Additional cmake configuration path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Add the path to simulator-sdk for use in the source code.
add_definitions(-DNAOQI_SIM_SDK="$ENV{PATH_TO_SIM_DIR}")

# Add the VREP configuration params and other build params.
add_definitions(-DNON_MATLAB_PARSING=1 -DMAX_EXT_API_CONNECTIONS=255 -std=gnu++0x)

# Find naoqi modules using FindNAOqiSIM.cmake.
find_package(NAOqiSIM REQUIRED COMPONENTS 
	alnaosim 
	alsimutils
	alrobotmodel
	alproxies
	almemoryfastaccess
	alcommon	  
	qi
	qitype
)

# Set boost older version library paths from the sdk
set(BOOST_SYSTEM_LIBRARIES
  ${_boost_root}/lib/libboost_system.so
  CACHE INTERNAL "" FORCE
)

# Set boost older version include paths from the sdk
set(BOOST_SYSTEM_INCLUDE_DIRS
  ${_boost_root}/include
  CACHE INTERNAL "" FORCE
)

# Set BOOST_LIBS variable to avoid conflict with system dirs.
set(BOOST_LIBS
  $ENV{PATH_TO_SIM_DIR}/lib/libboost_system.so)

# Set BOOST_INCLUDE_DIRS variable to avoid conflict with system dirs.
set(BOOST_INCLUDE_DIRS
  $ENV{PATH_TO_SIM_DIR}/include)

# Check if naoqi modules are found.
if(NAOqiSIM_FOUND)
	message(STATUS "NAOqi Simulator SDK found!")
endif()

# Set the root folder dir.
set(MAIN $ENV{PATH_TO_TEAM_NUST_DIR})

# Set the source files for the simulator interface.
FILE(GLOB SOURCE_MAIN    
	"${MAIN}/Resources/Simulator/*.cpp"
	"${MAIN}/Resources/Simulator/*.c")

# Set the include dirs for the simulator interface.
include_directories(${NAOqiSIM_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

# Build the simulator interface.
add_executable(VREP-Naoqi-Sim ${SOURCE_MAIN})

# Linking libraries
target_link_libraries(VREP-Naoqi-Sim ${NAOqiSIM_LIBRARIES} ${BOOST_LIBS} pthread)
