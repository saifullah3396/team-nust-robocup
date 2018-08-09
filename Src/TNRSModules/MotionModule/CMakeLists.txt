# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-motion-module)

include($ENV{PATH_TO_TEAM_NUST_DIR}/Make/cmake/common.cmake)
include_directories(include)

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

file (GLOB TNRS_MOTION_MODULE_SRC 
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KinematicsModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/FallDetector/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/TrajectoryPlanner/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PathPlanner/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BalanceModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BalanceModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PostureModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/PostureModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KickModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/KickModule/Types/*.cpp"
    #  "${CMAKE_CURRENT_SOURCE_DIR}/src/MovementModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BallThrow/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/BallThrow/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/GetupModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/GetupModule/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/HeadControl/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/HeadControl/Types/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/DiveModule/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/DiveModule/Types/*.cpp"
    )

# Building the code for remote usage.
if(MODULE_IS_REMOTE)
  add_library (tnrs-motion-module SHARED ${TNRS_MOTION_MODULE_SRC})
  # Linking third-party sbpl lib dir.  
  # Linking libraries from the sdk.
  qi_use_lib(tnrs-motion-module
    ALCOMMON
    ALPROXIES
    PTHREAD
    BOOST
    BOOST_FILESYSTEM
    BOOST_IOSTREAMS
    OPENCV2_CORE
    OPENCV2_HIGHGUI
    OPENCV2_CALIB3D
    OPENCV2_OBJDETECT
    OPENCV2_VIDEO
  )

  # Linking external libraries installed as pre-requisites.
  # For remote usage the libraries can just be installed on the local pc.
  target_link_libraries(tnrs-motion-module -lnlopt)
else()
  add_library (tnrs-motion-module SHARED ${TNRS_MOTION_MODULE_SRC})
  # Linking libraries from the sdk.
  qi_use_lib(
    tnrs-motion-module
    ALCOMMON
    ALPROXIES
    PTHREAD
    BOOST
    BOOST_FILESYSTEM
    BOOST_IOSTREAMS
    OPENCV2_CORE
    OPENCV2_HIGHGUI
    OPENCV2_CALIB3D
    OPENCV2_OBJDETECT
    OPENCV2_VIDEO
  )

  # Linking external libraries installed as pre-requisites in the toolchain
  # These libraries are compiled on the virtual-nao in virtualbox robot env
  # and placed in the folder cross-toolchain/cross/i686-aldebaran-linux-gnu/lib
  target_link_libraries(tnrs-motion-module -lnlopt -lsbpl)
endif()

target_link_libraries(tnrs-motion-module
  libtnrs-utils.so
  libtnrs-config-manager.so
  libtnrs-base.so
  libtnrs-comm-module.so
  libtnrs-control-module.so
  libtnrs-behavior-manager.so
  libtnrs-sbpl-lib.so
)
    
if (BUILD_TNRS_TESTS)
    set (test_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
    file (GLOB test_SRC "${CMAKE_CURRENT_SOURCE_DIR}/tests/src/*.cpp")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}" ${test_SRC})
        target_link_libraries (${targetname} tnrs-motion-module
          libtnrs-utils.so
          libtnrs-config-manager.so
          libtnrs-base.so
          libtnrs-comm-module.so
          libtnrs-control-module.so
          libtnrs-behavior-manager.so
          libtnrs-sbpl-lib.so
        )
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)
