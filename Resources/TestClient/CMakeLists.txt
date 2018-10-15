# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-client)
find_package(qibuild)

add_definitions(--std=gnu++0x)

file (GLOB TNRS_CLIENT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
qi_create_bin(tnrs-client ${TNRS_CLIENT_SRC})

qi_use_lib(tnrs-client  
  BOOST
  OPENCV2_CORE
  OPENCV2_HIGHGUI
  OPENCV2_CALIB3D
  OPENCV2_OBJDETECT
  OPENCV2_VIDEO
)
