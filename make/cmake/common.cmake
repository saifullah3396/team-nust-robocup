cmake_minimum_required(VERSION 2.8.12)

find_package(qibuild)

include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src/TNRSModules")

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

option(MODULE_IS_REMOTE "The module is compiled as a remote module (ON or OFF)" ON)
option(MODULE_IS_LOCAL_SIMULATED "The module is compiled as a remote module (ON or OFF)" OFF)

set(TNRS_COMPONENTS
    tnrs-utils
    tnrs-config-manager
    tnrs-debug-module
    tnrs-base
    tnrs-comm-module
    tnrs-control-module
    tnrs-behavior-manager
    tnrs-planning-module
    tnrs-random-lib
    tnrs-motion-module
    tnrs-sb-module
    tnrs-localization-module
    tnrs-vision-module
    )

####################################
# Settings
####################################
set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES OFF)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES OFF)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")

####################################
# Options
####################################
# To be used later for testing code.
SET (TOOLCHAIN_NAME CACHE STRING "Name of the toolchain.")
SET (BUILD_TNRS_TESTS OFF CACHE BOOL "Whether to build tests.")
SET (BUILD_TNRS_EXAMPLES OFF CACHE BOOL "Whether to build example.")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG_BUILD")

if(BUILD_TNRS_TESTS)
  if (MODULE_IS_REMOTE) 
    add_definitions(-DMODULE_IS_REMOTE -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  elseif (MODULE_IS_LOCAL_SIMULATED) 
    add_definitions(-DMODULE_IS_LOCAL_SIMULATED -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  else()
    message(FATAL_ERROR "You can not build tests with cross compilation toolchain.")
  endif()
else()
  if (MODULE_IS_REMOTE) 
    add_definitions(-DMODULE_IS_REMOTE -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  elseif (MODULE_IS_LOCAL_SIMULATED) 
    add_definitions(-DMODULE_IS_LOCAL_SIMULATED -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  else()
    add_definitions(-DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
  endif()
endif()
set(TNRS_BUILD "$ENV{PATH_TO_TEAM_NUST_DIR}/build")
if (MODULE_IS_REMOTE) 
	set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/remote")
	set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/remote")
elseif(MODULE_IS_LOCAL_SIMULATED)
	set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/sim")
	set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/sim")
else()
	set(TNRS_BUILD_DEBUG "${TNRS_BUILD}/Debug/cross")
	set(TNRS_BUILD_RELEASE "${TNRS_BUILD}/Release/cross")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set (FIND_BUILD_DIR ${TNRS_BUILD_DEBUG})
else()
  set (FIND_BUILD_DIR ${TNRS_BUILD_RELEASE})
endif()

link_directories(${FIND_BUILD_DIR}/lib)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${TNRS_BUILD_RELEASE}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${TNRS_BUILD_DEBUG}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${TNRS_BUILD_RELEASE}/lib")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${TNRS_BUILD_DEBUG}/lib")

link_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/resources/RandomLib/lib")

#message("MODULE PATH" ${CMAKE_MODULE_PATH})

#SET(CXX_WARN_FLAGS_EIGEN -Wall -Wfloat-equal)
#SET(CXX_WARN_FLAGS ${CXX_WARN_FLAGS_EIGEN} -pedantic)
SET(CXX_FLAGS ${CXX_WARN_FLAGS} -std=gnu++0x)
add_definitions(${CXX_FLAGS})
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/resources/Eigen")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src")
include_directories("$ENV{PATH_TO_TEAM_NUST_DIR}/src/ProcessingChilds")
