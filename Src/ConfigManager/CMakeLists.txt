# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-config-manager)

include($ENV{PATH_TO_TEAM_NUST_DIR}/Make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (TNRS_CONFIG_MANAGER_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager")

####################################
# Other parameters
####################################

find_path(EIGEN3_INCLUDE_DIRS "Eigen" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen3")
if (EIGEN3_INCLUDE_DIRS)
  include_directories ("${EIGEN3_INCLUDE_DIRS}")
endif (EIGEN3_INCLUDE_DIRS)

#set(TNRS_CONFIG_MANAGER_SRC "${TNRS_CONFIG_MANAGER_DIR}/src/*.cpp")
#qi_create_lib (tnrs-config-manager SHARED ${TNRS_CONFIG_MANAGER_SRC})

file (GLOB TNRS_CONFIG_MANAGER_SRC "${TNRS_CONFIG_MANAGER_DIR}/src/*.cpp")
add_library (tnrs-config-manager SHARED ${TNRS_CONFIG_MANAGER_SRC})

qi_use_lib(tnrs-config-manager
  BOOST
  BOOST_FILESYSTEM
  BOOST_IOSTREAMS
  PTHREAD
)

if (BUILD_TNRS_TESTS)
    set (test_DIR "${TNRS_CONFIG_MANAGER_DIR}/tests/")
    file (GLOB TESTS RELATIVE "${test_DIR}" "${TNRS_CONFIG_MANAGER_DIR}/tests/*.cpp")
    foreach (testname ${TESTS})
        string(REPLACE ".cpp" "" targetname "${testname}")
        add_executable (${targetname} "${test_DIR}/${testname}")
        target_link_libraries (${targetname} tnrs-config-manager)
    endforeach (testname ${TESTS})
endif (BUILD_TNRS_TESTS)

