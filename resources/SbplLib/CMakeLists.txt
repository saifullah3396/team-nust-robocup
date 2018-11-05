# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (tnrs-sbpl-lib)

find_package(qibuild)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(include)

####################################
# Settings
####################################
set (SBPL_LIB_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/resources/SbplLib")

####################################
# Other parameters
####################################

file (GLOB SBPL_SRC 
        "${SBPL_LIB_DIR}/src/discrete_space_information/*.cpp"
        "${SBPL_LIB_DIR}/src/heuristics/*.cpp"
        "${SBPL_LIB_DIR}/src/planners/*.cpp"
        "${SBPL_LIB_DIR}/src/utils/*.cpp"
     )
add_library (tnrs-sbpl-lib SHARED ${SBPL_SRC})
