# General parameters
cmake_minimum_required(VERSION 2.8.12)
project (team-nust-spl)

find_package(qibuild)

include($ENV{PATH_TO_TEAM_NUST_DIR}/Make/cmake/common.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Resources/RandomLib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Resources/SbplLib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/Utils)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/ConfigManager)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSBase)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/BehaviorManager)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/CommModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/ControlModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/SBModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/MotionModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/PlanningModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/VisionModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TNRSModules/LocalizationModule)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Src/TeamNUSTSPL)
