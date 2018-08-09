/**
 * @file MotionModule/include/MotionConfigs/MBHeadControlConfig.h
 *
 * This file defines the structs MBHeadControlConfig, 
 * HeadTargetSearchConfig and HeadTargetTrackConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/HeadControl/HeadTargetTypes.h"

/**
 * @struct MBHeadControlConfig
 * @brief Head control behavior base configuration
 */
struct MBHeadControlConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the head control behavior
   * @param maxRunTime: Maximum allowed run time for this behavior
   */ 
  MBHeadControlConfig(
    const MBHeadControlTypes& type, 
    const float& maxRunTime = 15.f) :
    MBConfig(MBIds::HEAD_CONTROL, maxRunTime, (int)type)
  {
  }
};
typedef boost::shared_ptr<MBHeadControlConfig> MBHeadControlConfigPtr;

/**
 * @struct HeadTargetSearchConfig
 * @brief Head control behavior to search for chosen target behavior 
 *   configuration
 */
struct HeadTargetSearchConfig : MBHeadControlConfig
{
  /**
   * Constructor
   * 
   * @param headTargetType: The head target type
   * @param scanLowerArea: Whether to scan the area near the robot feet
   *   as well
   * @param totalWaitTime: Total time to wait in each scan, left, right
   *   and middle
   * @param scanMaxYaw: Maximum head anglular motion about the z-axis
   * @param scanMaxPitch: Maximum head angular motion about the y-axis
   */
  HeadTargetSearchConfig(
    const HeadTargetTypes& headTargetType = HeadTargetTypes::BALL,
    const bool& scanLowerArea = false,
    const float& totalWaitTime = 1.f,
    const float& scanMaxYaw = 100.f * M_PI / 180,
    const float& scanMaxPitch = 16.f * M_PI / 180) :
    MBHeadControlConfig(MBHeadControlTypes::HEAD_TARGET_SEARCH),
    headTargetType(headTargetType),
    totalWaitTime(totalWaitTime),
    scanMaxYaw(scanMaxYaw),
    scanMaxPitch(scanMaxPitch) 
  {
  }
  
  /**
   * Returns true if the configuration parameters are valid
   * 
   * @return bool
   */ 
  bool isValid() {
    // To be defined
    return true;
  }
  
  //! Whether lower area scan is turned on
  bool scanLowerArea;
  
  //! Maximum head anglular motion about the z-axis
  float scanMaxYaw;
  
  //! Maximum head anglular motion about the y-axis
  float scanMaxPitch;
  
  //! Total time to wait in each scan, left, right and middle
  float totalWaitTime;
  
  //! Head target type
  HeadTargetTypes headTargetType;
};
typedef boost::shared_ptr<HeadTargetSearchConfig> HeadTargetSearchConfigPtr;

/**
 * @struct HeadTargetTrackConfig
 * @brief Head control behavior to track the chosen target behavior 
 *   configuration
 */
struct HeadTargetTrackConfig : MBHeadControlConfig
{
  /**
   * Constructor
   * 
   * @param headTargetType: The head target type
   * @param maxRunTime: Maximum allowed runtime for this behavior
   */
  HeadTargetTrackConfig(
    const HeadTargetTypes& headTargetType = HeadTargetTypes::BALL,
    const float& maxRunTime = 9999.f) : // Basically stay non stop in track
    MBHeadControlConfig(
      MBHeadControlTypes::HEAD_TARGET_TRACK, maxRunTime), 
    headTargetType(headTargetType)
  {
  }
  
  /**
   * Returns true if the configuration parameters are valid
   * 
   * @return bool
   */
  bool isValid() {
    return true;
  }
  
  //! Head target type
  HeadTargetTypes headTargetType;
};
typedef boost::shared_ptr<HeadTargetTrackConfig> HeadTargetTrackConfigPtr;
