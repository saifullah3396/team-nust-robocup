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
    const float& maxRunTime = 15.f);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBHeadControlConfig> 
    makeFromJson(const Json::Value& obj);
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
    const float& scanMaxPitch = 16.f * M_PI / 180);
    
  /**
   * @derived
   */  
  void validate() throw (BConfigException);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
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
    const float& maxRunTime = 9999.f);
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  //! Head target type
  HeadTargetTypes headTargetType;
};
typedef boost::shared_ptr<HeadTargetTrackConfig> HeadTargetTrackConfigPtr;
