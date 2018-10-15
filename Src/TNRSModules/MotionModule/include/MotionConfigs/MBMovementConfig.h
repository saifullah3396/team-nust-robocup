/**
 * @file MotionModule/include/MotionConfigs/MBMovementConfig.h
 *
 * This file defines the structs MBMovementConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#pragma once

#include "MBConfig.h"
#include "Utils/include/RobotStateDefinitions.h"

struct MBPostureConfig;
struct MBHeadControlConfig;
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;
typedef boost::shared_ptr<MBHeadControlConfig> MBHeadControlConfigPtr;

/**
 * @struct MBMovementConfig
 * @brief Movement behavior configuration
 */
struct MBMovementConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param goal: Robot goal pose
   * @param reachClosest: Whether to reach the closest position to goal if possible
   * @param type: Type of MovementModule
   */ 
  MBMovementConfig(
    const RobotPose2D<float>& goal = RobotPose2D<float>(0.0, 0.0, 0.0),
    const bool& reachClosest = true,
    const MBMovementTypes& type = MBMovementTypes::GO_TO_TARGET);
    
  /**
   * Constructor
   * 
   * @param goal: Robot goal pose
   * @param reachClosest: Whether to reach the closest position to goal if possible
   * @param type: Type of MovementModule
   * @param htConfig: Head control config if needed
   */ 
  MBMovementConfig(
    const RobotPose2D<float>& goal,
    const bool& reachClosest,
    const MBMovementTypes& type, 
    const MBHeadControlConfigPtr& htConfig);
  
  /**
   * Constructor
   * 
   * @param goal: Robot goal pose
   * @param reachClosest: Whether to reach the closest position to goal if possible
   * @param type: Type of MovementModule
   * @param postureConfig: Posture module config if needed
   */ 
  MBMovementConfig(
    const MBMovementTypes& type, 
    const RobotPose2D<float>& goal,
    const bool& reachClosest,
    const MBPostureConfigPtr& postureConfig);
  
  /**
   * Constructor
   * 
   * @param goal: Robot goal pose
   * @param reachClosest: Whether to reach the closest position to goal if possible
   * @param type: Type of MovementModule
   * @param htConfig: Head control config if needed
   * @param postureConfig: Posture module config if needed
   */ 
  MBMovementConfig(
    const MBMovementTypes& type, 
    const RobotPose2D<float>& goal,
    const bool& reachClosest,
    const MBHeadControlConfigPtr& htConfig,
    const MBPostureConfigPtr& postureConfig);
  
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
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBMovementConfig> 
    makeFromJson(const Json::Value& obj);

  bool reachClosest;
  RobotPose2D<float> goal;
  MBPostureConfigPtr postureConfig;
  MBHeadControlConfigPtr htConfig;
};

typedef boost::shared_ptr<MBMovementConfig> MBMovementConfigPtr;
