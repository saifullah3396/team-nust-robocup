/**
 * @file MotionModule/include/MotionConfigs/MBPostureConfig.h
 *
 * This file defines the structs MBPostureConfig 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct MBPostureConfig
 * @brief Posture behavior configuration
 */
struct MBPostureConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param jointsToReach: Joints to reach for the posture
   * @param timeToReachP: Time to reach the posture in
   * @param type: Type of the posture behavior
   */  
  MBPostureConfig(
    const VectorXf& jointsToReach,
    const float& timeToReachP = 2.f,
    const MBPostureTypes type = MBPostureTypes::INTERP_TO_POSTURE);
    
  /**
   * Constructor
   * 
   * @param targetPosture: Target posture to reach
   * @param timeToReachP: Time to reach the posture in
   * @param type: Type of the posture behavior
   */  
  MBPostureConfig(
    const PostureState& targetPosture,
    const float& timeToReachP = 2.f,
    const MBPostureTypes type = MBPostureTypes::INTERP_TO_POSTURE);

  /**
   * @derived
   */ 
  void validate() throw (BConfigException);
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBPostureConfig> 
    makeFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();

  //! Time to reach the posture in
  float timeToReachP;
  
  //! Joints to reach for the posture
  VectorXf jointsToReach;
  
  //! Target posture to reach
  PostureState targetPosture;
};
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;
