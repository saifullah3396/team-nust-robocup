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
    const MBPostureTypes type = MBPostureTypes::INTERP_TO_POSTURE) :
    MBConfig(MBIds::POSTURE, 3.f, (int)type),
    timeToReachP(timeToReachP),
    targetPosture(PostureState::UNKNOWN)
  {
  }
  
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
    const MBPostureTypes type = MBPostureTypes::INTERP_TO_POSTURE) :
    MBConfig(MBIds::POSTURE, 3.f, (int)type), 
    targetPosture(targetPosture),
    timeToReachP(timeToReachP)
  {
    if ((unsigned)targetPosture >= (unsigned)PostureState::STATIC_POSTURES) 
      return;
    jointsToReach = 
      VectorXf::Map(
        &postureDefinitions[(unsigned)targetPosture][0],
        sizeof(postureDefinitions[(unsigned)targetPosture]) / 
        sizeof(postureDefinitions[(unsigned)targetPosture][0])
      );
  }

  /**
   * Returns true if the configuration parameters are valid
   * 
   * @return bool
   */
  bool isValid() {
    try {
      if (timeToReachP <= 0.f || // Undefined time given
          (unsigned)targetPosture >= 
          (unsigned)PostureState::STATIC_POSTURES || 
          jointsToReach.size() != NUM_JOINTS) 
      { 
        throw 
          BConfigException(
            this, 
            "Invalid behavior configuration parameters passed.", 
            false, 
            EXC_INVALID_BCONFIG_PARAMETERS
          );
      }
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }

  //! Time to reach the posture in
  float timeToReachP;
  
  //! Joints to reach for the posture
  VectorXf jointsToReach;
  
  //! Target posture to reach
  PostureState targetPosture;
};
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;
