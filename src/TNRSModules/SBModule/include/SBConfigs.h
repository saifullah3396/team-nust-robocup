/**
 * @file SBModule/include/SBConfigs.h
 *
 * This file defines all the static behavior configurations
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "SBModule/include/StiffnessModule/StiffnessDefinitions.h"
#include "SBModule/include/StaticBehaviorIds.h"
#include "BehaviorManager/include/BehaviorConfig.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct SBConfig
 * @brief Base static behavior configuration
 */
struct SBConfig : BehaviorConfig
{
	/**
	 * Constructor
	 * 
	 * @param id: Id of the behavior
	 * @param maxRunTime: Max running time for the behavior
	 * @param type: Type of the behavior
	 */
  SBConfig(
    const SBIds& id,
    const float& maxRunTime,
    const int& type) :
  BehaviorConfig((unsigned)id, STATIC, maxRunTime, type)
  {
  }
};

typedef boost::shared_ptr<SBConfig> SBConfigPtr;

/**
 * @struct MBStiffnessConfig
 * @brief The stiffness behavior configuration
 */
struct SBStiffnessConfig : SBConfig
{
	/**
	 * Constructor
	 */
  SBStiffnessConfig() :
    SBConfig(SBIds::STIFFNESS, 3.f, (int)SBStiffnessTypes::STIFFNESS_INTERP),
    sToReach(vector<float>(NUM_JOINTS, 1.f)),
    timeToReachS(1.0f),
    targetState(StiffnessState::UNKNOWN)
  {
  }

	/**
	 * Constructor
	 * 
	 * @param sToReach: Stiffnesses to reach
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  SBStiffnessConfig(
    const vector<float>& sToReach,
    const float& timeToReachS,
    const SBStiffnessTypes& type = SBStiffnessTypes::STIFFNESS_INTERP) :
    SBConfig(SBIds::STIFFNESS, 3.f, (int)type),
    sToReach(sToReach),
    timeToReachS(timeToReachS),
    targetState(StiffnessState::UNKNOWN)
  {
  }
  
	/**
	 * Constructor
	 * 
	 * @param state: Stiffness state to reach
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  SBStiffnessConfig(
    const StiffnessState& state,
    const float& timeToReachS = 1.f,
    const SBStiffnessTypes& type = SBStiffnessTypes::STIFFNESS_INTERP) :
    SBConfig(SBIds::STIFFNESS, 3.f, (int)type),
    targetState(state),
    timeToReachS(timeToReachS)
  {
    if (state == StiffnessState::UNKNOWN)
      StiffnessState::ROBOCUP; // Set robocup as default
    sToReach =
      vector<float>(
        stiffnessDefinitions[(unsigned)state],
        stiffnessDefinitions[(unsigned)state] +
        sizeof(stiffnessDefinitions[(unsigned)state]) /
        sizeof(stiffnessDefinitions[(unsigned)state][0])
      );
  }
  
	/**
	 * Constructor
	 * 
	 * @param chainId: Robot chain whose stiffness is to be changed
	 * @param requiredS: Required value of stiffness
	 * @param timeToReachS: Time in which stiffness are required to reach
	 *   the given values
	 * @param type: Type of the stiffness behavior
	 */
  SBStiffnessConfig(
    const LinkChains& chainId,
    const float& requiredS,
    const float& timeToReachS,
    const SBStiffnessTypes& type = SBStiffnessTypes::STIFFNESS_INTERP) :
    SBConfig(SBIds::STIFFNESS, 3.f, (int)type),
    timeToReachS(timeToReachS),
    targetState(StiffnessState::UNKNOWN)
  {
    int chainStart = 0;
    int chainSize = 0;
    if (chainId == CHAIN_HEAD) {
      chainSize = HEAD_SIZE;
    } else if (chainId == CHAIN_L_ARM) {
      chainStart = L_SHOULDER_PITCH;
      chainSize = L_ARM_SIZE;
    } else if (chainId == CHAIN_L_ARM) {
      chainStart = R_SHOULDER_PITCH;
      chainSize = R_ARM_SIZE;
    } else if (chainId == CHAIN_L_LEG) {
      chainStart = L_HIP_YAW_PITCH;
      chainSize = L_LEG_SIZE;
    } else if (chainId == CHAIN_R_LEG) {
      chainStart = R_HIP_YAW_PITCH;
      chainSize = R_LEG_SIZE;
    }
    sToReach = vector<float>(NUM_JOINTS, NAN);
    for (int i = chainStart; i < chainStart + chainSize; ++i) {
      sToReach[i] = requiredS;
    }
  }

	/**
	 * Validates the given configuration parameters
	 */ 
  void validate() throw (BConfigException) {
    if (timeToReachS <= 0.f || // Undefined time given
        (unsigned)targetState >=
        (unsigned)StiffnessState::NUM_STATES ||
        sToReach.size() != NUM_JOINTS)
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false,
          EXC_INVALID_BCONFIG_PARAMETERS
        );
    }
  }

  //! Time to reach the given stiffnesses
  float timeToReachS;
  
  //! Stiffnesses to reach for each joint
  vector<float> sToReach;
  
  //! Target stiffness state
  StiffnessState targetState;
};

typedef boost::shared_ptr<SBStiffnessConfig> SBStiffnessConfigPtr;

/**
 * @struct MBLedsConfig
 * @brief The leds behavior configuration
 */
struct SBLedsConfig : SBConfig
{
	/**
	 * Constructor
	 * 
	 * @param inToReach: Required values of intensities to reach
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param type: Type of the led behavior
	 */
  SBLedsConfig(
    const vector<float>& inToReach,
    const float& timeToReachIn,
    const SBLedsTypes& type = SBLedsTypes::DIRECT_LEDS) :
    SBConfig(SBIds::LEDS, 45.f, (int)type),
    inToReach(inToReach),
    timeToReachIn(timeToReachIn)
  {
  }

	/**
	 * Constructor
	 * 
	 * @param groupId: Led group whose intensity is to be changed
	 * @param requiredIn: Required value of intensity
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param type: Type of the led behavior
	 */
  SBLedsConfig(
    const LedGroups& groupId,
    const float& requiredIn,
    const float& timeToReachIn,
    const SBLedsTypes& type = SBLedsTypes::DIRECT_LEDS) :
    SBConfig(SBIds::LEDS, 3.f, (int)type),
    timeToReachIn(timeToReachIn)
  {
    unsigned groupStart, groupSize;
    getGroup(groupStart, groupSize, groupId);
    inToReach = vector<float>(NUM_LED_ACTUATORS, NAN);
    for (size_t i = groupStart; i < groupStart + groupSize; ++i) {
      inToReach[i] = requiredIn;
    }
  }

	/**
	 * Constructor
	 * 
	 * @param groupId: Led group whose intensity is to be changed
	 * @param requiredIn: Required value of intensity
	 * @param timeToReacIn: Time in which intensities are required to reach
	 *   the given values
	 * @param bgr: The bgr values required for the given group
	 * @param type: Type of the led behavior
	 */
  SBLedsConfig(
    const LedGroups& groupId,
    const float& requiredIn,
    const float& timeToReachIn,
    const vector<unsigned>& bgr = vector<unsigned>(3),
    const SBLedsTypes& type = SBLedsTypes::DIRECT_LEDS) :
    SBConfig(SBIds::LEDS, 3.f, (int)type),
    timeToReachIn(timeToReachIn)
  {
    ASSERT(bgr.size() == 3) // For b, g, and r colors
    ASSERT(// RGB only supported for these led groups
      groupId == LedGroups::L_FACE ||
      groupId == LedGroups::R_FACE ||
      groupId == LedGroups::CHEST ||
      groupId == LedGroups::L_FOOT ||
      groupId == LedGroups::R_FOOT);
    unsigned groupStart, groupSize;
    getGroup(groupStart, groupSize, groupId);
    inToReach = vector<float>(NUM_LED_ACTUATORS, NAN);
    unsigned colorIndex = 2;
    float color;
    for (size_t i = groupStart; i < groupStart + groupSize; ++i) {
      color = bgr[colorIndex] / 255;
      inToReach[i] = requiredIn * color;
      if (!((i+1) % (groupSize / 3))) { //3 for blue, green, red
        colorIndex--;
      }
    }
  }

	/**
	 * Sets the start and size of the group of leds based on its id.
	 * 
	 * @param groupStart: Start index
	 * @param groupSize: Number of leds in the group
	 * @param groupId: Id of the group
	 */
  void getGroup(
    unsigned& groupStart,
    unsigned& groupSize,
    const LedGroups& groupId)
  {
    if (groupId == LedGroups::HEAD) {
      groupStart = HEAD_LED_REAR_LEFT_0_ACTUATOR;
      groupSize = NUM_HEAD_LED;
    } else if (groupId == LedGroups::L_FACE) {
      groupStart = FACE_LED_RED_LEFT_0_DEG_ACTUATOR; // OK
      groupSize = NUM_FACE_L_LED; // OK
    } else if (groupId == LedGroups::R_FACE) {
      groupStart = FACE_LED_RED_RIGHT_0_DEG_ACTUATOR; // OK
      groupSize = NUM_FACE_R_LED; // OK
    } else if (groupId == LedGroups::L_EAR) {
      groupStart = EARS_LED_LEFT_0_DEG_ACTUATOR; // OK
      groupSize = NUM_EL_LED; // OK
    } else if (groupId == LedGroups::R_EAR) {
      groupStart = EARS_LED_RIGHT_0_DEG_ACTUATOR; // OK
      groupSize = NUM_ER_LED; // OK
    } else if (groupId == LedGroups::CHEST) {
      groupStart = CHEST_BOARD_LED_RED_ACTUATOR;
      groupSize = NUM_C_LED;
    } else if (groupId == LedGroups::L_FEET) {
      groupStart = L_FOOT_LED_RED_ACTUATOR;
      groupSize = NUM_L_FEET_LED ;
    } else if (groupId == LedGroups::R_FEET) {
      groupStart = R_FOOT_LED_RED_ACTUATOR;
      groupSize = NUM_R_FEET_LED;
    }
  }

	/**
	 * Validates the given configuration parameters
	 */ 
  void validate() throw (BConfigException) {
    if (timeToReachIn <= 0.f || // Undefined time given
        inToReach.size() != NUM_LED_ACTUATORS)
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false,
          EXC_INVALID_BCONFIG_PARAMETERS
        );
    }
  }

	//! Time to reach the given led intensities
  float timeToReachIn;
  
  //! Intensities to reach for each led
  vector<float> inToReach;
};

typedef boost::shared_ptr<SBLedsConfig> SBLedsConfigPtr;

/**
 * @struct SBWDConfig
 * @brief The whistle detector behavior configuration
 */
struct SBWDConfig : SBConfig
{
	/**
	 * Constructor
	 * 
	 * @param type: Type of the led behavior
	 * @param timeToDetect: Max time given for whistle detection
	 */
  SBWDConfig(
    const SBWDTypes& type = SBWDTypes::AK_WHISTLE_DETECTOR,
    const float& timeToDetect = 10.f) :
    SBConfig(SBIds::WHISTLE_DETECTOR, 10.f, (int)type),
    timeToDetect(timeToDetect)
  {
  }

  /**
	 * Validates the given configuration parameters
	 */
  void validate() throw (BConfigException) {
    if (timeToDetect <= 0.f) // Undefined time given
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false,
          EXC_INVALID_BCONFIG_PARAMETERS
        );
    }
  }
  
  //! Max time for whistle detection
  float timeToDetect;
};

typedef boost::shared_ptr<SBWDConfig> SBWDConfigPtr;
