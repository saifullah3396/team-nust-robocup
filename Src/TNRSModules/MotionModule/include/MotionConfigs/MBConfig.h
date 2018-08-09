/**
 * @file MotionModule/include/MotionConfigs/MBConfig.h
 *
 * This file defines the struct MBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "MotionModule/include/MotionBehaviorIds.h"
#include "BehaviorManager/include/BehaviorConfig.h"

/**
 * @struct MBConfig
 * @brief Base class for all motion behavior configurations
 */
struct MBConfig : BehaviorConfig 
{
	/**
	 * Constructor
	 * 
	 * @param id: Id of the behavior
	 * @param maxRunTime: Max running time for the behavior
	 * @param type: Type of the behavior
	 */
  MBConfig(
    const MBIds& id, 
    const float& maxRunTime,
    const int& childType = -1) : 
  BehaviorConfig((unsigned)id, BaseBehaviorType::MOTION, maxRunTime, childType)
  {}
  
  /**
   * Returns true if the configuration is valid. This configuration can 
   * only be made valid through a child therefore it returns a false.
   * 
   * @return bool
   */
  virtual bool isValid() {
    return false;
  }
};
typedef boost::shared_ptr<MBConfig> MBConfigPtr;
