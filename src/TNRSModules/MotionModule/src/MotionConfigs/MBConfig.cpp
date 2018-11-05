/**
 * @file MotionModule/src/MotionConfigs/MBConfig.cpp
 *
 * This file implements the struct MBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBConfig.h"

MBConfig::MBConfig(
  const MBIds& id, 
  const float& maxRunTime,
  const int& childType) : 
BehaviorConfig((unsigned)id, BaseBehaviorType::MOTION, maxRunTime, childType)
{
}
