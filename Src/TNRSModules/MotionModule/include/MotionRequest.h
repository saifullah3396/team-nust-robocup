/**
 * @file MotionModule/include/MotionRequest.h
 *
 * This file defines the class MotionRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "MotionModule/include/MotionConfigs/MBConfig.h"
#include "BehaviorManager/include/BehaviorRequest.h"

/**
 * Types of request valid for MotionModule
 * 
 * @enum MotionRequestIds
 */ 
enum class MotionRequestIds {
  BEHAVIOR_REQUEST,
  KILL_BEHAVIOR
};

/**
 * @class MotionRequest
 * @brief A module request that can be handled by MotionModule
 */ 
class MotionRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  MotionRequest(const MotionRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::MOTION, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<MotionRequest> MotionRequestPtr;

/**
 * @class RequestMotionBehavior
 * @brief A request to start a motion behavior
 */ 
struct RequestMotionBehavior : public MotionRequest, BehaviorRequest
{
  /**
   * Constructor
   */ 
  RequestMotionBehavior(const MBConfigPtr& config) :
    MotionRequest(MotionRequestIds::BEHAVIOR_REQUEST),
    BehaviorRequest(config)
  {
  }
};
typedef boost::shared_ptr<RequestMotionBehavior> RequestMotionBehaviorPtr;

/**
 * @class KillMotionBehavior
 * @brief A request to kill a motion behavior
 */ 
struct KillMotionBehavior : public MotionRequest
{
  /**
   * Constructor
   */ 
  KillMotionBehavior() :
    MotionRequest(MotionRequestIds::KILL_BEHAVIOR)
  {
  }
};
typedef boost::shared_ptr<KillMotionBehavior> KillMotionBehaviorPtr;
