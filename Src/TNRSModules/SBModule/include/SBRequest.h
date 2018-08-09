/**
 * @file SBModule/include/SBRequest.h
 *
 * This file defines the class SBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "BehaviorManager/include/BehaviorRequest.h"

/**
 * Types of request valid for SBModule
 * 
 * @enum SBRequestIds
 */ 
enum class SBRequestIds {
  BEHAVIOR_REQUEST,
  KILL_BEHAVIOR
};

/**
 * @class SBRequest
 * @brief A module request that can be handled by SBModule
 */ 
class SBRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  SBRequest(const SBRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::STATIC, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<SBRequest> SBRequestPtr;

/**
 * @class RequestStaticBehavior
 * @brief A request to start a static behavior
 */ 
struct RequestStaticBehavior : public SBRequest, BehaviorRequest
{
  /**
   * Constructor
   */ 
  RequestStaticBehavior(const SBConfigPtr& config) :
    SBRequest(SBRequestIds::BEHAVIOR_REQUEST),
    BehaviorRequest(config)
  {
  }
};
typedef boost::shared_ptr<RequestStaticBehavior> RequestStaticBehaviorPtr;


/**
 * @class KillStaticBehavior
 * @brief A request to kill a static behavior
 */ 
struct KillStaticBehavior : public SBRequest
{
  /**
   * Constructor
   */ 
  KillStaticBehavior() :
    SBRequest(SBRequestIds::KILL_BEHAVIOR)
  {
  }
};
typedef boost::shared_ptr<KillStaticBehavior> KillStaticBehaviorPtr;
