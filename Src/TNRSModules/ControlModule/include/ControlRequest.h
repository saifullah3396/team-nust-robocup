/**
 * @file ControlModule/include/ControlRequest.h
 *
 * This file defines the class ControlRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"

/**
 * Types of request valid for ControlModule
 * 
 * @enum ControlRequestIds
 */ 
enum class ControlRequestIds {
  JOINT_REQUEST,
  STIFFNESS_REQUEST,
  LED_REQUEST
};

/**
 * @class ControlRequest
 * @brief A module request that can be handled by ControlModule
 */ 
class ControlRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  ControlRequest(const ControlRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::CONTROL, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<ControlRequest> ControlRequestPtr;
