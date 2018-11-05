/**
 * @file ControlModule/include/StiffnessRequest.h
 *
 * This file defines the class StiffnessRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "SBModule/include/SBRequest.h"

/**
 * @class StiffnessRequest
 * @brief Defines a basic stiffness actuator request
 */
struct StiffnessRequest : public ActuatorRequest, public SBRequest
{
  /**
   * Constructor
   */
  StiffnessRequest() :
    ActuatorRequest(NUM_JOINTS),
    SBRequest(SBRequestIds::STIFFNESS_REQUEST)
  {
  }
};
typedef boost::shared_ptr<StiffnessRequest> StiffnessRequestPtr;
