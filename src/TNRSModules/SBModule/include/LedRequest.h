/**
 * @file ControlModule/include/LedRequest.h
 *
 * This file defines the class LedRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "SBModule/include/SBRequest.h"

/**
 * @struct LedRequest
 * @brief Defines a basic leds actuation request
 */
struct LedRequest : public ActuatorRequest, public SBRequest
{
  /**
   * Constructor
   */
  LedRequest() :
    ActuatorRequest(NUM_LED_ACTUATORS),
    SBRequest(SBRequestIds::LED_REQUEST)
  {
  }
};
typedef boost::shared_ptr<LedRequest> LedRequestPtr;
