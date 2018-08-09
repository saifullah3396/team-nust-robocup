/**
 * @file ControlModule/include/ActuatorRequests.h
 *
 * This file defines the classes ActuatorRequest, JointRequest,
 * StiffnessRequest, and LedRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include "ControlModule/include/ControlRequest.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DebugUtils.h"

using namespace std;

/**
 * @class ActuatorRequest
 * @brief A class that defines a basic actuation request for commanded
 *   actuator values
 */
class ActuatorRequest : public ControlRequest
{
public:
  /**
   * Constructor
   *
   * @param id: Id of the request
   * @param size: Number of actuators in this request
   */
  ActuatorRequest(
    const ControlRequestIds& id,
    const size_t& size) :
    ControlRequest(id),
    size(size)
  {
    value.resize(size, NAN);
  }

  /**
   * Sets the values of all the actuators
   *
   * @param value: A vector of actuator request values
   */
  void setValue(const vector<float>& value)
  {
    ASSERT(value.size() == size);
    this->value = value;
  }

  /**
   * Sets the value of the actuator at given index
   *
   * @param value: The actuator value
   * @param index: The actuator index
   */
  void setValue(const float& value, const unsigned& index)
  {
    this->value[index] = value;
  }

  /**
   * Gets the current commanded actuator request
   *
   * @return vector<float>
   */
  vector<float> getValue()
  {
    return value;
  }

private:
  //! Vector of actuator request values
  vector<float> value;
  
  //! Total number of actuators in this request
  size_t size;
};
typedef boost::shared_ptr<ActuatorRequest> ActuatorRequestPtr;

/**
 * @class JointRequest
 * @brief Defines a basic joint actuation request
 */
struct JointRequest : public ActuatorRequest
{
  /**
   * Constructor
   */
  JointRequest() :
    ActuatorRequest(ControlRequestIds::JOINT_REQUEST, NUM_JOINTS)
  {
  }
};
typedef boost::shared_ptr<JointRequest> JointRequestPtr;

/**
 * @struct StiffnessRequest
 * @brief Defines a basic stiffness actuation request
 */
struct StiffnessRequest : public ActuatorRequest
{
  /**
   * Constructor
   */
  StiffnessRequest() :
    ActuatorRequest(ControlRequestIds::STIFFNESS_REQUEST, NUM_JOINTS)
  {
  }
};
typedef boost::shared_ptr<StiffnessRequest> StiffnessRequestPtr;

/**
 * @struct LedRequest
 * @brief Defines a basic leds actuation request
 */
struct LedRequest : public ActuatorRequest
{
  /**
   * Constructor
   */
  LedRequest() :
    ActuatorRequest(ControlRequestIds::LED_REQUEST, NUM_LED_ACTUATORS)
  {
  }
};
typedef boost::shared_ptr<LedRequest> LedRequestPtr;
