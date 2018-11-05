/**
 * @file ControlModule/include/ActuatorRequests.h
 *
 * This file defines the classes ActuatorRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DebugUtils.h"

using namespace std;

/**
 * @class ActuatorRequest
 * @brief A class that defines a basic actuation request for commanded
 *   actuator values
 */
class ActuatorRequest
{
public:
  /**
   * Constructor
   *
   * @param size: Number of actuators in this request
   */
  ActuatorRequest(const size_t& size) :
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
