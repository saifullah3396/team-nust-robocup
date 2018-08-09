/**
 * @file Utils/include/SwitchRequest.h
 *
 * This file defines the struct SwitchRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

/**
 * @class SwitchRequest
 * @brief A request to switch on and off some components
 */ 
struct SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchRequest(const bool& state) :
    state(state)
  {
  }
  
  bool state;
};
