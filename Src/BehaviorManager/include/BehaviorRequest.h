/**
 * @file Utils/include/BehaviorRequest.h
 *
 * This file declares and implements the class BehaviorRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include "BehaviorManager/include/BehaviorManager.h"
#include "BehaviorManager/include/BehaviorConfig.h"

/**
 * @struct BehaviorRequest
 * @brief The base class for creating behavior requests.
 */
class BehaviorRequest
{
public:
  /**
   * Constructor
   * 
   * @param config: Configuration of the requested behavior
   */
  BehaviorRequest(
    const BehaviorConfigPtr& config) : 
      config(config), accepted(false), received(false)
  {
  }

  BehaviorConfigPtr getReqConfig() const { return config; }
  bool getAccepted() const { return accepted; }
  bool isReceived() const { return received; }

private:
  /**
   * Sets there received variable to the given value
   * 
   * @param recieved: Required variable value
   */ 
  void setReceived(const bool& received) { 
    this->received = received;
  }

  /**
   * Sets the accepted variable to the given value
   * 
   * @param accepted: Required variable value
   */ 
  void setAccepted(const bool& accepted) { 
    this->accepted = accepted;
  }

  //! The request is accepted or not
  bool accepted;
  
  //! The request is received or not
  bool received;
  
  //! Vector of configuration of all the requested behaviors.
  BehaviorConfigPtr config;
  
  //! Behavior manager can access its private members
  friend class BehaviorManager;
};

typedef boost::shared_ptr<BehaviorRequest> BehaviorRequestPtr;
