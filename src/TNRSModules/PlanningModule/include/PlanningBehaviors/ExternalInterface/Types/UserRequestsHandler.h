/**
 * @file PlanningModule/ExternalInterface/Types/UserRequestsHandler.h
 *
 * This file declares the class UserRequestsHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"

/**
 * @class UserRequestsHandler
 * @brief The class for defining a interface to interact with the NIHA
 *   cognition module
 */
class UserRequestsHandler : public ExternalInterface
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  UserRequestsHandler(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    ExternalInterface(planningModule, config, "UserRequestsHandler"),
    behaviorState(waitForConn)
  {
  }

  /**
   * Destructor
   */
  ~UserRequestsHandler()
  {
  }

  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  
private:
  /**
   * * Returns the config casted as UserRequestsHandlerConfigPtr
   */ 
  UserRequestsHandlerConfigPtr getBehaviorCast();

  /**
   * Wait for connection state action
   */ 
  void waitForConnAction();

  /**
   * On request state action
   */ 
  void onRequestAction();

  //! Behavior state
  unsigned behaviorState;
  
  /**
   * All the possible states of this behavior
   * 
   * @enum BehaviorState
   */ 
  enum BehaviorState
  {
    waitForConn,
    onRequest
  };
};

typedef boost::shared_ptr<UserRequestsHandler> UserRequestsHandlerPtr;
