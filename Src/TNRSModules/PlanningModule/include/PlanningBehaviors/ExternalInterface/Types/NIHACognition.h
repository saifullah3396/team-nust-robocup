/**
 * @file PlanningModule/ExternalInterface/Types/NIHACognition.h
 *
 * This file declares the class NIHACognition
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"

/**
 * @class NIHACognition
 * @brief The class for defining a interface to interact with the NIHA
 *   cognition module
 */
class NIHACognition : public ExternalInterface
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  NIHACognition(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    ExternalInterface(planningModule, config, "NIHACognition"),
    behaviorState(waitForConn)
  {
  }

  /**
   * Destructor
   */
  ~NIHACognition()
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
   * * Returns the config casted as NIHACognitionConfigPtr
   */ 
  NIHACognitionConfigPtr getBehaviorCast();

  /**
   * Wait for connection state action
   */ 
  void waitForConnAction();
  
  /**
   * Send header state action
   */ 
  void sendHeaderAction();

  /**
   * Send data state action
   */ 
  void sendDataAction();

  /**
   * Data generator
   */ 
  void dataToString(string& data);

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
    sendHeader,
    sendData
  };
};

typedef boost::shared_ptr<NIHACognition> NIHACognitionPtr;
