/**
 * @file PlanningModule/KickSequences/BallIntercept.h
 *
 * This file declares the class BallIntercept
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/KickSequence/KickSequence.h"

/**
 * @class BallIntercept
 * @brief The class for defining a ball intercepting kick sequence
 */
class BallIntercept : public KickSequence
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  BallIntercept(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    KickSequence(planningModule, config, "BallIntercept")
  {
  }

  /**
   * Destructor
   */
  ~BallIntercept()
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
   * * Returns the config casted as BallInterceptConfigPtr
   */ 
  BallInterceptConfigPtr getBehaviorCast();
  
  /**
   * Startup state action
   */ 
  void startupAction();
  
  /**
   * Ball incoming state action
   */ 
  void ballIncomingAction();

  //! Current behavior state
  unsigned behaviorState;
  
  /**
   * All the possible states of this behavior
   * 
   * @enum BehaviorState
   */ 
  enum BehaviorState
  {
    startup,
    ballIncoming
  };
};

typedef boost::shared_ptr<BallIntercept> BallInterceptPtr;
