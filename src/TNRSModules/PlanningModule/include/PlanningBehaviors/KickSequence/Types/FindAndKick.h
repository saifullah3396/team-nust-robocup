/**
 * @file PlanningModule/KickSequences/FindAndKick.h
 *
 * This file declares the class FindAndKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

//#define SIMULATION

#include "PlanningModule/include/PlanningBehaviors/KickSequence/KickSequence.h"

/**
 * @class FindAndKick
 * @brief The class for defining a behavior sequence to find and kick a 
 *   ball
 */
class FindAndKick : public KickSequence
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  FindAndKick(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    KickSequence(planningModule, config, "FindAndKick"),
    behaviorState(headTapWait)
  {
    desKickVel = Point2f(0.5f, 0.f); // Default velocity
  }

  /**
   * Destructor
   */
  ~FindAndKick()
  {
  }

  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  void loadExternalConfig();
  
private:
  /**
   * * Returns the config casted as FindAndKickConfigPtr
   */ 
  FindAndKickConfigPtr getBehaviorCast();
  
  /**
   * Head tap wait state action
   */ 
  void headTapWaitAction();
  
  /**
   * Startup state action
   */ 
  void startupAction();
  
    /**
   * Search ball state action
   */ 
  void searchBallAction();
  
  /**
   * Kick state action
   */ 
  void kickAction();
  

  //! Current behavior state
  unsigned behaviorState;
  
  #ifdef SIMULATION  
  //! Ball damping coefficient
  static float coeffDamping;
  #else
  static float rollingFriction;
  #endif
  
  //! Kicking velocity desired 
  Point2f desKickVel;
  
  /**
   * All the possible states of this behavior
   * 
   * @enum BehaviorState
   */ 
  enum BehaviorState
  {
    headTapWait = 0,
    startup,
    searchBall,
    kick
  };
};

typedef boost::shared_ptr<FindAndKick> FindAndKickPtr;
