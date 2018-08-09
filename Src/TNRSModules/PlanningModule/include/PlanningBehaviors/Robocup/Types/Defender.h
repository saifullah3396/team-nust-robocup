/**
 * @file PlanningModule/PlanningBehaviors/Defender.h
 *
 * This file declares the class Defender.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017 
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/** 
 * @class Defender
 * @brief Class for defining the robocup defender behavior.
 */
class Defender : public Robocup
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  Defender(PlanningModule* planningModule) :
    Robocup(planningModule, "Defender"),
    behaviorState(findItself)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~Defender()
  {
  }

  void initiate();
  void update();
  void finishBehaviorSafely();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);

private:
  boost::shared_ptr<DefenderConfig> getBehaviorCast();

  void defenderAction();
  
  unsigned behaviorState;
  enum BehaviorState { // For each behavior
    findItself,
    idleSequence,
    searchBall,
    playBall,
    alignToKick,
    kicking
  };
};

typedef boost::shared_ptr<Defender> DefenderPtr;
