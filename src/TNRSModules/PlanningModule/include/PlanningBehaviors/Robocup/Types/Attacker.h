/**
 * @file PlanningModule/PlanningBehaviors/Attacker.h
 *
 * This file declares the class Attacker.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017 
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/** 
 * @class Attacker
 * @brief Class for defining the robocup attacker behavior.
 */
class Attacker : public Robocup
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  Attacker(PlanningModule* planningModule) :
    Robocup(planningModule, "Attacker"),
    behaviorState(findItself)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~Attacker()
  {
  }

  void initiate();
  void update();
  void finishBehaviorSafely();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);

private:
  boost::shared_ptr<AttackerConfig> getBehaviorCast();
  void attackerAction();

  unsigned behaviorState;
  enum BehaviorState { // For each behavior
    findItself, // To be defined
  };
};

typedef boost::shared_ptr<Attacker> AttackerPtr;
