/**
 * @file PlanningModule/PlanningBehaviors/RobocupPenalties.h
 *
 * This file declares the class RobocupPenalties.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/**
 * @class RobocupPenalties
 * @brief The class for defining the robocup penalties behavior
 */
class RobocupPenalties : public Robocup
{
public:
  /**
   * Default constructor for this class.
   *
   * @param planningModule: pointer to parent.
   */
  RobocupPenalties(PlanningModule* planningModule) :
    Robocup(planningModule, "RobocupPenalties"),
    behaviorState(penaltyCfg),
    striker(false)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~RobocupPenalties()
  {
  }
  ;

  void initiate();
  void update();
  void finishBehaviorSafely();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);

private:
  boost::shared_ptr<PenaltiesConfig> getBehaviorCast();
  
  void penaltyCfgAction();
  void startupAction();
  void strikerAction();
  void goalKeeperAction();

  bool striker;

  unsigned behaviorState;
  enum BehaviorState { // For each behavior
    penaltyCfg,
    startup,
    play
  };
};

typedef boost::shared_ptr<RobocupPenalties> RobocupPenaltiesPtr;
