/**
 * @file PlanningModule/PlanningBehaviors/GoalKeeper.h
 *
 * This file declares the class GoalKeeper.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017 
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/** 
 * @class GoalKeeper
 * @brief Class for defining the goal keeper behavior.
 */
class GoalKeeper : public Robocup
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  GoalKeeper(PlanningModule* planningModule) :
    Robocup(planningModule, "GoalKeeper"),
    behaviorState(correctPosture),
    lastInterceptTarget(RobotPose2D<float>(-100, -100, -100)),
    lastDiveTargetType(-1)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~GoalKeeper()
  {
  }

  void initiate();
  void update();
  void finishBehaviorSafely();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);

private:
  boost::shared_ptr<GoalKeeperConfig> getBehaviorCast();
  void goalKeeperAction();
  void correctPostureAction();
  bool shouldLocalize();
  void findItselfAction();
  void correctPositionAction();
  void waitForMoveAction();
  bool shouldSearchBall();
  void searchBallAction();
  void saveGoalAction();
  void interceptBallAction();
  bool divePossible();
  void diveAction();
  void diveFollowUpAction();
  bool shouldPlayBall();
  void playBallAction();
  bool goToBall(const float& distFromBall, const float& angle);
  bool findBallKickTarget();
  void alignToKickAction();
  void kickBallAction();
  void findBestBallAlignment(RobotPose2D<float>& alignPosition);
  bool behindObstacle(const Point2f& target);

  RobotPose2D<float> lastInterceptTarget;
  int lastDiveTargetType;

  unsigned behaviorState;
  enum BehaviorState { // For each behavior
    correctPosture,
    findItself,
    correctPosition,
    waitForMove,
    searchBall,
    saveGoal,
    dive,
    diveFollowUp,
    playBall,
    interceptBall,
    alignToKick,
    kickBall
  };
};

typedef boost::shared_ptr<GoalKeeper> GoalKeeperPtr;
