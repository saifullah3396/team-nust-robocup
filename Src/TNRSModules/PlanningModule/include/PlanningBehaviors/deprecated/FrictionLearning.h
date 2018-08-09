/**
 * @file PlanningModule/PlanningBehaviors/FrictionLearning.h
 *
 * This file declares the class FrictionLearning.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "ConfigManager/include/ConfigMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"

/** 
 * @class FrictionLearning
 * @brief The class for defining friction learning behavior.
 */
class FrictionLearning : public PlanningBehavior
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  FrictionLearning(PlanningModule* planningModule) :
    PlanningBehavior(planningModule, "FrictionLearning")
  {
    loadInitialConfig();
    behaviorState = startup;
    lastReqVelocity = Point2f(0.25f, 0);
  }

  /**
   * Default destructor for this class.
   */
  ~FrictionLearning()
  {
  }
  ;

  void
  initiate();
  void
  update();
  void
  finishBehaviorSafely();
  void
  loadInitialConfig();
  void
  setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);
  boost::shared_ptr<PBFrictionLearningConfig>
  getBehaviorCast();
  
private:
  void
  startupAction();
  void
  waitForHeadTapAction();
  void
  searchBallAction();
  void
  alignToKickAction();
  void
  kickBallAction();
  bool
  shouldSearchBall();
  void
  findBestBallAlignment(RobotPose2D<float>& alignPosition);

  unsigned behaviorState;
  Point2f lastReqVelocity;

  enum BehaviorState
  {
    startup,
    waitForHeadTap,
    searchBall,
    alignToKick,
    kickBall
  };
};

typedef boost::shared_ptr<FrictionLearning> FrictionLearningPtr;
