/**
 * @file PlanningModule/PlanningBehaviors/RobocupTest.h
 *
 * This file declares the class RobocupTest.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "ConfigManager/include/ConfigMacros.h"
#include "DebugModule/include/DebugBase.h"
#include "PlanningModule/include/PlanningBehavior.h"

/** 
 * @class RobocupTest
 * @brief The class for defining test sequences for all the robocup 
 *   behaviors.
 */
class RobocupTest : public PlanningBehavior, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to enable any kind of debugging.
  (int, debug, 1),
  //! Option to chnage behavior type on runtime.
  (int, forcedBehaviorType, -1),
)
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  RobocupTest(PlanningModule* planningModule) :
    PlanningBehavior(planningModule, "RobocupTest"), 
    DebugBase("RobocupTest", this)
  {
    initDebugBase();
    loadInitialConfig();
    firstGoalReached = false;
    secondGoalReached = false;
    thirdGoalReached = false;
    set = false;
    aligned = false;
    followBallKilled = false;
    startup = false;
  }

  /**
   * Default destructor for this class.
   */
  ~RobocupTest()
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
  boost::shared_ptr<PBRobocupTestConfig>
  getBehaviorCast();
  string
  getName() {
    return "RobocupTest";
  }
private:
  bool startup;
  bool firstGoalReached;
  bool secondGoalReached;
  bool thirdGoalReached;
  bool set;
  bool aligned;
  bool followBallKilled;
};

typedef boost::shared_ptr<RobocupTest> RobocupTestPtr;
