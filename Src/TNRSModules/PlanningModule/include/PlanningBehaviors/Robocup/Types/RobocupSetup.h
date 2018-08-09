/**
 * @file PlanningModule/PlanningBehaviors/RobocupSetup.h
 *
 * This file declares the class RobocupSetup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Jul 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/**
 * @class RobocupSetup
 * @brief The class for defining the robocup startup sequence
 */
class RobocupSetup : public Robocup
{
public:
  /**
   * Default constructor for this class.
   *
   * @param planningModule: pointer to parent.
   */
  RobocupSetup(PlanningModule* planningModule) :
    Robocup(planningModule, "RobocupSetup"),
    behaviorState(startup),
    setSequenceFinished(false)
  {   
  }

  /**
   * Default destructor for this class.
   */
  ~RobocupSetup()
  {
  }

  void initiate();
  void update();
  void finishBehaviorSafely();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);

private:
  boost::shared_ptr<RobocupSetupConfig> getBehaviorCast();
  
  void startupAction();
  void cfgHandlingAction();
  void readySequenceAction();
  void getInPositionsAction();
  void setSequenceAction();
  void gameplaySequenceAction();

  bool setSequenceFinished;
  unsigned behaviorState;
  enum BehaviorState {
    startup,
    robocupCfg,
    readySequence,
    getInPositions,
    setSequence,
    gameplaySequence
  };
};

typedef boost::shared_ptr<RobocupSetup> RobocupSetupPtr;
