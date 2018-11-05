/**
 * @file PlanningModule/PlanningBehaviors/MoveSequence.h
 *
 * This file declares the class MoveSequence.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/**
 * @class MoveSequence
 * @brief The class for defining different kinds of movements of the
 *   robot.
 */
class MoveSequence : public PlanningBehavior
{
public:
  /**
   * Default constructor for this class.
   *
   * @param planningModule: pointer to parent.
   */
  MoveSequence(PlanningModule* planningModule) :
    PlanningBehavior(planningModule, "MoveSequence")
  {
    loadInitialConfig();
    ballCounter = 0;
    failCounter = 0;
    walkDone = false;
  }

  /**
   * Default destructor for this class.
   */
  ~MoveSequence()
  {
  }
  ;

  void initiate();
  void update();
  void finishBehaviorSafely();
  void loadInitialConfig();
  void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);
  boost::shared_ptr<PBMoveSequenceConfig> getBehaviorCast();

private:
  int ballCounter;
  int failCounter;
  bool walkDone;
};

typedef boost::shared_ptr<MoveSequence> MoveSequencePtr;
