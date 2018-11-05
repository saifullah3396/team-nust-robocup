/**
 * @file PlanningModule/PlanningBehaviors/GetupSequence.h
 *
 * This file declares the class GetupSequence.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/**
 * @class GetupSequence
 * @brief The class for defining the getup sequence
 */
class GetupSequence : public PlanningBehavior
{
public:
  /**
   * Default constructor for this class.
   *
   * @param planningModule: pointer to parent.
   */
  GetupSequence(PlanningModule* planningModule) :
    PlanningBehavior(planningModule, "GetupSequence"), initialSetup(false)
  {
    loadInitialConfig();
  }

  /**
   * Default destructor for this class.
   */
  ~GetupSequence()
  {
  }
  ;

  void initiate();
  void update();
  void finishBehaviorSafely();
  void loadInitialConfig();
  void setBehaviorConfig(
    boost::shared_ptr<BehaviorConfig> behaviorConfig);
  boost::shared_ptr<PBGetupSequenceConfig> getBehaviorCast();

private:
  void fallSafetyFront();
  void fallSafetyBack();
  void getupSit();
  void getupFront();
  void getupBack();

  bool initialSetup;
};

typedef boost::shared_ptr<GetupSequence> GetupSequencePtr;
