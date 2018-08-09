/**
 * @file PlanningModule/PlanningBehaviors/GetupSequence.h
 *
 * This file declares the class GetupSequence.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/GetupSequence.h"

void
GetupSequence::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PBGetupSequenceConfig > (behaviorConfig));
}

boost::shared_ptr<PBGetupSequenceConfig>
GetupSequence::getBehaviorCast()
{
  return boost::static_pointer_cast < PBGetupSequenceConfig > (behaviorConfig);
}

void
GetupSequence::initiate()
{
  auto& type = getBehaviorCast()->type;
  if (type == PBGetupSequenceTypes::SIMPLE_GETUP) {
    if (IVAR(bool, PlanningModule::robotFallen)) {
      inBehavior = true;
      return;
    }
  }
}

void
GetupSequence::update()
{
  PRINT("Executing GetupSequnce.update()...")
  if (!inBehavior) return;
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState() || !reqChildBehaviorState()) return;
  updatePostureAndStiffness();
  if (!IVAR(bool, PlanningModule::robotFallen)) {
    inBehavior = false;
    return;
  }
  if (posture == PostureState::FALLING_FRONT) {
    fallSafetyFront();
  } else if (posture == PostureState::FALLING_BACK) {
    fallSafetyBack();
  } else if (posture == PostureState::FALL_FRONT) {
    getupFront();
  } else if (posture == PostureState::FALL_BACK) {
    getupBack();
  } else if (posture == PostureState::FALL_SIT) {
    getupSit();
  }
}

void
GetupSequence::fallSafetyFront()
{
}
void
GetupSequence::fallSafetyBack()
{
}
void
GetupSequence::getupSit()
{
}

void
GetupSequence::getupBack()
{
  PRINT("Executing GetupSequnce.update()...")
  if (!initialSetup) {
    cout << "initial setup done" << endl;
    if (stiffness != StiffnessState::GETUP) {
      if (lastSBFinished) {
        auto sConfig =
          boost::make_shared < SBStiffnessConfig > (SBStiffnessTypes::GETUP);
        setupSBRequest(sConfig);
      }
    } else if (posture != PostureState::GETUP_READY) {
      if (lastMBFinished) {
        auto pConfig = boost::make_shared<MBPostureConfig>();
        pConfig->timeToReachP = 1.0;
        pConfig->posture = PostureState::GETUP_READY;
        setupMBRequest(pConfig);
      }
    } else {
      initialSetup = true;
    }
  }
  cout << "Setting behavior" << endl;
  if (lastMBFinished) {
    auto getupConfig = boost::make_shared<MBGetupConfig>();
    getupConfig->type = MBGetupTypes::FALL_BACK;
    setupMBRequest(getupConfig);
  }
}

void
GetupSequence::getupFront()
{
  cout << "In getup front" << endl;
  if (!initialSetup) {
    cout << "Initial setup" << endl;
    if (stiffness != StiffnessState::GETUP) {
      if (lastSBFinished) {
        auto sConfig =
          boost::make_shared < SBStiffnessConfig > (SBStiffnessTypes::GETUP);
        setupSBRequest(sConfig);
      }
    } else if (posture != PostureState::GETUP_READY) {
      if (lastMBFinished) {
        auto pConfig = boost::make_shared<MBPostureConfig>();
        pConfig->timeToReachP = 1.0;
        pConfig->posture = PostureState::GETUP_READY;
        setupMBRequest(pConfig);
      }
    } else {
      initialSetup = true;
    }
  }

  cout << "Setting behavior" << endl;
  if (lastMBFinished) {
    auto getupConfig = boost::make_shared<MBGetupConfig>();
    getupConfig->type = MBGetupTypes::FALL_FRONT;
    setupMBRequest(getupConfig);
  }
}

void
GetupSequence::finishBehaviorSafely()
{
  inBehavior = false;
}

void
GetupSequence::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<PBGetupSequenceConfig>();
}
