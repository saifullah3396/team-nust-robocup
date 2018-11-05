/**
 * @file PlanningModule/PlanningBehaviors/DatasetExtractor.h
 *
 * This file declares the class DatasetExtractor.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/DatasetExtractor.h"

void
DatasetExtractor::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PBDatasetExtractorConfig > (behaviorConfig));
}

boost::shared_ptr<PBDatasetExtractorConfig>
DatasetExtractor::getBehaviorCast()
{
  return boost::static_pointer_cast < PBDatasetExtractorConfig > (behaviorConfig);
}

void
DatasetExtractor::finishBehaviorSafely()
{
  inBehavior = false;
}

void
DatasetExtractor::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<PBDatasetExtractorConfig>();
}

void
DatasetExtractor::initiate()
{
  LOG_INFO("DatasetExtractor behavior initiated...")
  OVAR(bool, PlanningModule::runVisionModule) = true;
  inBehavior = true;
}

void
DatasetExtractor::update()
{
  //LOG_INFO("Executing DatasetExtractor.update()...")
  if (!inBehavior) return;
  updatePostureAndStiffness();
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState() || !reqChildBehaviorState()) return;
  if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == headTapWait) {
	headTapWaitAction();  
  } else if (behaviorState == videoStart) {
    videoStartAction();
  } else if (behaviorState == videoStop) {
    videoStopAction();
  }
}

void
DatasetExtractor::startupAction()
{
  //LOG_INFO("Executing DatasetExtractor.startupAction()...")
  if (posture == PostureState::STAND && stiffness == StiffnessState::ROBOCUP) {
    behaviorState = headTapWait;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (lastSBFinished) {
      LOG_INFO("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared < SBStiffnessConfig > (SBStiffnessTypes::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND) {
    if (lastMBFinished) {
      LOG_INFO("Setting posture to stand at startup.")
      auto pConfig = boost::make_shared<MBPostureConfig>();
      pConfig->timeToReachP = 2.f;
      pConfig->posture = PostureState::STAND;
      setupMBRequest(pConfig);
    }
  }
}

void
DatasetExtractor::headTapWaitAction()
{
  //LOG_INFO("Executing DatasetExtractor.headTapWaitAction()...")
  bool headTapped =
    IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
  if(true) {
	//OVAR(int, PlanningModule::writeVideo) = 1; // -1, 0, 1, 2
    behaviorState = videoStart;
  }
}

void
DatasetExtractor::videoStartAction()
{
  //LOG_INFO("Executing DatasetExtractor.videoStartAction()...")
  bool headTapped =
    IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
  if(headTapped) {
	OVAR(int, PlanningModule::writeVideo) = -1;
    behaviorState = videoStop;
  }
}


void
DatasetExtractor::videoStopAction()
{
  //LOG_INFO("Executing DatasetExtractor.videoStopAction()...")
  bool headTapped =
    IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
  if(headTapped) {
	OVAR(int, PlanningModule::writeVideo) = 0; // -1, 0, 1, 2
    behaviorState = videoStart;
  }
}
