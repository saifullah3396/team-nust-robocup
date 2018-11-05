/**
 * @file LocalizationModule/src/LocalizationModule.cpp
 *
 * This file declares a class for the complete robot localization. 
 * All the functions and algorithms for robot state estimation and
 * pose determination will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 Feb 2017
 */

#include "LocalizationModule/include/LocalizationModule.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "LocalizationModule/include/FieldMap.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"

LocalizationModule::LocalizationModule(void* processingModule) :
    BaseModule(
      processingModule,
      (unsigned) TNSPLModules::LOCALIZATION,
      "LocalizationModule"), 
    runLocalization(false)
{
}

void
LocalizationModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, LocalizationModule::localizationThreadPeriod));
}

void
LocalizationModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void
LocalizationModule::init()
{
  particleFilter = 
    boost::shared_ptr <ParticleFilter> (new ParticleFilter(this));
  fieldMap = boost::make_shared < FieldMap > (this);
  particleFilter->init(RobotPose2D<float>(-4.25f, 0.f, 0));
}

void
LocalizationModule::handleRequests()
{
  while (!inRequests.isEmpty()) {
    auto request = inRequests.queueFront();
    if (boost::static_pointer_cast <LocalizationRequest>(request)) {
      auto reqId = request->getId();
      if (reqId == (unsigned)LocalizationRequestIds::SWITCH_LOCALIZATION) {
        auto sl = boost::static_pointer_cast <SwitchLocalization>(request);
        runLocalization = sl->state;
      } else if (reqId == (unsigned)LocalizationRequestIds::SWITCH_BALL_OBSTACLE) {
        auto sbo = boost::static_pointer_cast <SwitchBallObstacle>(request);
        fieldMap->setBallObstacle(sbo->state);
      } else if (reqId == (unsigned)LocalizationRequestIds::RESET_LOCALIZER) {
        particleFilter->reset();
      } else if (reqId == (unsigned)LocalizationRequestIds::POSITION_UPDATE) {
        auto pu = boost::static_pointer_cast <PositionUpdate>(request);
        particleFilter->addPositionInput(pu->input);
      } else if (reqId == (unsigned)LocalizationRequestIds::KNOWN_LANDMARKS_UPDATE) {
        auto klu = boost::static_pointer_cast <KnownLandmarksUpdate>(request);
        particleFilter->setKnownLandmarks(klu->landmarks);
      } else if (reqId == (unsigned)LocalizationRequestIds::UNKNOWN_LANDMARKS_UPDATE) {
        auto ulu = boost::static_pointer_cast <UnknownLandmarksUpdate>(request);
        particleFilter->setUnknownLandmarks(ulu->landmarks);
      }
    }
    inRequests.popQueue();
  }
}

void
LocalizationModule::mainRoutine()
{
  // Execution of this module is decided by planning module.
  if (runLocalization) {
    particleFilter->update();
    fieldMap->update();
    OVAR(bool, LocalizationModule::robotLocalized) =
      particleFilter->getLocalized();
  }
}
