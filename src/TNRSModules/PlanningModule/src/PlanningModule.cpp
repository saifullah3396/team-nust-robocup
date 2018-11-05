/**
 * @file PlanningModule/PlanningModule.h
 *
 * This file implements the class for behavior planning.
 * All the functions and algorithms for adding intelligence to the
 * robot will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/make_shared.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PlanningConfigs.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "Utils/include/HardwareIds.h"

PlanningModule::PlanningModule(void* parent, const ALMemoryProxyPtr& memoryProxy) :
  BaseModule(
    parent, 
    (unsigned) TNSPLModules::PLANNING, 
    "PlanningModule"
  ), memoryProxy(memoryProxy)
{
}

void PlanningModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, planningThreadPeriod));
}

void PlanningModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void PlanningModule::init()
{
  pbManager = boost::make_shared<PBManager>(this);
  OVAR(PlanningState, PlanningModule::planningState) = PlanningState::STARTUP;
  OVAR(int, PlanningModule::robocupRole) = -1;
  setupRoboCupDataHandler();
  sensorLayers.resize((unsigned)PlanningSensors::COUNT);
  sensorLayers[(unsigned)PlanningSensors::JOINT_TEMP] =
  SensorLayer::makeSensorHandle(
    SensorTypes::JOINTS + JointSensorTypes::TEMPERATURE,
    OVAR_PTR(vector<float>, jointTemperatureSensors),
    memoryProxy);
  sensorLayers[(unsigned)PlanningSensors::JOINT_CURRENT] =
  SensorLayer::makeSensorHandle(
    SensorTypes::JOINTS + JointSensorTypes::CURRENT,
    OVAR_PTR(vector<float>, jointCurrentSensors),
    memoryProxy);
  sensorLayers[(unsigned)PlanningSensors::TOUCH] =
    SensorLayer::makeSensorHandle(
      TOUCH_SENSORS,
      OVAR_PTR(vector<float>, touchSensors),
      memoryProxy);
  sensorLayers[(unsigned)PlanningSensors::SWITCH] =
    SensorLayer::makeSensorHandle(
      SWITCH_SENSORS,
      OVAR_PTR(vector<float>, switchSensors),
      memoryProxy);
  sensorLayers[(unsigned)PlanningSensors::BATTERY] =
    SensorLayer::makeSensorHandle(
      BATTERY_SENSORS,
      OVAR_PTR(vector<float>, batterySensors),
      memoryProxy);
  sensorsUpdate();
  LOG_INFO("Starting Robot Startup Behavior.")
  PlanningRequestPtr planningRequest = 
    boost::make_shared<RequestPlanningBehavior>(
      boost::make_shared <RequestBehaviorConfig>());
  addRequest(planningRequest); // publish to itself
}

void
PlanningModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <PlanningRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)PlanningRequestIds::BEHAVIOR_REQUEST) {
      auto rpb = 
        boost::static_pointer_cast<RequestPlanningBehavior>(request);
      pbManager->manageRequest(rpb);
    } else if (reqId == (unsigned)PlanningRequestIds::KILL_BEHAVIOR) {
      pbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void PlanningModule::mainRoutine()
{    
  sensorsUpdate();
  pbManager->update();
  OVAR(BehaviorInfo, PlanningModule::pBehaviorInfo) = 
    pbManager->getBehaviorInfo();
}

void PlanningModule::sensorsUpdate()
{
  for (size_t i = 0; i < sensorLayers.size(); ++i) {
    sensorLayers[i]->update();
  }
  RoboCupGameControlData gameControlData;
  AL::ALValue value = memoryProxy->getData("GameCtrl/RoboCupGameControlData");
  if (value.isBinary() && value.getSize() == sizeof(RoboCupGameControlData)) memcpy(
    &gameControlData,
    value,
    sizeof(RoboCupGameControlData));
  OVAR(RoboCupGameControlData, PlanningModule::gameData) = gameControlData;
}

void PlanningModule::setupRoboCupDataHandler()
{
  #ifdef MODULE_IS_REMOTE
  RoboCupGameControlData gameCtrlData;
  AL::ALValue value((const char*) &gameCtrlData, sizeof(gameCtrlData));
  memoryProxy->insertData("GameCtrl/RoboCupGameControlData", value);
  #endif
  memoryProxy->insertData(
    "GameCtrl/teamNumber",
    (int) IVAR(unsigned, PlanningModule::teamNumber));
  memoryProxy->insertData(
    "GameCtrl/teamColour",
    (int) IVAR(unsigned, PlanningModule::teamColor));
  memoryProxy->insertData(
    "GameCtrl/playerNumber",
    (int) IVAR(unsigned, PlanningModule::playerNumber));
}
