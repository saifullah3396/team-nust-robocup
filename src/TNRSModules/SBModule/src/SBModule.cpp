/**
 *@file SBModule/src/SBModule.cpp
 *
 * This file implements the class SBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "SBModule/include/LedRequest.h"
#include "SBModule/include/SBModule.h"
#include "SBModule/include/SBConfigs.h"
#include "SBModule/include/StaticBehavior.h"
#include "SBModule/include/SBRequest.h"
#include "SBModule/include/StiffnessRequest.h"

SBModule::SBModule(
  void* parent,
  const ALMemoryProxyPtr& memoryProxy,
  const ALDCMProxyPtr& dcmProxy,
  const ALMotionProxyPtr& motionProxy) :
  BaseModule(parent, (unsigned) TNSPLModules::STATIC, "SBModule"),
  memoryProxy(memoryProxy),
  dcmProxy(dcmProxy),
  motionProxy(motionProxy)
{
}

void SBModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, SBModule::sbThreadPeriod));
}

void SBModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
		new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
		new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void SBModule::init()
{
  sbManager = boost::make_shared<SBManager>(this);
  //! Make new layers for sensors directly related with sbmodule
  sensorLayers.resize((unsigned)SBSensors::COUNT);
  sensorLayers[(unsigned)SBSensors::JOINT_STIFFNESS] =
    SensorLayer::makeSensorHandle(
      SensorTypes::JOINTS + JointSensorTypes::HARDNESS_SENSOR,
      OVAR_PTR(vector<float>, jointStiffnessSensors), memoryProxy);
  sensorLayers[(unsigned)SBSensors::LED] =
    SensorLayer::makeSensorHandle(LED_SENSORS, OVAR_PTR(vector<float>, ledSensors), memoryProxy);
  //! Update the sensors
  sensorsUpdate();
  //! Make new layers for actuators directly related with sbmodule
  actuatorLayers.resize((unsigned)SBActuators::COUNT);
  actuatorLayers[(unsigned)SBActuators::JOINT_STIFFNESS] =
    ActuatorLayer::makeActuatorLayer(
      ActuatorTypes::JOINT_ACTUATORS + JointActuatorTypes::HARDNESS,
      dcmProxy);
  actuatorLayers[(unsigned)SBActuators::LED] =
    ActuatorLayer::makeActuatorLayer(LED_ACTUATORS, dcmProxy);
}

void
SBModule::handleRequests()
{
  //LOG_INFO("SBModule.handleRequests()")
  //SBRequestPtr sbRequest = 
  //  boost::make_shared<RequestStaticBehavior>(
  //    boost::make_shared <SBStiffnessConfig>());
  //inRequests.pushToQueue(sbRequest);
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <SBRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)SBRequestIds::STIFFNESS_REQUEST) {
      actuatorLayers[(unsigned)SBActuators::JOINT_STIFFNESS]->addRequest(
        boost::static_pointer_cast<StiffnessRequest>(request)
      );
    } else if (reqId == (unsigned)SBRequestIds::LED_REQUEST) {
      actuatorLayers[(unsigned)SBActuators::LED]->addRequest(
        boost::static_pointer_cast<LedRequest>(request)
      );
    } else if (reqId == (unsigned)SBRequestIds::BEHAVIOR_REQUEST) {
      auto rsb = 
        boost::static_pointer_cast<RequestStaticBehavior>(request);
      sbManager->manageRequest(rsb);
    } else if (reqId == (unsigned)SBRequestIds::KILL_BEHAVIOR) {
      sbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void SBModule::mainRoutine()
{
  /*static bool once = false;
  if (!once) {
    auto stiffnessCfg = boost::make_shared <SBStiffnessConfig>();
    vector<unsigned> bgr(3);
    bgr[0] = 255;
    bgr[1] = 255;
    auto ledsCfg = boost::make_shared <SBLedsConfig>(
      LedGroups::L_FACE, 1.f, 1.f, bgr, SBLedsTypes::DIRECT_LEDS);
    IVAR(BehaviorRequestPtr, SBModule::sBehaviorRequest)->setReqConfig(ledsCfg);
    IVAR(BehaviorRequestPtr, SBModule::sBehaviorRequest)->assignId();
    once = true;
  }*/
  sensorsUpdate();
  sbManager->update();
  OVAR(BehaviorInfo, SBModule::sBehaviorInfo) = 
    sbManager->getBehaviorInfo();
  actuatorsUpdate();
}

void SBModule::sensorsUpdate()
{
  for (size_t i = 0; i < sensorLayers.size(); ++i) {
    sensorLayers[i]->update();
  }
}

void SBModule::actuatorsUpdate()
{
  for (size_t i = 0; i < actuatorLayers.size(); ++i) {
    actuatorLayers[i]->update();
  }
}
