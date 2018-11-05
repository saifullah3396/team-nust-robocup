/**
 * @file ControlModule/src/ControlModule.cpp
 *
 * This file implements the class ControlModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "ControlModule/include/ControlModule.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"

ControlModule::ControlModule(void* teamNUSTSPL,
  const ALMemoryProxyPtr& memoryProxy, const ALDCMProxyPtr& dcmProxy) :
  BaseModule(teamNUSTSPL, (unsigned) TNSPLModules::CONTROL, "ControlModule"),
  memoryProxy(memoryProxy), dcmProxy(dcmProxy)
{
}

void
ControlModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, controlThreadPeriod));
}

void
ControlModule::initMemoryConn()
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
ControlModule::init()
{
  try {
#ifdef MODULE_IS_REMOTE
    RoboCupGameControlData gameCtrlData;
    AL::ALValue value((const char*) &gameCtrlData, sizeof(gameCtrlData));
    memoryProxy->insertData("GameCtrl/RoboCupGameControlData", value);
#endif
    sensorLayers.resize(NUM_SENSOR_TYPES);
    actuatorLayers.resize(NUM_ACTUATOR_TYPES);
    createSensorHandle(
      JOINTS + POSITION,
      OVAR_PTR(vector<float>, jointPositionSensors));
    createSensorHandle(
      JOINTS + TEMPERATURE,
      OVAR_PTR(vector<float>, jointTemperatureSensors));
    createSensorHandle(
      JOINTS + CURRENT,
      OVAR_PTR(vector<float>, jointCurrentSensors));
    createSensorHandle(
      JOINTS + HARDNESS_SENSOR,
      OVAR_PTR(vector<float>, jointStiffnessSensors));
    createSensorHandle(TOUCH_SENSORS, OVAR_PTR(vector<float>, touchSensors));
    createSensorHandle(SWITCH_SENSORS, OVAR_PTR(vector<float>, switchSensors));
    createSensorHandle(
      BATTERY_SENSORS,
      OVAR_PTR(vector<float>, batterySensors));
    createSensorHandle(
      INERTIAL_SENSORS,
      OVAR_PTR(vector<float>, inertialSensors));
    createSensorHandle(SONAR_SENSORS, OVAR_PTR(vector<float>, sonarSensors));
    createSensorHandle(FSR_SENSORS, OVAR_PTR(vector<float>, fsrSensors));
    createSensorHandle(LED_SENSORS, OVAR_PTR(vector<float>, ledSensors));
    PRINT("Initialized sensor handles.")
    createActuatorLayer(JOINT_ACTUATORS + ANGLES);
    createActuatorLayer(JOINT_ACTUATORS + HARDNESS);
    createActuatorLayer(LED_ACTUATORS);
    PRINT("Initialized actuator handles.")
    //#ifdef MODULE_IS_REMOTE
    //((TeamNUSTSPL*) teamNUSTSPL)->
    //getSharedMemoryProxy->
    //subscribeToEvent("atPostProcess", "TeamNUSTSPL", "sensorUpdate");
    //((TeamNUSTSPL*) teamNUSTSPL)->
    //getSharedMemoryProxy->
    //subscribeToEvent("atPreProcess", "TeamNUSTSPL", "actuatorUpdate");
    //#else
    //((TeamNUSTSPL*) teamNUSTSPL)->
    //atPostProcess(boost::bind(&ControlModule::sensorsUpdate));
    //((TeamNUSTSPL*) teamNUSTSPL)->
    //atPreProcess(boost::bind(&ControlModule::actuatorsUpdate));
    //#endif
    setupRoboCupDataHandler();
    PRINT("Initialized RobocupDataHandler.")
    sensorsUpdate();
  } catch (const exception& e) {
    ERROR(e.what())
  }
}

void
ControlModule::handleRequests()
{
  //PRINT("ControlModule.handleRequests()")
  //static float headJoint = 0;
  //auto jointRequest = boost::make_shared<JointRequest>();
  //jointRequest->setValue(headJoint, 0);
  //cout << jointRequest->getValue() << endl;
  //inRequests.pushToQueue(jointRequest); 
  //auto ledRequest = boost::make_shared<LedRequest>();
  //ledRequest->setValue(0, 0);
  //cout << ledRequest->getValue() << endl;
  //inRequests.pushToQueue(ledRequest); 
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <ControlRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)ControlRequestIds::JOINT_REQUEST) {
      actuatorLayers[JOINT_ACTUATORS + ANGLES]->addRequest(
        boost::static_pointer_cast<ActuatorRequest>(request)
      );
    } else if (reqId == (unsigned)ControlRequestIds::STIFFNESS_REQUEST) {
      actuatorLayers[JOINT_ACTUATORS + HARDNESS]->addRequest(
        boost::static_pointer_cast<ActuatorRequest>(request)
      );
    } else if (reqId == (unsigned)ControlRequestIds::LED_REQUEST) {
      actuatorLayers[LED_ACTUATORS]->addRequest(
        boost::static_pointer_cast<ActuatorRequest>(request)
      );
    }
  }
  //headJoint += M_PI / 180.f;
  inRequests.popQueue();
}

void
ControlModule::mainRoutine()
{
  //auto tStart = high_resolution_clock::now();
  sensorsUpdate();
  actuatorsUpdate();
  //duration<double> timeSpan = high_resolution_clock::now() - tStart;
  //PRINT("ControlModule.mainRoutine() Time: " << timeSpan.count() << "seconds.");
}

void
ControlModule::sensorsUpdate()
{
  try {
    for (size_t i = 0; i < sensorLayers.size(); ++i) {
      if ( i == 1 || i == 2 || i == 10)
        continue;
      if (sensorLayers[i]) sensorLayers[i]->update();
    }
    float temp = memoryProxy->getData("Motion/Walk/NbStep");
    OVAR(int, ControlModule::nFootsteps) = static_cast<int>(temp);
    RoboCupGameControlData gameControlData;
    AL::ALValue value = memoryProxy->getData("GameCtrl/RoboCupGameControlData");
    if (value.isBinary() && value.getSize() == sizeof(RoboCupGameControlData)) memcpy(
      &gameControlData,
      value,
      sizeof(RoboCupGameControlData));
    OVAR(RoboCupGameControlData, ControlModule::gameData) = gameControlData;
  } catch (exception &e) {
    ERROR(e.what())
  }
}

void
ControlModule::actuatorsUpdate()
{
  try {
    for (size_t i = 0; i < actuatorLayers.size(); ++i) {
      actuatorLayers[i]->update();
    }
  } catch (exception &e) {
    ERROR(e.what())
  }
}

void
ControlModule::createSensorHandle(const unsigned& sensorIndex,
  const vectorFloatPtr& sensorHandle)
{
  try {
    if (!sensorLayers[sensorIndex]) {
      switch (sensorIndex) {
      case JOINTS + POSITION:
        sensorLayers[sensorIndex] =
          boost::make_shared < JointSensors > (memoryProxy, POSITION);
        break;
      case JOINTS + TEMPERATURE:
        sensorLayers[sensorIndex] =
          boost::make_shared < JointSensors > (memoryProxy, TEMPERATURE);
        break;
      case JOINTS + CURRENT:
        sensorLayers[sensorIndex] =
          boost::make_shared < JointSensors > (memoryProxy, CURRENT);
        break;
      case JOINTS + HARDNESS_SENSOR:
        sensorLayers[sensorIndex] =
          boost::make_shared < JointSensors > (memoryProxy, HARDNESS_SENSOR);
        break;
      case TOUCH_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < TouchSensors > (memoryProxy);
        break;
      case SWITCH_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < SwitchSensors > (memoryProxy);
        break;
      case BATTERY_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < BatterySensors > (memoryProxy);
        break;
      case INERTIAL_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < InertialSensors > (memoryProxy);
        break;
      case SONAR_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < SonarSensors > (memoryProxy);
        break;
      case FSR_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < FsrSensors > (memoryProxy);
        break;
      case LED_SENSORS:
        sensorLayers[sensorIndex] =
          boost::make_shared < LedSensors > (memoryProxy);
        break;
      }
      sensorLayers[sensorIndex]->setSensorHandle(sensorHandle);
    } else {
      PRINT("This sensor handle is already defined.")
    }
  } catch (exception &e) {
    ERROR(e.what())
  }
}

void
ControlModule::createActuatorLayer(const unsigned& actuatorIndex)
{
  try {
    if (!actuatorLayers[actuatorIndex]) {
      switch (actuatorIndex) {
      case JOINT_ACTUATORS + ANGLES:
        actuatorLayers[actuatorIndex] =
          boost::make_shared<JointActuators>(dcmProxy, ANGLES);
        break;
      case JOINT_ACTUATORS + HARDNESS:
        actuatorLayers[actuatorIndex] =
          boost::make_shared<JointActuators>(dcmProxy, HARDNESS);
        break;
      case LED_ACTUATORS:
        actuatorLayers[actuatorIndex] =
          boost::make_shared<LedActuators>(dcmProxy);
        break;
      }
    }
  } catch (exception &e) {
    ERROR(e.what())
  }
}

void
ControlModule::setupRoboCupDataHandler()
{
  memoryProxy->insertData(
    "GameCtrl/teamNumber",
    (int) IVAR(unsigned, ControlModule::teamNumber));
  memoryProxy->insertData(
    "GameCtrl/teamColour",
    (int) IVAR(unsigned, ControlModule::teamColor));
  memoryProxy->insertData(
    "GameCtrl/playerNumber",
    (int) IVAR(unsigned, ControlModule::playerNumber));
}
