/**
 * @file ControlModule/HardwareLayer.cpp
 *
 * This file implements classes SensorLayer and ActuatorLayers,
 * and defines their child classes fir each type of sensors and 
 * actuators
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 June 2017
 */

#include "Utils/include/DebugUtils.h"
#include "ControlModule/include/HardwareLayer.h"

void
SensorLayer::update()
{
  try {
#ifdef MODULE_IS_REMOTE
    *sensorHandle = memoryProxy->getListData(keys);
#else
    for (size_t i = 0; i < num; ++i) {
      (*sensorHandle)[i] = *sensorPtrs[i];
    }
#endif
  } catch (const exception& e) {
    ERROR(e.what())
  }
}

SensorLayerPtr SensorLayer::makeSensorHandle(
  const unsigned& sensorIndex,
  const vectorFloatPtr& sensorHandle,
  const ALMemoryProxyPtr& memoryProxy)
{
  try {
    SensorLayerPtr sl;
    switch (sensorIndex) {
      case JOINTS + POSITION:
        sl = boost::make_shared < JointSensors > (memoryProxy, POSITION);
        break;
      case JOINTS + TEMPERATURE:
        sl = boost::make_shared < JointSensors > (memoryProxy, TEMPERATURE);
        break;
      case JOINTS + CURRENT:
        sl = boost::make_shared < JointSensors > (memoryProxy, CURRENT);
        break;
      case JOINTS + HARDNESS_SENSOR:
        sl = boost::make_shared < JointSensors > (memoryProxy, HARDNESS_SENSOR);
        break;
      case TOUCH_SENSORS:
        sl = boost::make_shared < TouchSensors > (memoryProxy);
        break;
      case SWITCH_SENSORS:
        sl = boost::make_shared < SwitchSensors > (memoryProxy);
        break;
      case BATTERY_SENSORS:
        sl = boost::make_shared < BatterySensors > (memoryProxy);
        break;
      case INERTIAL_SENSORS:
        sl = boost::make_shared < InertialSensors > (memoryProxy);
        break;
      case SONAR_SENSORS:
        sl = boost::make_shared < SonarSensors > (memoryProxy);
        break;
      case FSR_SENSORS:
        sl = boost::make_shared < FsrSensors > (memoryProxy);
        break;
      case LED_SENSORS:
        sl = boost::make_shared < LedSensors > (memoryProxy);
        break;
    }
    sl->setSensorHandle(sensorHandle);
    return sl;
  } catch (exception &e) {
    ERROR(e.what())
  }
}

void
ActuatorLayer::update()
{
  try {
    if (requests.empty())
      return;
    //! Get the earliest request in queue
    auto request = requests.front(); 
    for (size_t i = 0; i < num; ++i)
      commands[5][i][0] = (request->getValue())[i];
    commands[4][0] = dcmProxy->getTime(0);
    //cout << "commands:\n" << commands << endl;
    LOG_INFO("Setting actuator request...");
    dcmProxy->setAlias(commands);
    //! Execute in remove
    requests.pop();
  } catch (const exception& e) {
    ERROR(e.what())
  }
}

void
ActuatorLayer::setActuatorAlias()
{
  AL::ALValue commandAlias;
  commandAlias.arraySetSize(2);
  commandAlias[0] = string(alias + "Actuators");
  commandAlias[1].arraySetSize(num);
  for (size_t i = 0; i < num; ++i)
    commandAlias[1][i] = string(keys[i]);
  try {
    dcmProxy->createAlias(commandAlias);
  } catch (const exception& e) {
    ERROR(e.what())
  }
}

void
ActuatorLayer::setActuatorCommand()
{
  commands.arraySetSize(6);
  commands[0] = string(alias + "Actuators");
  commands[1] = string("ClearAll");
  commands[2] = string("time-separate");
  commands[3] = 0;
  commands[4].arraySetSize(1);
  commands[5].arraySetSize(num);
  for (size_t i = 0; i < num; ++i)
    commands[5][i].arraySetSize(1);
}

ActuatorLayerPtr ActuatorLayer::makeActuatorLayer(
  const unsigned& actuatorIndex,
  const ALDCMProxyPtr& dcmProxy)
{
  try {
    ActuatorLayerPtr al;
    switch (actuatorIndex)
    {
      case JOINT_ACTUATORS + ANGLES:
        al = boost::make_shared<JointActuators>(dcmProxy, ANGLES);
        break;
      case JOINT_ACTUATORS + HARDNESS:
        al = boost::make_shared<JointActuators>(dcmProxy, HARDNESS);
        break;
      case LED_ACTUATORS:
        al = boost::make_shared<LedActuators>(dcmProxy);
        break;
    }
    return al;
  } catch (exception &e) {
    ERROR(e.what())
  }
}
