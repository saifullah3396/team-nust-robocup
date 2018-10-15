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

void
ActuatorLayer::update()
{
  try {
    //cout << "actuator update " << alias << ":" << endl;
    //cout << "requests.size(): " << requests.size() << endl;
    if (requests.empty())
      return;
    //! Get the earliest request in queue
    auto request = requests.front(); 
    for (size_t i = 0; i < num; ++i)
      commands[5][i][0] = (request->getValue())[i];
    commands[4][0] = dcmProxy->getTime(0);
    //cout << "commands:\n" << commands << endl;
    //cout << "set request" << endl;
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
  commands[1] = string("Merge");
  commands[2] = string("time-separate");
  commands[3] = 0;
  commands[4].arraySetSize(1);
  commands[5].arraySetSize(num);
  for (size_t i = 0; i < num; ++i)
    commands[5][i].arraySetSize(1);
}
