/**
 * @file ControlModule/include/ControlModule.h
 *
 * This file declares the class ControlModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#pragma once

#include "Utils/include/ConfigMacros.h"
#include "ControlModule/include/HardwareLayer.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/RoboCupGameControlData.h"

/**
 * @class ControlModule
 * @brief This class defines the outer loop controller for interfacing
 *   with NaoQi lower-level device communication manager (DCM) and
 *   ALMemory for sensor extraction and actuation
 */
class ControlModule : public BaseModule
{
CREATE_INPUT_CONNECTOR(ControlInput,
  (int, controlThreadPeriod),
  (int, playerNumber),
  (int, teamNumber),
  (int, teamPort),
  (int, teamColor),
)CREATE_OUTPUT_CONNECTOR(ControlOutput,
  (vector<float>, jointPositionSensors),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, jointTemperatureSensors),
  (vector<float>, jointCurrentSensors),
  (vector<float>, touchSensors),
  (vector<float>, switchSensors),
  (vector<float>, batterySensors),
  (vector<float>, inertialSensors),
  (vector<float>, sonarSensors),
  (vector<float>, fsrSensors),
  (vector<float>, ledSensors),
  (RoboCupGameControlData, gameData),
  (int, nFootsteps),
)
public:
  /**
   * Initializes the controller module for interface with lower-level
   * NaoQi architecture (DCM)
   *
   * @param teamNUSTSPL: pointer to base class which is in this
   *   case TeamNUSTSPL
   */
  ControlModule(void* teamNUSTSPL, const ALMemoryProxyPtr& memoryProxy,
    const ALDCMProxyPtr& dcmProxy);

  /**
   * Destructor
   */
  ~ControlModule()
  {
    delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void
  init();

  /**
   * Derived from BaseModule
   */
  void
  handleRequests();

  /**
   * Derived from BaseModule
   */
  void
  mainRoutine();

  /**
   * Derived from BaseModule
   */
  void
  initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void
  setThreadPeriod();

  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   *
   * @return void
   */
  void
  sensorsUpdate();

  /**
   * Sends the requested actuator commands to NaoQi DCM for execution
   *
   * @return void
   */
  void
  actuatorsUpdate();

private:
  /**
   * Initializes a sensor handle
   *
   * @param sensorIndex: index of the sensor layer object
   * @param sensorHandle: pointer to the object recieving sensor values
   * @return void
   */
  void
  createSensorHandle(const unsigned& sensorIndex,
    const vectorFloatPtr& sensorHandle);

  /**
   * Initializes an actuator layer
   *
   * @param actuatorIndex: index of the actuator layer object
   */
  void
  createActuatorLayer(const unsigned& actuatorIndex);

  /**
   * Sets up the team configuration data to be inserted in ALMemory
   * for robocup game controller
   *
   * @return void
   */
  void
  setupRoboCupDataHandler();

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Vector of pointer to ActuatorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers;

  //! Pointer to NaoQi internal memory proxy
  ALMemoryProxyPtr memoryProxy;

  //! Pointer to NaoQi internal dcm proxy
  ALDCMProxyPtr dcmProxy;
};
