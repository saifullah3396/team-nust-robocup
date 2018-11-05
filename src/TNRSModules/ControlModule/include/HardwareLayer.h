/**
 * @file ControlModule/include/HardwareLayer.h
 *
 * This file declares classes SensorLayer and ActuatorLayers,
 * and defines their child classes for each type of sensors and 
 * actuators
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 June 2017
 */

#pragma once

#include <queue>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "ControlModule/include/ActuatorRequests.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/HardwareIds.h"

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;
typedef boost::shared_ptr<vector<float> > vectorFloatPtr;

/**
 * @class SensorLayer
 * @brief Defines a layer of sensor interface handles
 */
class SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  SensorLayer(const ALMemoryProxyPtr& memoryProxy) :
    memoryProxy(memoryProxy)
  {
  }

  /**
   * Destructor
   */
  ~SensorLayer()
  {
  }

  /**
   * Updates the sensor values from NaoQi ALMemory
   *
   * @return void
   */
  void
  update();

  /**
   * Sets the sensor pointer
   *
   * @param sensorHandle: shared pointer to the variable that stores
   *   the sensor values
   * @return void
   */
  void
  setSensorHandle(const vectorFloatPtr& sensorHandle)
  {
    this->sensorHandle = sensorHandle;
    this->sensorHandle->resize(num);
  }

  /**
   * Makes a handle for the given sensor layer and returns it
   *
   * @param sensorIndex: index of the sensor layer object
   * @param sensorHandle: pointer to the object recieving sensor values
   *
   * @return boost::shared_ptr<SensorLayer>
   */
  static boost::shared_ptr<SensorLayer> makeSensorHandle(
    const unsigned& sensorIndex,
    const vectorFloatPtr& sensorHandle,
    const ALMemoryProxyPtr& memoryProxy);

protected:
  /**
   * Sets the naoqi sensor pointers
   *
   * @return void
   */
  void
  setSensorPtr()
  {
  #ifndef MODULE_IS_REMOTE
    sensorPtrs.resize(num);
    for (size_t i = 0; i < num; ++i)
      sensorPtrs[i] = (float*) memoryProxy->getDataPtr(keys[i]);
  #endif
  }

  /**
   * Initializes the sensor class for given sensors based on their keys
   *
   * @return void
   */
  virtual void
  init() = 0;

  //! Vector of sensor keys
  vector<string> keys;

  //! Number of Sensors
  unsigned num;

private:
  //! Pointer to NaoQi internal memory
  ALMemoryProxyPtr memoryProxy;

  //! Extracted values of the sensors
  vectorFloatPtr sensorHandle;

  //! Pointers to sensors of NaoQi ALMemory
  vector<float*> sensorPtrs;
};

typedef boost::shared_ptr<SensorLayer> SensorLayerPtr;

/**
 * @class ActuatorLayer
 * @brief Defines a layer of actuator handles
 */
class ActuatorLayer
{
public:
  /**
   * Constructor
   *
   * @param dcmProxy: pointer to NaoQi's dcm proxy.
   */
  ActuatorLayer(
    const ALDCMProxyPtr& dcmProxy) :
    dcmProxy(dcmProxy)
  {
  }

  /**
   * Destructor
   */
  ~ActuatorLayer()
  {
  }

  /**
   * Sends the actuator requests to NaoQi DCM for execution
   *
   * @param request: The requested actuators
   */
  void update();

  /**
   * Makes a handle for the given actuator layer and returns it
   *
   * @param actuatorIndex: index of the actuator layer object
   */
  static boost::shared_ptr<ActuatorLayer>
  makeActuatorLayer(const unsigned& actuatorIndex, const ALDCMProxyPtr& dcmProxy);

  virtual void addRequest(const ActuatorRequestPtr& request) 
    { requests.push(request); }

protected:
  /**
   * Initializes the actuators command alias
   *
   * @return void
   */
  void
  setActuatorAlias();

  /**
   * Initializes actuator command object
   *
   * @return void
   */
  void
  setActuatorCommand();

  /**
   * Initializes actuator class for given actuators keys
   *
   * @return void
   */
  virtual void
  init() = 0;

  //! Vector to memory keys
  vector<string> keys;

  //! Number of actuators
  unsigned num;

  //! Unique actuation request command alias
  string alias;

private:
  queue<ActuatorRequestPtr> requests;

  //! NaoQi DCM architecture time
  float dcmTime;

  //! Commands sent to DCM
  AL::ALValue commands;
  
  //! Pointer to NaoQi device communication manager (DCM)
  ALDCMProxyPtr dcmProxy;
};

typedef boost::shared_ptr<ActuatorLayer> ActuatorLayerPtr;

/**
 * @class JointSensors
 * @brief Defines the handles for JointSensors
 */
class JointSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   * @param type: type of joint sensor
   */
  JointSensors(const ALMemoryProxyPtr& memoryProxy,
    const JointSensorTypes& type) :
    SensorLayer(memoryProxy), type(type)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~JointSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_JOINTS;
    string typeString;
    keys.clear();
    keys.resize(num);
    switch (type) {
    case POSITION:
      typeString = string("Position/Sensor/Value");
      break;
    case HARDNESS_SENSOR:
      typeString = string("Hardness/Actuator/Value"); //Only case where actuator is used as sensor.
      break;
    case TEMPERATURE:
      typeString = string("Temperature/Sensor/Value");
      break;
      //case TEMP_STATUS:
      //  typeString = string("Temperature/Sensor/Status");
      //  break;
    case CURRENT:
      typeString = string("ElectricCurrent/Sensor/Value");
      break;
    }

    keys[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/" + typeString);
    keys[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/" + typeString);
    keys[L_SHOULDER_ROLL] = string(
      "Device/SubDeviceList/LShoulderRoll/" + typeString);
    keys[L_SHOULDER_PITCH] = string(
      "Device/SubDeviceList/LShoulderPitch/" + typeString);
    keys[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/" + typeString);
    keys[L_ELBOW_ROLL] = string(
      "Device/SubDeviceList/LElbowRoll/" + typeString);
    keys[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/" + typeString);
    //!keys[lHandPosition] =
    //!string("Device/SubDeviceList/LHand/"  + typeString);
    keys[R_SHOULDER_ROLL] = string(
      "Device/SubDeviceList/RShoulderRoll/" + typeString);
    keys[R_SHOULDER_PITCH] = string(
      "Device/SubDeviceList/RShoulderPitch/" + typeString);
    keys[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/" + typeString);
    keys[R_ELBOW_ROLL] = string(
      "Device/SubDeviceList/RElbowRoll/" + typeString);
    keys[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/" + typeString);
    //!keys[rHandPosition] =
    //!string("Device/SubDeviceList/RHand/"  + typeString);
    keys[L_HIP_YAW_PITCH] = string(
      "Device/SubDeviceList/LHipYawPitch/" + typeString);
    keys[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/" + typeString);
    keys[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/" + typeString);
    keys[L_KNEE_PITCH] = string(
      "Device/SubDeviceList/LKneePitch/" + typeString);
    keys[L_ANKLE_PITCH] = string(
      "Device/SubDeviceList/LAnklePitch/" + typeString);
    keys[L_ANKLE_ROLL] = string(
      "Device/SubDeviceList/LAnkleRoll/" + typeString);
    keys[R_HIP_YAW_PITCH] = string(
      "Device/SubDeviceList/LHipYawPitch/" + typeString);
    //!LHipYawPitch = RHipYawPitch
    keys[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/" + typeString);
    keys[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/" + typeString);
    keys[R_KNEE_PITCH] = string(
      "Device/SubDeviceList/RKneePitch/" + typeString);
    keys[R_ANKLE_PITCH] = string(
      "Device/SubDeviceList/RAnklePitch/" + typeString);
    keys[R_ANKLE_ROLL] = string(
      "Device/SubDeviceList/RAnkleRoll/" + typeString);
  }

private:
  //! Type of the joint sensor
  JointSensorTypes type;
};

/**
 * @class TouchSensors
 * @brief Defines the handles for TouchSensors
 */
class TouchSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  TouchSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~TouchSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_TOUCH_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[HEAD_TOUCH_FRONT] = string(
      "Device/SubDeviceList/Head/Touch/Front/Sensor/Value");
    keys[HEAD_TOUCH_REAR] = string(
      "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value");
    keys[HEAD_TOUCH_MIDDLE] = string(
      "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value");
    keys[L_HAND_TOUCH_BACK] = string(
      "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value");
    keys[L_HAND_TOUCH_LEFT] = string(
      "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value");
    keys[L_HAND_TOUCH_RIGHT] = string(
      "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value");
    keys[R_HAND_TOUCH_BACK] = string(
      "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value");
    keys[R_HAND_TOUCH_LEFT] = string(
      "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value");
    keys[R_HAND_TOUCH_RIGHT] = string(
      "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value");
  }
};

/**
 * @class SwitchSensors
 * @brief Defines the handles for SwitchSensors
 */
class SwitchSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  SwitchSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~SwitchSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_SWITCH_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[CHEST_BOARD_BUTTON] = string(
      "Device/SubDeviceList/ChestBoard/Button/Sensor/Value");
    keys[L_FOOT_BUMPER_RIGHT] = string(
      "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value");
    keys[L_FOOT_BUMPER_LEFT] = string(
      "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value");
    keys[R_FOOT_BUMPER_RIGHT] = string(
      "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value");
    keys[R_FOOT_BUMPER_LEFT] = string(
      "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value");
  }
};

/**
 * @class BatterySensors
 * @brief Defines the handles for BatterySensors
 */
class BatterySensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  BatterySensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~BatterySensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_BATTERY_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[HEAD_CPU_TEMPERATURE] = string(
      "Device/SubDeviceList/Battery/Temperature/Sensor/Value");
    keys[BATTERY_CURRENT] = string(
      "Device/SubDeviceList/Battery/Current/Sensor/Value");
    keys[BATTERY_CHARGE] = string(
      "Device/SubDeviceList/Battery/Charge/Sensor/Value");
    keys[BATTERY_TEMPERATURE] = string(
      "Device/SubDeviceList/Battery/Temperature/Sensor/Value");
  }
};

/**
 * @class InertialSensors
 * @brief Defines the handles for InertialSensors
 */
class InertialSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  InertialSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~InertialSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_INERTIAL_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[GYROSCOPE_X] = string(
      "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value");
    keys[GYROSCOPE_Y] = string(
      "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value");
    keys[GYROSCOPE_Z] = string(
      "Device/SubDeviceList/InertialSensor/GyrZ/Sensor/Value");
    keys[TORSO_ANGLE_X] = string(
      "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
    keys[TORSO_ANGLE_Y] = string(
      "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
    keys[TORSO_ANGLE_Z] = string(
      "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
    keys[ACCELEROMETER_X] = string(
      "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
    keys[ACCELEROMETER_Y] = string(
      "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
    keys[ACCELEROMETER_Z] = string(
      "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
  }
};

/**
 * @class FsrSensors
 * @brief Defines the handles for FsrSensors
 */
class FsrSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  FsrSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~FsrSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_FSR_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[L_FOOT_FSR_FL] = string(
      "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value");
    keys[L_FOOT_FSR_FR] = string(
      "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value");
    keys[L_FOOT_FSR_RL] = string(
      "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value");
    keys[L_FOOT_FSR_RR] = string(
      "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value");
    keys[L_FOOT_TOTAL_WEIGHT] = string(
      "Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
    keys[L_FOOT_COP_X] = string(
      "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value");
    keys[L_FOOT_COP_Y] = string(
      "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value");
    keys[R_FOOT_FSR_FL] = string(
      "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value");
    keys[R_FOOT_FSR_FR] = string(
      "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value");
    keys[R_FOOT_FSR_RL] = string(
      "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value");
    keys[R_FOOT_FSR_RR] = string(
      "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value");
    keys[R_FOOT_TOTAL_WEIGHT] = string(
      "Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");
    keys[R_FOOT_COP_X] = string(
      "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value");
    keys[R_FOOT_COP_Y] = string(
      "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value");
  }
};

/**
 * @class SonarSensors
 * @brief Defines the handles for SonarSensors
 */
class SonarSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  SonarSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~SonarSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_SONAR_SENSORS;
    keys.clear();
    keys.resize(num);
    keys[L_US_SONAR] = string("Device/SubDeviceList/US/Left/Sensor/Value");
    keys[L_US_SONAR_1] = string("Device/SubDeviceList/US/Left/Sensor/Value1");
    keys[L_US_SONAR_2] = string("Device/SubDeviceList/US/Left/Sensor/Value2");
    keys[L_US_SONAR_3] = string("Device/SubDeviceList/US/Left/Sensor/Value3");
    keys[L_US_SONAR_4] = string("Device/SubDeviceList/US/Left/Sensor/Value4");
    keys[L_US_SONAR_5] = string("Device/SubDeviceList/US/Left/Sensor/Value5");
    keys[L_US_SONAR_6] = string("Device/SubDeviceList/US/Left/Sensor/Value6");
    keys[L_US_SONAR_7] = string("Device/SubDeviceList/US/Left/Sensor/Value7");
    keys[L_US_SONAR_8] = string("Device/SubDeviceList/US/Left/Sensor/Value8");
    keys[L_US_SONAR_9] = string("Device/SubDeviceList/US/Left/Sensor/Value9");
    keys[R_US_SONAR] = string("Device/SubDeviceList/US/Right/Sensor/Value");
    keys[R_US_SONAR_1] = string("Device/SubDeviceList/US/Right/Sensor/Value1");
    keys[R_US_SONAR_2] = string("Device/SubDeviceList/US/Right/Sensor/Value2");
    keys[R_US_SONAR_3] = string("Device/SubDeviceList/US/Right/Sensor/Value3");
    keys[R_US_SONAR_4] = string("Device/SubDeviceList/US/Right/Sensor/Value4");
    keys[R_US_SONAR_5] = string("Device/SubDeviceList/US/Right/Sensor/Value5");
    keys[R_US_SONAR_6] = string("Device/SubDeviceList/US/Right/Sensor/Value6");
    keys[R_US_SONAR_7] = string("Device/SubDeviceList/US/Right/Sensor/Value7");
    keys[R_US_SONAR_8] = string("Device/SubDeviceList/US/Right/Sensor/Value8");
    keys[R_US_SONAR_9] = string("Device/SubDeviceList/US/Right/Sensor/Value9");
  }
};

/**
 * @class LedSensors
 * @brief Defines the handles for LedSensors
 */
class LedSensors : public SensorLayer
{
public:
  /**
   * Constructor
   *
   * @param memoryProxy: pointer to NaoQi's memory proxy.
   */
  LedSensors(const ALMemoryProxyPtr& memoryProxy) :
    SensorLayer(memoryProxy)
  {
    init();
    setSensorPtr();
  }

  /**
   * Destructor
   */
  ~LedSensors()
  {
  }

protected:
  /**
   * Derived from SensorLayer
   */
  void
  init()
  {
    num = NUM_LED_ACTUATORS;
    keys.clear();
    keys.resize(num);
    keys[FACE_LED_RED_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/0Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/45Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/90Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/135Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/180Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/225Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/270Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/315Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/0Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/45Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/90Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/135Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/180Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/225Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/270Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/315Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/0Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/45Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/90Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/135Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/180Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/225Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/270Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/315Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/45Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/90Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/135Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/180Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/225Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/270Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/315Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/0Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/45Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/90Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/135Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/180Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/225Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/270Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/315Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/0Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/45Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/90Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/135Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/180Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/225Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/270Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/315Deg/Actuator/Value");
    keys[EARS_LED_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/0Deg/Actuator/Value");
    keys[EARS_LED_LEFT_36_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/36Deg/Actuator/Value");
    keys[EARS_LED_LEFT_72_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/72Deg/Actuator/Value");
    keys[EARS_LED_LEFT_108_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/108Deg/Actuator/Value");
    keys[EARS_LED_LEFT_144_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/144Deg/Actuator/Value");
    keys[EARS_LED_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/180Deg/Actuator/Value");
    keys[EARS_LED_LEFT_216_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/216Deg/Actuator/Value");
    keys[EARS_LED_LEFT_252_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/252Deg/Actuator/Value");
    keys[EARS_LED_LEFT_288_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/288Deg/Actuator/Value");
    keys[EARS_LED_LEFT_324_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/324Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/0Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_36_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/36Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_72_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/72Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_108_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/108Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_144_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/144Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/180Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_216_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/216Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_252_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/252Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_288_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/288Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_324_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/324Deg/Actuator/Value");
    keys[CHEST_BOARD_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Red/Actuator/Value");
    keys[CHEST_BOARD_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Green/Actuator/Value");
    keys[CHEST_BOARD_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Blue/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/0/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/1/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_2_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/2/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/0/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/1/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_2_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/2/Actuator/Value");
    keys[HEAD_LED_MIDDLE_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Middle/Right/0/Actuator/Value");
    keys[HEAD_LED_FRONT_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Right/0/Actuator/Value");
    keys[HEAD_LED_FRONT_RIGHT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Right/1/Actuator/Value");
    keys[HEAD_LED_FRONT_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Left/0/Actuator/Value");
    keys[HEAD_LED_FRONT_LEFT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Left/1/Actuator/Value");
    keys[HEAD_LED_MIDDLE_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Middle/Left/0/Actuator/Value");
    keys[L_FOOT_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Red/Actuator/Value");
    keys[L_FOOT_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Green/Actuator/Value");
    keys[L_FOOT_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Blue/Actuator/Value");
    keys[R_FOOT_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Red/Actuator/Value");
    keys[R_FOOT_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Green/Actuator/Value");
    keys[R_FOOT_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Blue/Actuator/Value");
  }
};

/**
 * @class JointActuators
 * @brief Defines the handles for JointActuators
 */
class JointActuators : public ActuatorLayer
{
public:
  /**
   * Constructor
   *
   * @param dcmProxy: pointer to NaoQi's dcm proxy.
   * @param type Joint actuator type
   */
  JointActuators(
    const ALDCMProxyPtr& dcmProxy, 
    const JointActuatorTypes& type) :
    ActuatorLayer(dcmProxy), type(type)
  {
    init();
  }

  /**
   * Destructor
   */
  ~JointActuators()
  {
  }

protected:
  /**
   * Derived from ActuatorLayer
   */
  void
  init()
  {
    string typeString;
    num = NUM_JOINTS;
    keys.clear();
    keys.resize(num);
    switch (type) {
    case ANGLES:
      alias = "Position";
      typeString = string("Position/Actuator/Value");
      break;
    case HARDNESS:
      alias = "Stiffness";
      typeString = string("Hardness/Actuator/Value");
      break;
    }

    keys[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/" + typeString);
    keys[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/" + typeString);
    keys[L_SHOULDER_ROLL] = string(
      "Device/SubDeviceList/LShoulderRoll/" + typeString);
    keys[L_SHOULDER_PITCH] = string(
      "Device/SubDeviceList/LShoulderPitch/" + typeString);
    keys[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/" + typeString);
    keys[L_ELBOW_ROLL] = string(
      "Device/SubDeviceList/LElbowRoll/" + typeString);
    keys[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/" + typeString);
    //!keys[lHandPosition] =
    //!string("Device/SubDeviceList/LHand/"  + typeString);
    keys[R_SHOULDER_ROLL] = string(
      "Device/SubDeviceList/RShoulderRoll/" + typeString);
    keys[R_SHOULDER_PITCH] = string(
      "Device/SubDeviceList/RShoulderPitch/" + typeString);
    keys[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/" + typeString);
    keys[R_ELBOW_ROLL] = string(
      "Device/SubDeviceList/RElbowRoll/" + typeString);
    keys[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/" + typeString);
    //!keys[rHandPosition] =
    //!string("Device/SubDeviceList/RHand/"  + typeString);
    keys[L_HIP_YAW_PITCH] = string(
      "Device/SubDeviceList/LHipYawPitch/" + typeString);
    keys[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/" + typeString);
    keys[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/" + typeString);
    keys[L_KNEE_PITCH] = string(
      "Device/SubDeviceList/LKneePitch/" + typeString);
    keys[L_ANKLE_PITCH] = string(
      "Device/SubDeviceList/LAnklePitch/" + typeString);
    keys[L_ANKLE_ROLL] = string(
      "Device/SubDeviceList/LAnkleRoll/" + typeString);
    keys[R_HIP_YAW_PITCH] = string(
      "Device/SubDeviceList/LHipYawPitch/" + typeString);
    //!LHipYawPitch = RHipYawPitch
    keys[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/" + typeString);
    keys[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/" + typeString);
    keys[R_KNEE_PITCH] = string(
      "Device/SubDeviceList/RKneePitch/" + typeString);
    keys[R_ANKLE_PITCH] = string(
      "Device/SubDeviceList/RAnklePitch/" + typeString);
    keys[R_ANKLE_ROLL] = string(
      "Device/SubDeviceList/RAnkleRoll/" + typeString);
    setActuatorAlias();
    setActuatorCommand();
  }

private:
  //! Type of joint actuator
  JointActuatorTypes type;
};

/**
 * @class LedActuators
 * @brief Defines the handles for LedActuators
 */
class LedActuators : public ActuatorLayer
{
public:
  /**
   * Constructor
   *
   * @param dcmProxy: pointer to NaoQi's dcm proxy.
   */
  LedActuators(
    const ALDCMProxyPtr& dcmProxy) :
    ActuatorLayer(dcmProxy)
  {
    init();
  }

  /**
   * Destructor
   */
  ~LedActuators()
  {
  }

protected:
  /**
   * Derived from ActuatorLayer
   */
  void
  init()
  {
    alias = "Led";
    num = NUM_LED_ACTUATORS;
    keys.clear();
    keys.resize(num);
    keys[FACE_LED_RED_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/0Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/45Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/90Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/135Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/180Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/225Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/270Deg/Actuator/Value");
    keys[FACE_LED_RED_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Left/315Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/0Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/45Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/90Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/135Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/180Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/225Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/270Deg/Actuator/Value");
    keys[FACE_LED_GREEN_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Left/315Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/45Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/90Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/135Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/180Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/225Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/270Deg/Actuator/Value");
    keys[FACE_LED_BLUE_LEFT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Left/315Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/45Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/90Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/135Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/180Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/225Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/270Deg/Actuator/Value");
    keys[FACE_LED_RED_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Red/Right/315Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/0Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/45Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/90Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/135Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/180Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/225Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/270Deg/Actuator/Value");
    keys[FACE_LED_GREEN_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Green/Right/315Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_45_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/45Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_90_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/90Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_135_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/135Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/180Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_225_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/225Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_270_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/270Deg/Actuator/Value");
    keys[FACE_LED_BLUE_RIGHT_315_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Face/Led/Blue/Right/315Deg/Actuator/Value");
    keys[EARS_LED_LEFT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/0Deg/Actuator/Value");
    keys[EARS_LED_LEFT_36_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/36Deg/Actuator/Value");
    keys[EARS_LED_LEFT_72_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/72Deg/Actuator/Value");
    keys[EARS_LED_LEFT_108_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/108Deg/Actuator/Value");
    keys[EARS_LED_LEFT_144_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/144Deg/Actuator/Value");
    keys[EARS_LED_LEFT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/180Deg/Actuator/Value");
    keys[EARS_LED_LEFT_216_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/216Deg/Actuator/Value");
    keys[EARS_LED_LEFT_252_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/252Deg/Actuator/Value");
    keys[EARS_LED_LEFT_288_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/288Deg/Actuator/Value");
    keys[EARS_LED_LEFT_324_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Left/324Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_0_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/0Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_36_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/36Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_72_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/72Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_108_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/108Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_144_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/144Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_180_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/180Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_216_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/216Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_252_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/252Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_288_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/288Deg/Actuator/Value");
    keys[EARS_LED_RIGHT_324_DEG_ACTUATOR] = string(
      "Device/SubDeviceList/Ears/Led/Right/324Deg/Actuator/Value");
    keys[CHEST_BOARD_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Red/Actuator/Value");
    keys[CHEST_BOARD_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Green/Actuator/Value");
    keys[CHEST_BOARD_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/ChestBoard/Led/Blue/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/0/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/1/Actuator/Value");
    keys[HEAD_LED_REAR_LEFT_2_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Left/2/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/0/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/1/Actuator/Value");
    keys[HEAD_LED_REAR_RIGHT_2_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Rear/Right/2/Actuator/Value");
    keys[HEAD_LED_MIDDLE_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Middle/Right/0/Actuator/Value");
    keys[HEAD_LED_FRONT_RIGHT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Right/0/Actuator/Value");
    keys[HEAD_LED_FRONT_RIGHT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Right/1/Actuator/Value");
    keys[HEAD_LED_FRONT_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Left/0/Actuator/Value");
    keys[HEAD_LED_FRONT_LEFT_1_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Front/Left/1/Actuator/Value");
    keys[HEAD_LED_MIDDLE_LEFT_0_ACTUATOR] = string(
      "Device/SubDeviceList/Head/Led/Middle/Left/0/Actuator/Value");
    keys[L_FOOT_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Red/Actuator/Value");
    keys[L_FOOT_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Green/Actuator/Value");
    keys[L_FOOT_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/LFoot/Led/Blue/Actuator/Value");
    keys[R_FOOT_LED_RED_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Red/Actuator/Value");
    keys[R_FOOT_LED_GREEN_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Green/Actuator/Value");
    keys[R_FOOT_LED_BLUE_ACTUATOR] = string(
      "Device/SubDeviceList/RFoot/Led/Blue/Actuator/Value");
    setActuatorAlias();
    setActuatorCommand();
  }
};
