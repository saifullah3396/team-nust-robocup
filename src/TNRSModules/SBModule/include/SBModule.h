/**
 * @file SBModule/include/SBModule.h
 *
 * This file declares the class SBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "ControlModule/include/HardwareLayer.h"
#include "ControlModule/include/ActuatorRequests.h"
#include "SBModule/include/SBManager.h"
#include "TNRSBase/include/BaseIncludes.h"

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

/**
 * @class SBModule
 * @brief The base module that handles all kinds of static behaviors
 */
class SBModule : public BaseModule
{
	/**
	 * Definition of input connector and variables for this module
	 */ 
	CREATE_INPUT_CONNECTOR(SBInput,
		(int, sbThreadPeriod),
	)

	/**
	 * Definition of output connector and variables for this module
	 */ 
	CREATE_OUTPUT_CONNECTOR(SBOutput,
    (vector<float>, jointStiffnessSensors),
    (vector<float>, ledSensors),
		(StiffnessState, stiffnessState),
		(bool, whistleDetected),
    (BehaviorInfo, sBehaviorInfo),
	)

public:
  /**
   * Constructor
   *
   * @param parent: parent: Pointer to parent module
   * @param memoryProxy: Pointer to NaoQi's memory proxy
   * @param dcmProxy: Pointer to NaoQi's DCM proxy
   * @param motionProxy: Pointer to NaoQi's motion proxy
   */
  SBModule(
   void* parent,
   const ALMemoryProxyPtr& memoryProxy,
   const ALDCMProxyPtr& dcmProxy,
   const ALMotionProxyPtr& motionProxy);

  /**
   * Destructor
   */
  ~SBModule()
  {    
		delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();

  /**
   * Gets the NaoQi's motion proxy 
   *
   * @return Pointer to MotionProxy
   */
  boost::shared_ptr<AL::ALMotionProxy> getSharedMotionProxy()
  {
    return motionProxy;
  }

private:
  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   */
  void sensorsUpdate();

  /**
   * Sends the requested actuator commands to NaoQi DCM for execution
   */
  void actuatorsUpdate();

  //! Static behaviors manager shared object
  SBManagerPtr sbManager;

  //! Pointer to NaoQi internal motion class
  boost::shared_ptr<AL::ALMotionProxy> motionProxy;

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Vector of pointer to ActuatorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers;

  //! Pointer to NaoQi internal memory proxy
  ALMemoryProxyPtr memoryProxy;

  //! Pointer to NaoQi internal dcm proxy
  ALDCMProxyPtr dcmProxy;

  enum class SBSensors : unsigned {
    JOINT_STIFFNESS,
    LED,
    COUNT
  };

  enum class SBActuators : unsigned {
    JOINT_STIFFNESS,
    LED,
    COUNT
  };
};