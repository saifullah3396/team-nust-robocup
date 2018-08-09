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
#include "TNRSBase/include/BaseIncludes.h"
#include "SBModule/include/SBManager.h"
#include "ControlModule/include/ActuatorRequests.h"

typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;

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
		(vector<float>, jointStiffnessSensors),
		(vector<float>, ledSensors),
	)

	/**
	 * Definition of output connector and variables for this module
	 */ 
	CREATE_OUTPUT_CONNECTOR(SBOutput,
		(StiffnessState, stiffnessState),
		(bool, whistleDetected),
    (BehaviorInfo, sBehaviorInfo),
	)

public:
  /**
   * Constructor
   *
   * @param parent: Pointer to parent module
   * @param motionProxy: Pointer to NaoQi's motion proxy
   */
  SBModule(void* parent, const ALMotionProxyPtr& motionProxy);

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
  //! Static behaviors manager shared object
  SBManagerPtr sbManager;

  //! Pointer to NaoQi internal motion class
  boost::shared_ptr<AL::ALMotionProxy> motionProxy;
};
