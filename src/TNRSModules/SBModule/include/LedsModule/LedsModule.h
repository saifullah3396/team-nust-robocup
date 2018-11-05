/**
 * @file SBModule/include/LedsModule/LedsModule.h
 *
 * This file declares the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "SBModule/include/StaticBehavior.h"
#include "SBModule/include/LedRequest.h"
#include "Utils/include/MathsUtils.h"

/**
 * @class LedsModule
 * @brief A class for generating led requests to control robot leds
 */ 
class LedsModule : public StaticBehavior
{
public:
  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  LedsModule(
	SBModule* sbModule,
	const BehaviorConfigPtr& config,
	const string& name = "LedsModule") : 
	StaticBehavior(sbModule, config, name)
  {
    ledRequest = boost::make_shared<LedRequest>();
  }
  
  /**
   * Destructor
   */
  ~LedsModule()
  {
  }

  /**
   * Returns its own child based on the given type
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<LedsModule> getType(
    SBModule* sbModule, const BehaviorConfigPtr& cfg);
  
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}
  
protected:
 /**
	* Returns the casts of config to SBLedsConfigPtr
  */ 
  SBLedsConfigPtr getBehaviorCast();
  
  //! Intensity to be reached by the leds
  vector<float> inToReach;

  //! Time to reach the desired intensity for interpolated behavior.
  float timeToReachIn;
  
  //! Led request
  LedRequestPtr ledRequest;
};

typedef boost::shared_ptr<LedsModule> LedsModulePtr;
