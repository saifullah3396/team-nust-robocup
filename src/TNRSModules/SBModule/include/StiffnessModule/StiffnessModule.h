/**
 * @file SBModule/include/StiffnessModule/StiffnessModule.h
 *
 * This file declares the class StiffnessModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#pragma once

#include "SBModule/include/StaticBehavior.h"
#include "SBModule/include/StiffnessModule/StiffnessDefinitions.h"
#include "Utils/include/StiffnessState.h"

/**
 * @class StiffnessModule
 * @brief The base class for creating behaviors to control robot 
 *   joint stiffnesses
 */
class StiffnessModule : public StaticBehavior
{
public:
  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  StiffnessModule(
		SBModule* sbModule,
		const BehaviorConfigPtr& config,
		const string& name = "StiffnessModule") : 
		StaticBehavior(sbModule, config, name)
  {
    targetState = StiffnessState::UNKNOWN;
  }

  /**
   * Destructor
   */
  virtual ~StiffnessModule()
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
  static boost::shared_ptr<StiffnessModule> getType(
    SBModule* sbModule, 
    const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}

protected:
	/**
	 * Returns the cast of config to SBStiffnessConfigPtr
	 */ 
	SBStiffnessConfigPtr getBehaviorCast();

  //! Stiffnesses to be reached by the joints 	
  vector<float> sToReach;

  //! Time to reach stiffnesses
  float timeToReachS;

  //! End stiffness state.
  StiffnessState targetState;
};

typedef boost::shared_ptr<StiffnessModule> StiffnessModulePtr;
