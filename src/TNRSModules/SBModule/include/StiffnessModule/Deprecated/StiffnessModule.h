/**
 * @file SBModule/StiffnessModule/StiffnessModule.h
 *
 * This file declares the class for sending the stiffness changes 
 * requests to the control module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#pragma once

#include "SBModule/StaticBehavior.h"
#include "SBModule/StiffnessModule/StiffnessDefinitions.h"

class StiffnessModule : public StaticBehavior
{
public:

  /**
   * Default constructor for this class.
   * 
   * @param sBModule: pointer to parent.
   */
  StiffnessModule(SBModule* sBModule) :
    StaticBehavior(sBModule)
  {
    loadInitialConfig();
  }

  /**
   * Default destructor for this class.
   */
  ~StiffnessModule()
  {
  }
  ;

  void
  initiate();
  void
  update();

private:
  void
  loadInitialConfig();

  //! Initial stiffnesses.
  vector<float> stiffnessesI;

  //! Difference with desired stiffness.
  vector<float> stiffnessesDelta;

  //! Time step for interpolation.
  float timeStep;

  //! Stiffnesses to be reached by the joints 	
  float sToReach;

  //! Time to reach stiffnesses
  float timeToReachS;

  //! Start of actuator indices.
  unsigned start;

  //! End of actuators indices.
  unsigned end;
};
