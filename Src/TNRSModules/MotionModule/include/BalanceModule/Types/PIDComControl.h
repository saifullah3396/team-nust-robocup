/**
 * @file MotionModule/BalanceModule/Types/PIDComControl.h
 *
 * This file declares the class PIDComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/BalanceModule/BalanceModule.h"

class PIDComControl : public BalanceModule
{
public:

  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  PIDComControl(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    BalanceModule(motionModule, config, "PIDComControl")
  {
  }

  /**
   * Default destructor for this class.
   */
  ~PIDComControl()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  
private:
  /**
	 * Returns the cast of config to PIDComControlConfigPtr
	 */
  PIDComControlConfigPtr getBehaviorCast();

  //! Desired support Leg
  unsigned supportLeg;
  
  //! File stream to store actual center of mass state.
  fstream comLog;
};

typedef boost::shared_ptr<PIDComControl> PIDComControlPtr;
