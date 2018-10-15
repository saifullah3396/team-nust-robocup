/**
 * @file MotionModule/BalanceModule/Types/MPComControl.h
 *
 * This file declares the class MPComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"

template <typename Scalar>
class MPComControl : public BalanceModule<Scalar>
{
public:

    /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  MPComControl(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    BalanceModule<Scalar>(motionModule, config, "MPComControl"),
    execTime(0.f)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~MPComControl()
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
	 * Returns the cast of config to MPComControlConfigPtr
	 */
  MPComControlConfigPtr getBehaviorCast();
  void balanceMotionPrimitive();
  
  //! Total desired time to reach balance
  float timeToReachB;
  
  //! Motion execution time
  float execTime;
  
  //! Desired support Leg
  unsigned supportLeg;
  
  //! File stream to store actual center of mass state.
  fstream comLog;
};

typedef boost::shared_ptr<MPComControl<MType> > MPComControlPtr;
