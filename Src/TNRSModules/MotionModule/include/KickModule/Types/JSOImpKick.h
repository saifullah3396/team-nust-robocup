/**
 * @file MotionModule/include/KickModule/Types/JSOImpKick.h
 *
 * This file declares the class JSOImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "MotionModule/include/KickModule/MaxMomentumEEOpt.h"
#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"

class JSOImpKick : public JointSpaceKick
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  JSOImpKick(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    JointSpaceKick(motionModule, config, "JSOImpKick")
  {
    maxMomentumEEOpt = new MaxMomentumEEOpt(this);
  }

  /**
   * Default destructor for this class.
   */
  ~JSOImpKick()
  {
    delete maxMomentumEEOpt;
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
private:
  /**
   * Returns the cast of config to JSOImpKickConfigPtr
   */ 
  JSOImpKickConfigPtr getBehaviorCast();
  
  /**
   * Sets up kick parameters according to the behavior configuration
   */ 
  bool setupKickBase() throw (BehaviorException);
  
  /**
   * Finds best the end-effector point and orientation based on maximum
   * mass*velocity product and x-coordinate on the foot surface.   
   */
  virtual void findBestEEAndImpactPose();
  
  //! Best end-effector solver
  MaxMomentumEEOpt* maxMomentumEEOpt;
};

typedef boost::shared_ptr<JSOImpKick> JSOImpKickPtr;
