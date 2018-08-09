/**
 * @file MotionModule/include/KickModule/JSE2DImpKick.h
 *
 * This file declares the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"
#include "MotionModule/include/KickModule/Types/KickImpact2DSolver.h"
#include "Utils/include/NLOptimizer.h"

class JSE2DImpKick : public JointSpaceKick
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  JSE2DImpKick(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    JointSpaceKick(motionModule, config, "JSE2DImpKick")
  {
    kickImpact2DSolver = new KickImpact2DSolver(this);
  }

  /**
   * Default destructor for this class.
   */
  ~JSE2DImpKick()
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
   * Returns the cast of config to JSE2DImpKickConfigPtr
   */ 
  JSE2DImpKickConfigPtr getBehaviorCast();
  
  /**
   * Sets up kick parameters according to the behavior configuration
   */ 
  bool setupKickBase() throw (BehaviorException);
  
  /**
   * Solves for the best impact conditions to find impact pose and 
   * velocity of the end-effector
   */ 
  void solveForImpact();
  
  //! Ball initial velocity
  Vector3f ballVelocity;
  
  //! Best impact conditions solver
  KickImpact2DSolver* kickImpact2DSolver;
  friend class KickImpact2DSolver;
};

typedef boost::shared_ptr<JSE2DImpKick> JSE2DImpKickPtr;
