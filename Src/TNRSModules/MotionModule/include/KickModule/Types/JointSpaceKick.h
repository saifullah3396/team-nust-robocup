/**
 * @file MotionModule/include/KickModule/JointSpaceKick.h
 *
 * This file declares the class JointSpaceKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"
#include "MotionModule/include/TrajectoryPlanner/CbOptimizer.h"
#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/KickModule/KickModule.h"

class JointSpaceKick : public KickModule
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  JointSpaceKick(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "JointSpaceKick") :
    KickModule(motionModule, config, name),
    kickSetup(false),
    minTimeToKick(1.f),
    behaviorState(posture),
    desImpactVelKnown(false)
  {
  }

  /**
   * Destructor
   */
  ~JointSpaceKick()
  {
  }

protected:
  /**
   * Sets up the kick parameters according to the behavior configuration
   */ 
  virtual bool setupKickBase() throw (BehaviorException) = 0;
  
  //! Finds the desired impact velocity based on the target distance and
  //! solving momentum conservation for foot-ball collision based on 
  //! end-effector virtual mass calculations.
  virtual void computeDesImpactVel(const VectorXf& impJoints);
  
  /** 
   * Plots the feet, ball and kick cartesian poses in x-y plane
   */
  virtual void plotKick();
  
  /** 
   * Plots the joint trajectories
   */
  virtual void plotJointTrajectories();

  /**
   * Defines the kick trajectory.
   */
  virtual void
  defineTrajectory();

  /**
   * Requests kick execution based on naoqi joint interpolation.
   */
  virtual void
  requestExecution();
  
  /**
   * Sets up the balance module with the input config as a child
   */
  virtual void setupBalance();
    
  /**
   * Sets the desired ball velocity based on the required information.
   * For example, one way to solve for it is using known frictional 
   * parameters. Another could be using a pretrained learning classifier.
   */ 
	virtual void setDesBallVel();

  //! Whether the kick has been setup and ready for execution
  bool kickSetup;
  
  //! Minumum limit for the total optimized trajectory time 
  float minTimeToKick;
  
  //! Desired end-effector velocity on impact in cartesian space
  Vector3f desImpactVel;
  
  //! Desired ball velocity in 1D in target direction
  float desBallVel;
  
  //! Vector of required cartesian poses for interpolation
  vector<Matrix4f> cPoses;

  //! Discretized kick trajectories with step size equal to cycleTime.
  vector<vector<float> > jointTrajectories;
  
  //! Target Position in base leg frame
  Vector3f targetPosition;
  
  //! Distance from ball to target
  float targetDistance;
  
  //! Whether to compute desImpactVel based on distance // Impulse/Vm based solution
  bool desImpactVelKnown;
  
  //! Current state of this behavior
  unsigned behaviorState;
  
  /**
   * States of this behavior
   * 
   * @enum BehaviorState
   */ 
  enum BehaviorState
  {
    posture,
    balance,
    kick,
    postKickPosture
  };

private:
  JSKickConfigPtr getBehaviorCast();
};

typedef boost::shared_ptr<JointSpaceKick> JointSpaceKickPtr;
