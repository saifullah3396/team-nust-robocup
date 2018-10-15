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

template <typename Scalar>
class JointSpaceKick : public KickModule<Scalar>
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
    KickModule<Scalar>(motionModule, config, name),
    kickSetup(false),
    minTimeToKick(1.0),
    kickTimeToImpact(0.0),
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
  virtual void setupKickBase() = 0;
  
  //! Finds the desired impact velocity based on the target distance and
  //! solving momentum conservation for foot-ball collision based on 
  //! end-effector virtual mass calculations.
  virtual void computeDesImpactVel(const Matrix<Scalar, Dynamic, 1>& impJoints);
  
  /**
   * Plots kick parameters
   */ 
  void plotKick();

  /**
   * Plots kick end-effector trajectory
   */ 
  void plotEETrajectory();
  
  /**
   * Saves the end-effector pose in realtime to logger
   */ 
  void logEndEffectorActual();
  
  /** 
   * Plots the joint trajectories
   */
  //virtual void plotJointTrajectories();

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
  Scalar minTimeToKick;
  
  //! Time taken by kick trajectory until it reaches the hitting pose
  Scalar kickTimeToImpact;
  
  //! Desired end-effector velocity on impact in cartesian space
  Matrix<Scalar, 3, 1> desImpactVel;
  
  //! Desired ball velocity in 1D in target direction
  Scalar desBallVel;
  
  //! Vector of required cartesian poses for interpolation
  vector<Matrix<Scalar, 4, 4>> cPoses;

  //! Discretized kick trajectories with step size equal to cycleTime.
  vector<vector<Scalar> > jointTrajectories;
  
  //! Target Position in base leg frame
  Matrix<Scalar, 3, 1> targetPosition;
  
  //! Distance from ball to target
  Scalar targetDistance;
  
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

typedef boost::shared_ptr<JointSpaceKick<MType> > JointSpaceKickPtr;
