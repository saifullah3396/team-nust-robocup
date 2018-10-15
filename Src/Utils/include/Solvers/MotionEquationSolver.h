/**
 * @file PlanningBehaviors/KickSequence/MotionEquationSolver.h
 *
 * This file declares the class MotionEquationSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/NLOptimizer.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/EnvConsts.h"

/**
 * @class MotionEquationSolver
 * @brief A class to solve the equation of motion for ball to estimate
 *   the time it will take to get closest to the target.
 */
class MotionEquationSolver : public NLOptimizer
{
public:
  /**
   * Constructor
   */
  MotionEquationSolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI) : 
  target(target), posI(posI), velI(velI),
  distFromTarget(0.f), timeToReach(0.f)
  {
  }
  
  /**
   * Destructor
   */
  ~MotionEquationSolver()
  {
  }

  /**
   * Optimizes the knots for minimum time under given constraints
   */
  virtual void optDef();

  Vector2f getEndPosition() { return endPosition; }
  Vector2f getEndVelocity() { return endVelocity; }
  float getTimeToReach() { return timeToReach; }
  float getDistFromTarget() { return distFromTarget; }

protected:
  /**
   * Solves the equation of motion and returns position based on time
   */  
  virtual Vector2f solvePosition(const double& time) = 0;
  
  /**
   * Solves the equation of motion and returns velocity based on time
   */  
  virtual Vector2f solveVelocity(const double& time) = 0;

  /**
   * Evaluates the minimum time objective function
   */
  double
  costFunction(const vector<double> &vars, vector<double> &grad, void *data);
  
  //! Target to find solution for
  Vector2f target;
  
  //! Ball initial position
  Vector2f posI;
  
  //! Ball initial velocity
  Vector2f velI;

  //! Solved end position
  Vector2f endPosition;
  
  //! Solved end velocity
  Vector2f endVelocity;

  //! Distance of the end position from the given target 
  float distFromTarget;
  
  //! Time to reach the end position
  float timeToReach;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class DampedMESolver
 * @brief A class to solve the equation of motion of a damped system
 *   without stiffness
 */
class DampedMESolver : public MotionEquationSolver {
public:
  /**
   * Constructor
   */ 
  DampedMESolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI,
    const double& damping) : 
    MotionEquationSolver(target, posI, velI), damping(damping) 
  {}
  
  /**
   * Destructor
   */ 
  ~DampedMESolver() {}
  
  /**
   * Derived
   */ 
  Vector2f solvePosition(const double& time) {
    return posI + velI / damping * (1 - exp(-damping * time));
  }
  
  /**
   * Derived
   */ 
  Vector2f solveVelocity(const double& time) {
    return velI * exp(-damping * time);
  }
  
private:
  double damping;
};

/**
 * @class FrictionMESolver
 * @brief A class to solve the equation of motion of friction based
 *   motion equation
 */
class FrictionMESolver : public MotionEquationSolver {
public:
  /**
   * Constructor
   */ 
  FrictionMESolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI,
    const double& friction) : 
    MotionEquationSolver(target, posI, velI)
  {
    // velocity direction
    auto unitVel = velI / velI.norm();
    accel[0] = -friction * gConst * unitVel[0];
    accel[1] = -friction * gConst * unitVel[1];
  }
  
  /**
   * Destructor
   */ 
  ~FrictionMESolver() {}
  
  /**
   * Derived
   */ 
  Vector2f solvePosition(const double& time) {
    return posI + velI * time + 0.5 * accel * time * time;
  }
  
  /**
   * Derived
   */ 
  Vector2f solveVelocity(const double& time) {
    return velI + accel * time;
  }
  
private:
  Vector2f accel;
};

