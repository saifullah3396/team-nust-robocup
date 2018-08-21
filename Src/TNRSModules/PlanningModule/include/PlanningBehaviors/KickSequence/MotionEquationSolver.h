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
  target(target), posI(posI), velI(velI)
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

protected:
  /**
   * Solves the equation of motion and returns position based on time
   */  
  virtual Vector2f solveMotionEq(const double& time) = 0;

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
  Vector2f solveMotionEq(const double& time) {
    return posI + velI / damping * (1 - exp(-damping * time));
  }
  
private:
  double damping;
};
