/**
 * @file MotionModule/include/KickModule/Types/KickImpact2DSolver.h
 *
 * This file declares the class KickImpact2DSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/NLOptimizer.h"

template <typename Scalar>
class JSE2DImpKick;

/**
 * @class KickImpact2DSolver
 * @brief The class to solve for the robot's impact state based on given
 *   conditions of ball and foot at the start of the impact.
 */
template <typename Scalar>
class KickImpact2DSolver : public NLOptimizer
{
public:
  /**
   * Constructor.
   * 
   * @param jse2DImpKick: Associated kick for optimization
   */
  KickImpact2DSolver(JSE2DImpKick<Scalar>* jse2DImpKick);
  
  /**
   * Default destructor for this class.
   */
  ~KickImpact2DSolver()
  {
  }

  /**
   * Optimizes the knots for minimum time under given constraints.
   */
  void optDef();
  
private:
  /**
   * NLOPT based function for defining inequality constraints
   */
  void
  ineqConstraints(unsigned nCons, double *result, unsigned nVars,
    const double* vars, double* grad, void* data);
    
  /**
   * Evaluates the minimum time objective function
   */
  double
  costFunction(const vector<double> &vars, vector<double> &grad, void *data);
  
  //! Initial ball velocity
  Matrix<Scalar, 2, 1> iBallVel; // This is a vector of velocity magnitude and its angle.
  
  //! Final desired ball velocity
  Matrix<Scalar, 2, 1> fBallVelDes;

  //! Associated kick for optimization 
  JSE2DImpKick<Scalar>* kickPtr;
  
  //! Kinematics module
  boost::shared_ptr<KinematicsModule<Scalar> > kM;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
