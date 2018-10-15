/**
 * @file MotionModule/include/KickModule/MaxMomentumEEOpt.h
 *
 * This file declares the class MaxMomentumEEOpt
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/NLOptimizer.h"

template <typename Scalar>
class KickModule;

/**
 * @class MaxMomentumEEOpt
 * @brief A class to solve for the best end-effector for a kick with 
 *   maximimum product of virtual mass and possible velocity hence 
 *   momentum.
 */
template <typename Scalar>
class MaxMomentumEEOpt : public NLOptimizer
{
public:
  /**
   * Constructor
   * 
   * @param kickModule: Associated kick module for solving the 
   *   optimization problem
   */
  MaxMomentumEEOpt(KickModule<Scalar>* kickModule);
  
  /**
   * Destructor
   */
  ~MaxMomentumEEOpt()
  {
  }

  /**
   * Optimizes the knots for minimum time under given constraints
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

  //! Associated kick module
  KickModule<Scalar>* kickModulePtr;
  
  //! Velocity constraints
  Matrix<Scalar, Dynamic, 1> velLimits;
  
  //! Kinematics  module
  boost::shared_ptr<KinematicsModule<Scalar> > kM;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
