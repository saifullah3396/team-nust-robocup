/**
 * @file MotionModule/TrajectoryPlanner/CbOptimizer.h
 *
 * This file declares the class CbOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/CubicSpline.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/TrajectoryPlanner/TrajOptimizer.h"

/**
 * @class CbOptimizer
 * @brief The class to generate optimize cubic spline based joint
 *   trajectories.
 */
template <typename Scalar>
class CbOptimizer : public TrajOptimizer<Scalar>
{
public:
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module 
   * @param chainIndex leg chain index according to kinematics module
   * @param chainIndex base leg chain index according to kinematics module 
   *   if balancing is needed
   * @param cb The cubic spline object which is to be optimized.
   */
  CbOptimizer(
    MotionModule* motionModule,
    const unsigned& chainIndex,
    const unsigned& baseLeg,
    CubicSpline<Scalar>* cb) : 
  TrajOptimizer<Scalar>(motionModule, chainIndex, baseLeg), 
  cb(cb), 
  zmpCons(false),
  torqueCons(false), 
  eeCons(false),
  consEvalSteps(20)
  {
    ASSERT(chainSize == this->cb->getSplineDim());
  }
  
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module 
   * @param chainIndex leg chain index according to kinematics module
   * @param chainIndex base leg chain index according to kinematics module 
   *   if balancing is needed
   * @param endEffector endeffector transformation matrix from chain end
   * @param eeVelMax maximum velocity constraints on end-effector
   * @param cb The cubic spline object which is to be optimized.
   */
  CbOptimizer(
    MotionModule* motionModule,
    const unsigned& chainIndex,
    const unsigned& baseLeg,
    const Matrix<Scalar, Dynamic, Dynamic>& endEffector,
    const Matrix<Scalar, Dynamic, 1>& eeVelMax,
    CubicSpline<Scalar>* cb) : 
  TrajOptimizer<Scalar>(motionModule, chainIndex, baseLeg), 
  cb(cb), 
  zmpCons(false),
  torqueCons(false), 
  consEvalSteps(20),
  endEffector(endEffector),
  eeVelMax(eeVelMax),
  eeCons(true)
  {
    ASSERT(chainSize == this->cb->getSplineDim());
  }
  
  /**
   * Default destructor for this class.
   */
  ~CbOptimizer()
  {
  }

  /**
   * Sets the cb spline object.
   */ 
  void setCB(CubicSpline<Scalar>* cb) 
  { 
    this->cb = cb;
    ASSERT(chainSize == this->cb->getSplineDim());
  }

  void setZmpCons(const bool& zmpCons) 
    { this->zmpCons = zmpCons; }
    
  void setTorqueCons(const bool& torqueCons) 
    { this->torqueCons = torqueCons; }

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
    const double* knots, double* grad, void* data);
    
  /**
   * Evaluates the minimum time objective function
   */
  double
  costFunction(const vector<double> &knots, vector<double> &grad, void *data);

  /**
   * Computes dynamic constraints such as zmp or torque using inverse dynamics.
   */  
  Matrix<Scalar, 1, Dynamic> computeDynCons(
    const Matrix<Scalar, Dynamic, Dynamic>& times,
    const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs);

  /**
   * Computes end-effector velocity constraints
   */  
  Matrix<Scalar, 1, Dynamic> computeEECons(
    const Matrix<Scalar, Dynamic, Dynamic>& times, 
    const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs);

  CubicSpline<Scalar>* cb;
  
  bool zmpCons;
  bool torqueCons;
  bool eeCons;
  
  //! Matrix defining the end-effector transformation frame from chain end
  Matrix<Scalar, 4, 4> endEffector;
  
  //! Maximum ee cartesian velocities constraints
  Matrix<Scalar, Dynamic, 1> eeVelMax;
  
  //! Number of inner points to be used for evaluation
  unsigned consEvalSteps;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

