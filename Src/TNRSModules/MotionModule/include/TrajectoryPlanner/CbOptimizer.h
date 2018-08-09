/**
 * @file MotionModule/TrajectoryPlanner/CbOptimizer.h
 *
 * This file declares the class CbOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/TrajectoryPlanner/CubicSpline.h"
#include "MotionModule/include/TrajectoryPlanner/TrajOptimizer.h"

/**
 * @class CbOptimizer
 * @brief The class to generate optimize cubic spline based joint
 *   trajectories.
 */
class CbOptimizer : public TrajOptimizer
{
public:
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module 
   * @param cb The cubic spline object which is to be optimized.
   */
  CbOptimizer(
    MotionModule* motionModule,
    const unsigned& chainIndex,
    const unsigned& baseLeg,
    CubicSpline* cb) : 
  TrajOptimizer(motionModule, chainIndex, baseLeg), 
  cb(cb), 
  zmpCons(false),
  torqueCons(false), 
  consEvalSteps(20)
  {
    ASSERT(chainSize == this->cb->getSplineDim());
  }
  
  /**
   * Default destructor for this class.
   */
  ~CbOptimizer()
  {
  }
  ;

  /**
   * Sets the cb spline object.
   */ 
  void setCB(CubicSpline* cb) 
  { 
    this->cb = cb;
    ASSERT(chainSize == this->cb->getSplineDim());
  }

  void setZmpCons(const bool& zmpCons) { this->zmpCons = zmpCons; }
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
  RowVectorXf computeDynCons(
    const MatrixXf& times,
    const vector<MatrixXf> coeffs);

  CubicSpline* cb;
  
  bool zmpCons;
  bool torqueCons;
  
  //! Number of inner points to be used for evaluation
  unsigned consEvalSteps;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
