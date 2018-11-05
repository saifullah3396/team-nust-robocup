/**
 * @file MotionModule/TrajectoryPlanner/CubicSpline.h
 *
 * This file declares the class to generate cubic splines based on
 * required conditions.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once
#include <numeric>
#include <nlopt/nlopt.hpp>
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/GnuPlotter.h"

/**
 * @class CubicSpline
 * @brief The class to generate cubic splines based on
 *   required conditions.
 */
class CubicSpline
{
public:
  /**
   * Constructor for this class with given knots.
   *
   * @param splineDim: Dimension of spline (Cartesian X-Y-Z).
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output.
   * @param controlPoints: The control points of
   *   the spline.
   * @param knots: The knot vector of spline.
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  CubicSpline(const float& stepSize, const unsigned& splineDim,
    const unsigned& maxDerivative, const MatrixXf& controlPoints,
    const VectorXf& knots, const MatrixXf& boundaryVels);

  /**
   * Constructor for this class with unspecified knots to get
   * a default spline with equal knots.
   *
   * @param splineDim: Dimension of spline (Cartesian X-Y-Z).
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output.
   * @param controlPoints: The control points of
   *   the spline.
   * @param trajectoryTime: Total required time to for the spline.
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  CubicSpline(const float& stepSize, const unsigned& splineDim,
    const unsigned& maxDerivative, const MatrixXf& controlPoints,
    const float& trajectoryTime, const MatrixXf& boundaryVels);

  /**
   * Default destructor for this class.
   */
  ~CubicSpline()
  {
  }
  ;

  /**
   * The function that generates and plots the spline on given time
   * interval.
   *
   * @param nInnerPoints: Number of inner spline points.
   * @param startTime: Starting time of knots (Generally zero).
   */
  void
  plotSpline(const unsigned& nInnerPoints, const float& startTime);

  /**
   * Optimizes the knots for minimum time under given constraints.
   */
  void
  optimizeKnots(const RowVectorXf& maxVels);

  /**
   * Returns the coefficients generated.
   */
  vector<MatrixXf>
  getCoeffs()
  {
    return coeffs;
  }

  /**
   * Returns the trajectories of required derivative.
   */
  void
  getTrajectories(vector<vector<float> >& traj, vector<float>& trajTime,
    const unsigned& derivative);
private:
  /**
   * Locates the knot interval for the given step.
   *
   * @param step: Step of the Bspline.
   *
   * @return int
   */
  int
  locateKnot(const float& step);

  /**
   * Generates a cubic spline matrix for solving
   * the system of equations along with the right-hand side b matrix
   * for each variable to find x = invA * b.
   */
  void
  genParams();

  /**
   * Generates a cubic spline matrix for solving
   * the system of equations along with the right-hand side b matrix
   * for each variable to find x = invA * b.
   */
  void
  genInvAMat();

  /**
   * Evaluates the cubic spline at specified knot vector.
   *
   * @param knots: The knots vector.
   */
  void
  evaluate(const VectorXf& knots);

  /**
   * Generates the spline coefficient matrices by solving x = invA * b
   * and then using the specific relations.
   */
  void
  genCoeffs();
  void
  ineqConstraints(unsigned nCons, double *result, unsigned nVars,
    const double* knots, double* grad, void* data);
  double
  minTimeObj(const vector<double> &knots, vector<double> &grad, void *data);
  static double
  objWrapper(const vector<double> &knots, vector<double> &grad, void *data);
  static void
  ineqWrapper(unsigned nCons, double *result, unsigned nVars,
    const double* knots, double* grad, void* data);

  //! Order of the derivative.
  vector<unsigned> derivativeOrder;

  //! Dimension of the spline (3D, 2D).
  unsigned splineDim;

  //! Order of the derivative.
  unsigned nDerivatives;

  //! Number of knots.
  unsigned nKnots;

  //! Trajectory step size
  float stepSize;

  //! Output spline.
  MatrixXf cubicSpline;

  //! Knot Sequence.
  VectorXf knots;

  //! Knot sequence for multiple variables.
  MatrixXf knotsRep;

  //!Control Points.
  MatrixXf controlPoints;

  //! Upper control points.
  MatrixXf controlPointsU;

  //! Lower control points.
  MatrixXf controlPointsL;

  //! Difference of upper and lower control points.
  MatrixXf controlPointsDiff;

  //! Velocities at the boundaries for multiple variables
  MatrixXf boundaryVels;

  //! A matrix for Ax = b.
  MatrixXf A;

  //! Inverse of the A matrix.
  MatrixXf invA;

  //! b matrix for Ax = b.
  MatrixXf b;

  //! Inner boundary accelerations x (bAccels) = invA b.
  MatrixXf bAccels;

  //! Upper inner boundary accelerations.
  MatrixXf bAccelsU;

  //! Lower inner boundary accelerations.
  MatrixXf bAccelsL;

  //! Coefficients of the spline.
  vector<MatrixXf> coeffs;

  //! Maximum reachable velocity for spline.
  RowVectorXf maxVels;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
