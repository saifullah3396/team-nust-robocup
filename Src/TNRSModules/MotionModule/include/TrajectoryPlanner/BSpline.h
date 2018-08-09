/**
 * @file TrajectoryPlanner/BSpline.h
 *
 * This file declares a BSpline generator.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>

#include "Utils/include/MathsUtils.h"

using namespace std;
using namespace Eigen;

class BSpline
{
public:
  /**
   * Default constructor for this class.
   */
  BSpline();

  /**
   * Default virtual destructor for this class.
   */
  virtual
  ~BSpline();

  /**
   * Sets up the resultant bspline properties.
   * Its dimension and derivatives to be computed.
   */
  void
  setupSpline();

  /**
   * The function that generates the output bspline.
   *
   * @param double step: The step of bspline over the interval.
   * @param vector<VectorXd>& bSpline: Resulting zeroth, first and second order derivatives of the bspline in space.
   * @return boolean
   */
  bool
  generateSpline(double step);

  /**
   * Sets the order of the spline.
   *
   * @param unsigned splineOrder: Order of the bspline.
   */
  void
  setSplineOrder(unsigned splineOrder)
  {
    this->splineOrder = splineOrder;
    nControlPoints = knotVector.size() - splineOrder;
  }

  /**
   * Sets the vector for desired knots for the bspline generation.
   *
   * @param vector<double> knotVector: Sequence of knots for the bspline.
   */
  void
  setKnotVector(vector<double> knotVector);

  /**
   * Sets the bspline control points.
   *
   * @param VectorXd controlPoint: The control point in space depending upon the spline dimension.
   * @param unsigned index: Index of the control point.
   */
  void
  setControlPoint(VectorXd controlPoint, unsigned index);

  /**
   * Returns the value of minimum knot in the knot sequence.
   *
   * @return double
   */
  double
  getMinKnot()
  {
    return minKnot;
  }

  /**
   * Returns the value of maximum knot in the knot sequence.
   *
   * @return double
   */
  double
  getMaxKnot()
  {
    return maxKnot;
  }

  /**
   * Output spline.
   *
   * @var vector
   */
  vector<VectorXd>
  getSpline()
  {
    return bSpline;
  }
  ;

private:
  /**
   * Locates the knot interval for the given step.
   *
   * @param double &step: Step of the Bspline.
   * @param vector<double> &knotVector: BSpline knot sequence.
   * @return bool
   */
  int
  locateKnot(const double &step);

  /**
   * Generates a bspline table for basis functions.
   *
   * @param int& knotLocation: Location of the knot for current step.
   * @param double& step: Step of the Bspline.
   * @param vector<double>& knotVector: BSpline knot sequence.
   * @return bool
   */
  void
  bSplineTable(const int &knotLocation, const double &step,
    vector<double> &basisVector);

  /**
   * Generates the spline with all declared derivatives.
   *
   * @param int& knotLocation: Location of the knot for current step.
   * @param int dim: Dimension
   * @param vector<double>& basisVector: BSpline basis functions.
   */
  void
  evaluateSpline(const int &knotLocation, const int &dim,
    vector<double> &basisVector);

  /**
   * Output spline.
   *
   * @var vector
   */
  vector<VectorXd> bSpline;

  /**
   * Knot Sequence.
   *
   * @var vector
   */
  vector<double> knotVector;

  /**
   * Control Points.
   *
   * @var vector
   */
  vector<VectorXd> controlPoints;

  /**
   * Order of the derivative.
   *
   * @var vector
   */
  vector<unsigned> derivativeOrder;

  /**
   * Dimension of the spline (3D, 2D).
   *
   * @var unsigned
   */
  unsigned splineDim;

  /**
   * Order of the spline.
   *
   * @var unsigned
   */
  unsigned splineOrder;

  /**
   * Order of the derivative.
   *
   * @var unsigned
   */
  unsigned nDerivatives;

  /**
   * Number of control points.
   *
   * @var unsigned
   */
  unsigned nControlPoints;

  /**
   * Min knot value in knot sequence.
   *
   * @var double
   */
  double minKnot;

  /**
   * Max knot value in knot sequence.
   *
   * @var double
   */
  double maxKnot;
};
