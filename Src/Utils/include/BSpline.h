/**
 * @file Utils/include/BSpline.h
 *
 * This file declares the class BSpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include "Utils/include/Spline.h"

template <typename Scalar>
class BSplineNormalFinder;

/**
 * @class BSpline
 * @brief A class to define a b-spline and its helper functions
 */
template <typename Scalar>
class BSpline : public Spline<Scalar>
{
public:
  /**
   * Constructor with given knots
   *
   * @param degree: Degree of spline
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output
   * @param controlPoints: The control points of the spline
   * @param knots: The knot vector of spline
   * @param stepSize: Spline evaluation interval
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  BSpline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Matrix<Scalar, Dynamic, 1>& knots,
    const Scalar& stepSize);

  /**
   * Constructor with unspecified knots which sets equally spaced knots
   *
   * @param degree: Degree of spline
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output
   * @param controlPoints: The control points of the spline
   * @param splineTime: Total required time to for the spline
   * @param stepSize: Spline evaluation interval
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  BSpline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Scalar& splineTime, 
    const Scalar& stepSize);

  /**
   * Constructor that sets up the spline from xml file
   *
   * @param filePath: input xml file
   */
  BSpline(const string& filePath);

  /**
   * Destructor
   */ 
  ~BSpline() 
  {
  }
  
  /**
   * Derived from Spline
   */
  virtual void
  evaluateSpline(
    vector<vector<Scalar> >& spline, 
    vector<Scalar>& splineTime,
    const unsigned& derivative);

  /**
   * Plots the spline on given time interval.
   */
  void plotSpline();

  /**
   * Derived from Spline
   */
  virtual void splineFromXml(const string& filePath) {}

  /**
   * Sets up the resultant bspline properties.
   * Its dimension and derivatives to be computed.
   */
  void setup();

  /**
   * The function that generates the output bspline.
   *
   * @param Scalar step: The step of bspline over the interval.
   * @param vector<VectorXd>& bSpline: Resulting zeroth, first and second order derivatives of the bspline in space.
   * @return boolean
   */
  bool
  generateSplineAtStep(vector<Matrix<Scalar, Dynamic, 1>>& splineAtStep, const Scalar& step);
  
  /**
   * Finds normal on the spline to the given vector
   * 
   * @param normal: input normal vector
   * @param splinePoint: output spline point
   * 
   * @return if a point on spline is found
   */ 
  bool findNormalToVec(
    const Matrix<Scalar, 3, 1>& normal,
    Matrix<Scalar, 3, 1>& splinePoint,
    const Matrix<Scalar, 2, 1>& tBounds);
  
  /**
   * Returns the value of minimum knot in the knot sequence.
   *
   * @return Scalar
   */
  Scalar
  getMinKnot()
  {
    return minKnot;
  }

  /**
   * Returns the value of maximum knot in the knot sequence.
   *
   * @return Scalar
   */
  Scalar
  getMaxKnot()
  {
    return maxKnot;
  }
  
  /**
   * Sets the given knot vector
   * 
   * @param knots: the input knots sequence
   */ 
  void setKnots(const Matrix<Scalar, Dynamic, 1>& knots);
  
  /**
   * Gets the spline
   *
   * @param derivative: derivative order to return
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, Dynamic, Dynamic> getSpline(const unsigned& derivative)
  {
    return bSpline[derivative];
  }

protected:
  /**
   * Derived from Spline
   */
  void validateParameters();

private:
  /**
   * Locates the knot interval for the given step.
   *
   * @param step: Step of the Bspline.
   * @param knots: BSpline knot sequence.
   * @return bool
   */
  int
  locateKnot(Scalar step);

  /**
   * Generates a bspline table for basis functions.
   *
   * @param int& knotLocation: Location of the knot for current step.
   * @param Scalar& step: Step of the Bspline.
   * @param vector<Scalar>& knots: BSpline knot sequence.
   * @return bool
   */
  void
  bSplineTable(const int &knotLocation, const Scalar &step,
    vector<Scalar> &basisVector);

  /**
   * Generates the spline with all declared derivatives.
   *
   * @param int& knotLocation: Location of the knot for current step.
   * @param int dim: Dimension
   * @param vector<Scalar>& basisVector: BSpline basis functions.
   * 
   * @return the spline of given dimension and derivative
   */
  vector<Scalar>
  evaluateSplineAtBasis(
    const int& knotLocation, 
    const int& dim, 
    vector<Scalar>& basisVector);

  //! Output spline and its derivatives 0, 1, ...
  vector<Matrix<Scalar, Dynamic, Dynamic>> bSpline;

  //! Order of the derivative.
  vector<unsigned> derivativeOrder;

  //! Order of the spline.
  unsigned order;

  //! Order of the derivative.
  unsigned nDerivatives;

  //! Min knot value in knot sequence.
  Scalar minKnot;

  //! Max knot value in knot sequence.
  Scalar maxKnot;
  
  friend class BSplineNormalFinder<Scalar>;
};

