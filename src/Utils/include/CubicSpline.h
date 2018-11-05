/**
 * @file Utils/inlclude/CubicSpline.h
 *
 * This file declares the class CubicSpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once
#include "Utils/include/PolySpline.h"

/**
 * @class CubicSpline
 * @brief A class to define a cubic spline and its helper functions
 */
template <typename Scalar>
class CubicSpline : public PolySpline<Scalar>
{
public:
  /**
   * Constructor with given knots
   *
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output
   * @param controlPoints: The control points of the spline
   * @param knots: The knot vector of spline
   * @param stepSize: Spline evaluation interval
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  CubicSpline(
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Matrix<Scalar, Dynamic, 1>& knots,
    const Scalar& stepSize, 
    const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds);

  /**
   * Constructor with unspecified knots which sets equally spaced knots
   *
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   * @param maxDerivative: Maximum derivatives to be acquired
   *   in the resultant spline output
   * @param controlPoints: The control points of the spline
   * @param splineTime: Total required time to for the spline
   * @param stepSize: Spline evaluation interval
   * @param boundary: Spline boundary
   *   conditions (velocity at the end points)
   */
  CubicSpline(
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Scalar& splineTime, 
    const Scalar& stepSize, 
    const Matrix<Scalar, Dynamic, Dynamic>& boundaryVels);

  /**
   * Constructor that sets up the spline from xml file
   *
   * @param filePath: input xml file
   */
  CubicSpline(const string& filePath);

  /**
   * Default destructor for this class.
   */
  ~CubicSpline()
  {
  }

  /**
   * Derived from Spline
   */ 
  void setup();

private:
  /**
   * Derived from PolySpline
   */
  virtual void genParams();

  /**
   * Derived from PolySpline
   */
  virtual void genCoeffs();
  
  /**
   * Derived from PolySpline
   */
  virtual void genInvAMat();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
