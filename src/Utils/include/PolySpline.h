/**
 * @file Utils/inlclude/PolySpline.h
 *
 * This file declares the class PolySpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once
#include "Utils/include/Spline.h"

/**
 * @class PolySpline
 * @brief A class to define a piece-wise polynomial spline
 */
template <typename Scalar>
class PolySpline : public Spline<Scalar>
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
  PolySpline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Matrix<Scalar, Dynamic, 1>& knots,
    const Scalar& stepSize, 
    const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds);

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
  PolySpline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Scalar& splineTime, 
    const Scalar& stepSize, 
    const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds);

  /**
   * Constructor that sets up the spline from xml file
   *
   * @param filePath: input xml file
   */
  PolySpline(const string& filePath);

  /**
   * Default destructor for this class.
   */
  ~PolySpline()
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
   * Evaluates the spline coefficients at specified knot vector
   *
   * @param knots: The knots vector.
   */
  virtual void evaluateCoeffs(const Matrix<Scalar, Dynamic, 1>& knots);

  /**
   * Plots the spline on given time interval.
   *
   * @param nInnerPoints: Number of inner spline points
   * @param startTime: Starting time of knots (Generally zero)
   */
  void plotSpline(const unsigned& nInnerPoints, const Scalar& startTime);

  /**
   * Derived from Spline
   */
  virtual void splineFromXml(const string& filePath);

  /**
   * Getters used by friend classes
   */ 
  Matrix<Scalar, Dynamic, Dynamic>& getRepKnots() { return knotsRep; }
  Matrix<Scalar, Dynamic, Dynamic>& getBAccels() { return bAccels; }
  Matrix<Scalar, Dynamic, Dynamic>& getBAccelsU() { return bAccelsU; }
  Matrix<Scalar, Dynamic, Dynamic>& getBAccelsL() { return bAccelsL; }
  Matrix<Scalar, Dynamic, Dynamic>& getCpDiff() { return controlPointsDiff; }

protected:
  /**
   * Generates a spline matrix for solving
   * the system of equations along with the right-hand side b matrix
   * for each variable to find x = invA * b.
   */
  virtual void genParams() = 0;

  /**
   * Generates the spline coefficient matrices by solving x = invA * b
   * and then using the specific relations.
   */
  virtual void genCoeffs() = 0;

  /**
   * Generates a spline matrix for solving
   * the system of equations along with the right-hand side b matrix
   * for each variable to find x = invA * b.
   */
  virtual void genInvAMat() = 0;
  
  /**
   * Derived from Spline
   */
  void validateParameters();
  
  //! Type of boundary given conditions 
  //! natural or clamped
  string boundaryType;
  
  //! Knot sequence for multiple variables.
  Matrix<Scalar, Dynamic, Dynamic> knotsRep;

  //! Velocities at the boundaries for multiple variables
  Matrix<Scalar, Dynamic, Dynamic> boundaryConds;
  
  //! Upper control points.
  Matrix<Scalar, Dynamic, Dynamic> controlPointsU;

  //! Lower control points.
  Matrix<Scalar, Dynamic, Dynamic> controlPointsL;

  //! Difference of upper and lower control points.
  Matrix<Scalar, Dynamic, Dynamic> controlPointsDiff;

  //! A matrix for Ax = b.
  Matrix<Scalar, Dynamic, Dynamic> A;

  //! Inverse of the A matrix.
  Matrix<Scalar, Dynamic, Dynamic> invA;

  //! b matrix for Ax = b.
  Matrix<Scalar, Dynamic, Dynamic> b;

  //! Inner boundary accelerations x (bAccels) = invA b.
  Matrix<Scalar, Dynamic, Dynamic> bAccels;

  //! Upper inner boundary accelerations.
  Matrix<Scalar, Dynamic, Dynamic> bAccelsU;

  //! Lower inner boundary accelerations.
  Matrix<Scalar, Dynamic, Dynamic> bAccelsL;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
