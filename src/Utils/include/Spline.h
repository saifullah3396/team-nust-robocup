/**
 * @file Utils/inlclude/Spline.h
 *
 * This file declares the class Spline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once
#include <fstream>
#include "boost/lexical_cast.hpp"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <bits/stdc++.h>
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Exceptions/TNRSException.h"

enum SplineType {
  PIECE_WISE_POLY,
  BASIS
};

template <typename Scalar>
class Spline;

/**
 * Enumeration for possible types of spline exceptions
 *
 * @enum SplineExceptionType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
  SplineExceptionType,
  (EXC_INVALID_XML)
)

/**
 * @class SplineException
 * @brief Spline exception management class
 */
template <typename Scalar>
class SplineException : public TNRSException
{
public:
  /**
   * Constructor
   *
   * @param spline: In which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type: Argument parser exception type
   */
  SplineException(
    Spline<Scalar>* spline,
    const string& message,
    const bool& bSysMsg,
    const SplineExceptionType& type
  ) throw ();

  /**
   * Destructor
   */
  ~SplineException() throw () {}

  string getExcPrefix()
    { return "Exception caught in spline;\n\t"; }

private:
  SplineExceptionType type;
};

/**
 * @class Spline
 * @brief The base class for defining splines
 */
template <typename Scalar>
class Spline
{
public:
  /**
   * Constructor with given knots
   *
   * @param degeree: Degree of spline
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   *   in the resultant spline output
   * @param controlPoints: The control points of the spline
   * @param knots: The knot vector of spline
   * @param stepSize: Spline evaluation interval
   * @param type: Spline type
   */
  Spline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Matrix<Scalar, Dynamic, 1>& knots,
    const Scalar& stepSize, 
    const unsigned& type);

  /**
   * Constructor with unspecified knots which sets equally spaced knots
   *
   * @param degeree: Degree of spline
   * @param dim: Dimension of spline (Cartesian X-Y-Z)
   * @param controlPoints: The control points of the spline
   * @param splineTime: Total required time to for the spline
   * @param stepSize: Spline evaluation interval
   * @param type: Spline type
   */
  Spline(
    const unsigned& degree,
    const unsigned& dim,
    const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
    const Scalar& splineTime, 
    const Scalar& stepSize, 
    const unsigned& type);
    
  /**
   * Constructor that sets up the spline from xml file
   *
   * @param filePath: input xml file
   */
  Spline(const string& filePath);

  /**
   * Default destructor for this class.
   */
  virtual ~Spline() 
  {
  }

  /**
   * Sets up the class variables necessary for computing spline
   */ 
  virtual void setup() {}//= 0;

  /**
   * Evaludates the spline of required derivative order
   */
  virtual void
  evaluateSpline(
    vector<vector<Scalar> >& spline, 
    vector<Scalar>& splineTime,
    const unsigned& derivative) = 0;

  /**
   * Parses an xml file to get the spline properties
   * 
   * @param filePath: input xml file
   */
  virtual void splineFromXml(const string& filePath);

  /**
   * Plots the spline on given time interval.
   *
   * @param nInnerPoints: Number of inner spline points
   * @param startTime: Starting time of knots (Generally zero)
   */
  virtual void plotSpline() {}//= 0;
  
  /**
   * Getters
   */ 
  unsigned& getNKnots() { return nKnots; }
  unsigned& getDim() { return dim; }
  Scalar& getStepSize() { return stepSize; }
  Matrix<Scalar, Dynamic, 1>& getKnots() { return knots; }
  vector<Matrix<Scalar, Dynamic, Dynamic>>& getCoeffs() { return coeffs; }

protected:
  /**
   * Checks whether the spline input parameters are valid
   */ 
  virtual void validateParamters() {}
  
  //! Spline type
  int type;
  
  //! Dimension of the spline (3D, 2D).
  unsigned dim;

  //! Degree of the spline
  unsigned degree;

  //! Number of knots.
  unsigned nKnots;

  //! Spline step size
  Scalar stepSize;

  //! Knot Sequence.
  Matrix<Scalar, Dynamic, 1> knots;

  //!Control Points.
  Matrix<Scalar, Dynamic, Dynamic> controlPoints;

  //! Coefficients of the spline.
  vector<Matrix<Scalar, Dynamic, Dynamic>> coeffs;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
