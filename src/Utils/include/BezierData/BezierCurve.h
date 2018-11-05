/**
 * @file Utils/inlclude/BezierCurve.h
 *
 * This file declares the class BezierCurve
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
#include "Utils/include/PlotEnv.h"
#include "Utils/include/Exceptions/TNRSException.h"

class BezierCurve;

/**
 * Enumeration for possible types of bezier curve exceptions
 *
 * @enum BezierCurveExceptionType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
  BezierCurveExceptionType,
  (EXC_INVALID_XML)
)

/**
 * @class BezierCurveException
 * @brief BezierCurve exception management class
 */
class BezierCurveException : public TNRSException
{
public:
  /**
   * Constructor
   *
   * @param curve: In which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type: Argument parser exception type
   */
  BezierCurveException(
    BezierCurve* curve,
    const string& message,
    const bool& bSysMsg,
    const BezierCurveExceptionType& type
  ) throw ();

  /**
   * Destructor
   */
  ~BezierCurveException() throw () {}

  string getExcPrefix()
    { return "Exception caught in bezier curve;\n\t"; }

private:
  BezierCurveExceptionType type;
};

/**
 * @class BezierCurve
 * @brief A class for defining bezier curves
 */
class BezierCurve
{
public:
  /**
   * Constructor with given knots
   *
   * @param degree: Degree of the curve
   * @param dim: Dimension of the curve (Cartesian X-Y-Z)
   * @param controlPoints: The control points of the curve
   * @param stepSize: Evaluation interval step
   */
  BezierCurve(
    const unsigned& degree,
    const unsigned& dim,
    const MatrixXf& controlPoints,
    const float& stepSize);
    
  /**
   * Constructor that sets up the curve from xml file
   *
   * @param filePath: input xml file
   */
  BezierCurve(const string& filePath);

  /**
   * Default destructor for this class.
   */
  virtual ~BezierCurve() 
  {
  }

protected:
  //! Dimension of the spline (3D, 2D).
  unsigned dim;

  //! Degree of the spline
  unsigned degree;

  //! BezierCurve step size
  float stepSize;

  //! Knot Sequence.
  VectorXf knots;

  //!Control Points.
  MatrixXf controlPoints;

  Matrix4f bezierMat;
  bezierMat << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
