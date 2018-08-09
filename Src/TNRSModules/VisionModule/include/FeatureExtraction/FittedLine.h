/**
 * @file FeatureExtraction/FittedLine.h
 *
 * This file defines the struct FittedLine.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Utils/include/VisionUtils.h"

/**
 * @struct FittedLine
 * @brief Holds information about a line
 */
struct FittedLine
{
  /**
   * Constructor
   */
  FittedLine() :
    circleLine(false)
  {
  }

  /**
   * Constructor
   * 
   * @param f: Other fitted line
   */
  FittedLine(const FittedLine& fl) :
    p1(fl.p1), p2(fl.p2), unit(fl.unit), d(fl.d), circleLine(fl.circleLine)
  {
  }

  /**
   * Constructor
   * 
   * @param p1: One of the two end points of the line
   * @param p2: One of the two end points of the line
   */
  FittedLine(const Point2f& p1, const Point2f& p2) :
    p1(p1), p2(p2), circleLine(false)
  {
  }

  Point2f p1; //! One of the two end points of the line
  Point2f p2; //! One of the two end points of the line
  Point2f unit; //! Unit vector defining the line orientation
  Point2f perp; //! Unit vector defining the line orientation
  float perpDist;
  float angle; //! Line slope
  float d; //! Line length magnitude
  Point2f diff; //! Difference vector of points
  bool circleLine;
  boost::shared_ptr<vector<Point2f> > points;
};

typedef boost::shared_ptr<FittedLine> FittedLinePtr;
