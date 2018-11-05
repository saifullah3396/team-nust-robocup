/**
 * @file FeatureExtraction/ScannedCurve.h
 *
 * This file defines the struct ScannedCurve.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct ScannedCurve
 * @brief Holds information about a curve resulting from scanning the 
 *   image.
 */
struct ScannedCurve
{
  /**
   * Constructor
   */
  ScannedCurve()
  {
  }

  vector<Point> upper; //! Upper contour of the curve
  vector<Point> lower; //! Lower contour of the curve
  boost::shared_ptr<ScannedCurve> succ; //! Successor curve
  boost::shared_ptr<ScannedCurve> pred; //! Preceding curve
};

typedef boost::shared_ptr<ScannedCurve> ScannedCurvePtr;
