/**
 * @file VisionModule/include/FeatureExtraction//RANSACEllipseFit.h
 *
 * This file defines the struct RANSACParams and declares the class 
 * RANSACEllipseFit
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/DirectEllipseFit.h"
#include "VisionModule/include/FeatureExtraction/Ellipse.h"
#include "Utils/include/MathsUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace Utils;

/**
 * @struct RANSACParams
 * @brief Holds RANSAC configuration parameters
 */
struct RANSACParams
{
  /**
   * Constructor
   */
  RANSACParams()
  {
  }

  /**
   * Constructor
   */
  RANSACParams(const unsigned& n, const unsigned& iter,
    const unsigned& minPoints, const float& distThresh) :
    n(n), iter(iter), minPoints(minPoints), distThresh(distThresh)
  {
  }

  //! Number of points to select randomly at each iteration
  unsigned n;

  //! Maximum number of iterations to be performed
  unsigned iter;

  //! Minimum number of points to process the ellipse fitting.
  unsigned minPoints;

  //! Distance threshold to be used for considering a point as an inlier
  float distThresh;
};

/**
 * @class RANSACEllipseFit
 * @brief Applies RANSAC based ellipse fitting to the input points 
 *   vectors and finds the best fitting ellipse.
 */
class RANSACEllipseFit
{
public:
  /**
   * Constructor
   * 
   * @param params: The RANSAC parameters to be used for fitting.
   */
  RANSACEllipseFit(const RANSACParams& params, const int& imageWidth,
    const int& imageHeight) :
    params(params), imageWidth(imageWidth), imageHeight(imageHeight)
  {
  }

  bool
  findBestEllipse(Ellipse& bestEllipse, vector<Point2f>& bestInliers);
  bool
  directFit(Ellipse& ellipse, const vector<Point2f>& points);

  void
  setPointVectors(const vector<vector<Point2f> >& pointVectors)
  {
    this->pointVectors = pointVectors;
  }

private:
  void
  chooseRandom(vector<Point2f>& points, vector<Point2f>& consensusSet);

  void
  distance(Ellipse& e, const vector<Point2f>& points,
    vector<Point2f>& inliersSet);

  RANSACParams params;
  vector<vector<Point2f> > pointVectors;
  int imageWidth;
  int imageHeight;
};
