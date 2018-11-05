/**
 * @file FeatureExtraction/Ellipse.h
 *
 * This file defines the struct Ellipse.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

/**
 * @struct Ellipse
 * @brief Holds information about an ellipse
 */
struct Ellipse
{
  /**
   * Constructor
   */
  Ellipse(const int& imageWidth, const int& imageHeight) :
    imageWidth(imageWidth), imageHeight(imageHeight)
  {
    conic = vector<float>(6, 0.f);
    cx = cy = 0.f;
    rl = rs = 0.f;
    phi = 0.f;
  }

  bool
  isGood()
  {

    if (rs < 0.05 * rl || rl > max(imageWidth, imageHeight)) return false;
    return true;
  }

  /**
   * Finds geometric parameters from algebraic parameters
   */
  void
  alge2geom()
  {
    double tmp1 = conic[1] * conic[1] - 4 * conic[0] * conic[2];
    double tmp2 = sqrt(
      (conic[0] - conic[2]) * (conic[0] - conic[2]) + conic[1] * conic[1]);
    double tmp3 =
      conic[0] * conic[4] * conic[4] + conic[2] * conic[3] * conic[3] - conic[1] * conic[3] * conic[4] + tmp1 * conic[5];

    double r1 = -sqrt(2 * tmp3 * (conic[0] + conic[2] + tmp2)) / tmp1;
    double r2 = -sqrt(2 * tmp3 * (conic[0] + conic[2] - tmp2)) / tmp1;
    rl = r1 >= r2 ? r1 : r2;
    rs = r1 <= r2 ? r1 : r2;

    cx = (2 * conic[2] * conic[3] - conic[1] * conic[4]) / tmp1;
    cy = (2 * conic[0] * conic[4] - conic[1] * conic[3]) / tmp1;

    phi = 0.5 * atan2(conic[1], conic[0] - conic[2]);
    if (r1 > r2) phi += M_PI_2;
  }

  /**
   * Finds algebraic parameters from geometric parameters
   */
  void
  geom2alge()
  {
    conic[0] = rl * rl * sin(phi) * sin(phi) + rs * rs * cos(phi) * cos(phi);
    conic[1] = 2 * (rs * rs - rl * rl) * sin(phi) * cos(phi);
    conic[2] = rl * rl * cos(phi) * cos(phi) + rs * rs * sin(phi) * sin(phi);
    conic[3] = -2 * conic[0] * cx - conic[1] * cy;
    conic[4] = -conic[1] * cx - 2 * conic[2] * cy;
    conic[5] =
      conic[0] * cx * cx + conic[1] * cx * cy + conic[2] * cy * cy - rl * rl * rs * rs;
  }

  /**
   * Finds the contour of the ellipse with respect to the given image 
   *   parameters
   */
  void
  constructContour(const int& n = 200)
  {
    contour.clear();
    float dtheta = M_PI * 2 / n;
    float theta = 0;
    for (int i = 0; i < n; ++i, theta += dtheta) {
      Point2f p(
        cx + rl * cos(theta) * cos(0) + rs * sin(theta) * sin(0),
        cy + rl * cos(theta) * sin(0) + rs * sin(theta) * cos(0));
      if (p.x < imageWidth && p.y < imageHeight) contour.push_back(p);
    }
  }

  vector<Point2f>&
  getContour()
  {
    return contour;
  }

  //! Algebraic parameters as coefficients of ellipse conic section
  vector<float> conic; //! Coeffs conic[0], conic[1], conic[2], conic[3], conic[4], conic[5]

  //! Geometric parameters of the ellipse
  float cx; //! Ellipse center in x coordinate
  float cy; //! Ellipse center in y coordinate
  float rl; //! Ellipse major axis radius
  float rs; //! Ellipse minor axis radius
  float phi; //! Ellipse angle in radians

  vector<Point2f> contour; //! Ellipse contour
  int imageWidth; //! Width of the image
  int imageHeight; //! Height of the image

  Point2f cWorld;
  Point2f rlWorld;
  Point2f rsWorld;
};
