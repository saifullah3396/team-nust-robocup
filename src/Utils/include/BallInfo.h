/**
 * @file Utils/include/BallInfo.h
 *
 * This file defines the struct BallInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>

using namespace cv;

/**
 * @struct BallInfo
 * @brief Holds information about the latest state of the ball
 */
struct BallInfo
{
  BallInfo()
  {
  }
  BallInfo(const unsigned& camera, const float& radius) :
    camera(camera), 
    cameraNext(camera), 
    found(false), 
    ballAge(-1.f),
    radius(radius)
  {
  }
  BallInfo(
    unsigned camera, bool found, Point posImage, Point2f posRel) :
    camera(camera), 
    cameraNext(camera), 
    found(found), 
    posImage(posImage), 
    posRel(posRel),
    bboxWidth(0),
    bboxHeight(0),
    ballAge(-1.f)
  {
  }

  unsigned camera;
  unsigned cameraNext;
  int bboxWidth;
  int bboxHeight;
  bool found;
  float ballAge;
  float radius;
  Point posImage;
  Point2f posRel;
  Point2f velRel;
  Point2f accRel;
};
