/**
 * @file Utils/include/Landmark.h
 *
 * This file defines the structs Landmark and ObsLandmarks.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017 
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/RobotStateDefinitions.h"

using namespace cv;

/**
 * @struct Landmark
 * @brief The struct for defining a field landmark.
 */
struct Landmark
{
  Landmark() :
    id(-1), type(0), pos(Point2f(-100, -100))
  {
  }
  Landmark(const unsigned& type) :
    id(-1), type(type), pos(Point2f(-100, -100))
  {
  }
  Landmark(const unsigned& type, const Point2f& pos) :
    id(-1), type(type), pos(pos)
  {
  }
  int id;
  unsigned type;
  Point2f pos;
};

typedef boost::shared_ptr<Landmark> LandmarkPtr;

/**
 * @struct UnknownLandmark
 * @brief The struct for defining the landmarks for which a match in 
 *   the actual field is not known.
 */
struct UnknownLandmark : Landmark
{
  UnknownLandmark()
  {
  }
  UnknownLandmark(const unsigned& type) :
    Landmark(type)
  {
  }
  UnknownLandmark(const unsigned& type, const Point2f& pos) :
    Landmark(type, pos)
  {
  }
};

typedef boost::shared_ptr<UnknownLandmark> UnknownLandmarkPtr;

/**
 * @struct UnknownLandmark
 * @brief The struct for defining the landmarks for which a match in 
 *   the actual field is known such as T, L corners, or center circle.
 */
struct KnownLandmark : Landmark
{
  KnownLandmark() :
    poseFromLandmark(RobotPose2D<float>(0.f, 0.f, 0.f))
  {
  }
  KnownLandmark(const unsigned& type) :
    Landmark(type)
  {
  }
  KnownLandmark(const unsigned& type, const Point2f& pos,
    const RobotPose2D<float>& poseFromLandmark) :
    Landmark(type, pos), poseFromLandmark(poseFromLandmark)
  {
  }
  RobotPose2D<float> poseFromLandmark;
};

typedef boost::shared_ptr<KnownLandmark> KnownLandmarkPtr;
