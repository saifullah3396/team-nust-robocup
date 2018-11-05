/**
 * @file FeatureExtraction/GoalPost.h
 *
 * This file defines the struct GoalPost.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct GoalPost
 * @brief Holds information about a detected goalpost
 */
struct GoalPost
{
  /** 
   * Constructor
   * 
   * @param p: Center point of the detected goal post in world
   */
  GoalPost(const Point2f& world, const Point2f& image,
    const float& timeDetected) :
    world(world), image(image), timeDetected(timeDetected), refresh(true)
  {
  }

  GoalPost(const Point2f& world, const Point2f& image) :
    world(world), image(image), refresh(true)
  {
  }

  bool
  checkDuplicate(const boost::shared_ptr<GoalPost>& other)
  {
    if (norm(other->world - this->world) < minDistValidation) {
      *this = *other;
      refresh = true;
      return true;
    } else {
      refresh = false;
      return false;
    }
  }

  Point2f world;
  Point2f image;
  bool refresh;
  float timeDetected;static constexpr float minDistValidation = 0.3f;
  // 30 cms
};
typedef boost::shared_ptr<GoalPost> GoalPostPtr;
