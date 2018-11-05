/**
 * @file Utils/include/GoalInfo.h
 *
 * This file defines the struct GoalInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/RobotStateDefinitions.h"

using namespace cv;

/**
 * @struct GoalInfo
 * @brief Holds information about the latest state of the goal
 */
struct GoalInfo
{
  GoalInfo() :
    found(false), ours(-1)
  {
    id = 0;
    leftPost = Point2f(-100, -100);
    rightPost = Point2f(-100, -100);
    mid = Point2f(-100, -100);
    poseFromGoal = RobotPose2D<float>(0.f, 0.f, 0.f);
  }

  void
  assignId()
  {
    unsigned prevId = id;
    do {
      id = rand() % 10 + 1;
    } while (id == prevId);
  }

  bool found;
  unsigned id;
  int ours;
  Point2f leftPost;
  Point2f rightPost;
  Point2f mid;
  RobotPose2D<float> poseFromGoal;
};
