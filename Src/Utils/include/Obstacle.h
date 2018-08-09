/**
 * @file Utils/include/Obstacle.h
 *
 * This file defines the structs Obstacle and ObsObstacles.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 6 Jan 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/ObstacleType.h"

using namespace cv;

/**
 * @struct Obstacle
 * @brief The struct  for defining the possible obstacles in
 *   field.
 */
struct Obstacle
{
  Obstacle()
  {
  }
  Obstacle(const ObstacleType& type) :
    type(type)
  {
  }
  Obstacle(const ObstacleType& type, const Point2f& center) :
    type(type), center(center)
  {
  }
  Obstacle(const ObstacleType& type, const Point2f& center,
    const Point2f& leftBound, const Point2f& rightBound) :
    type(type), center(center), leftBound(leftBound), rightBound(rightBound)
  {
  }
  ObstacleType type;
  Point2f center;
  Point2f leftBound;
  Point2f rightBound;
};

/**
 * @struct ObsObstacles
 * @brief A struct that holds the information about all the latest 
 *   observed obstacles.
 */
struct ObsObstacles
{
  /**
   * Constructor
   */
  ObsObstacles()
  {
    id = 0;
  }

  void
  assignId()
  {
    unsigned prevId = id;
    do {
      id = rand() % 10 + 1;
    } while (id == prevId);
  }

  //! Identity of the current observed obstacles
  unsigned id;

  //! vector of Obstacles
  vector<Obstacle> data;
};
