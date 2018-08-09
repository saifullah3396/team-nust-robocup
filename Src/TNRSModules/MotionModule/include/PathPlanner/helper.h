/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "MotionModule/include/PathPlanner/GridMap2D.h"
#include "MotionModule/include/PathPlanner/angles.h"
#include <math.h>

namespace PathPlannerSpace
{

#define DEBUG_HASH 0
#define DEBUG_TIME 0

  static const double TWO_PI = 2 * M_PI;

  static const double FLOAT_CMP_THR = 0.0001;

  enum Leg
  {
    RIGHT = 0,
    LEFT = 1,
    NOLEG = 2
  };

  /**
   * @return Squared euclidean distance between two integer coordinates
   * (cells).
   */
  inline double
  euclideanDistanceSq(int x1, int y1, int x2, int y2)
  {
    // note: do *not* use pow() to square!
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
  }

/// @return Euclidean distance between two integer coordinates (cells).
  inline double
  euclideanDistance(int x1, int y1, int x2, int y2)
  {
    return sqrt(double(euclideanDistanceSq(x1, y1, x2, y2)));
  }

/// @return Euclidean distance between two coordinates.
  inline double
  euclideanDistance(double x1, double y1, double x2, double y2)
  {
    return sqrt(euclideanDistanceSq(x1, y1, x2, y2));
  }

/// @return Squared euclidean distance between two coordinates.
  inline double
  euclideanDistanceSq(double x1, double y1, double x2, double y2)
  {
    // note: do *not* use pow() to square!
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
  }

/// @return The distance of two neighbored cell.
  inline double
  gridCost(int x1, int y1, int x2, int y2, float cellSize)
  {
    int x = abs(x1 - x2);
    int y = abs(y1 - y2);

    if (x + y > 1) return M_SQRT2 * cellSize;
    else return cellSize;
  }

/// @brief Discretize a (continuous) angle into a bin.
  inline int
  angleState2Cell(double angle, int angleBinNum)
  {
    double binSizeHalf = M_PI / angleBinNum;
    return int(
      angles::normalizeAnglePositive(angle + binSizeHalf) / TWO_PI * angleBinNum);
  }

/// @brief Calculate the respective (continuous) angle for a bin.
  inline double
  angleCell2State(int angle, int angleBinNum)
  {
    double binSize = TWO_PI / angleBinNum;
    return angle * binSize;
  }

  /**
   * @brief Discretize a (continuous) state value into a cell. (Should be
   * used to discretize a State to a PlanningState.)
   */
  inline int
  state2Cell(float value, float cellSize)
  {
    return value >= 0 ? int(value / cellSize) : int(value / cellSize) - 1;
  }

  /**
   * @brief Calculate the respective (continuous) state value for a cell.
   * (Should be used to get a State from a discretized PlanningState.)
   */
  inline double
  cell2State(int value, double cellSize)
  {
    return (double(value) + 0.5) * cellSize;
  }

/// @brief Discretize a (continuous) value into cell size.
// TODO: check consistency for negative values
  inline int
  discVal(double length, double cellSize)
  {
    return int(floor((length / cellSize) + 0.5));
  }

  /**
   * @brief Calculates the continuous value for a length discretized in cell
   * size.
   */
// TODO: check consistency for negative values
  inline double
  contVal(int length, double cellSize)
  {
    return double(length * cellSize);
  }

/// @return The hash value of the key.
  inline unsigned int
  intHash(int key)
  {
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
  }

  /**
   * @return The hash tag for a PlanningState (represented by x, y, theta and
   * leg).
   */
  inline unsigned int
  calcHashTag(int x, int y, int theta, int leg, int maxHashSize)
  {
    return intHash(
      (intHash(x) << 3) + (intHash(y) << 2) + (intHash(theta) << 1) + (intHash(
        leg))) % maxHashSize;
  }

  /**
   * @brief Checks if a footstep (represented by its center and orientation)
   * collides with an obstacle. The check is done by recursively testing if
   * either the circumcircle of the foot, the inner circle of the foot or the
   * area in between has an appropriate distance to the nearest obstacle.
   *
   * @param x Global position of the foot in x direction.
   * @param y Global position of the foot in y direction.
   * @param theta Global orientation of the foot.
   * @param height Size of the foot in x direction.
   * @param width Size of the foot in y direction.
   * @param accuracy (0) circumcircle of the foot; (1) incircle of the foot;
   * (2) circumcircle and incircle recursivly checked for the whole foot
   * @param distanceMap Contains distance information to the nearest
   * obstacle.
   *
   * @return True if the footstep collides with an obstacle.
   */
  bool
  collisionCheck(double x, double y, double theta, double height, double width,
    int accuracy, const GridMap2D& distanceMap);

  /**
   * @brief Crossing number method to determine whether a point lies within a
   * polygon or not.
   * @param edges (x,y)-points defining the polygon.
   *
   * Check http://geomalgorithms.com/a03-_inclusion.html for further details.
   */
  bool
  pointWithinPolygon(int x, int y,
    const std::vector<std::pair<int, int> >& edges);

}
;
