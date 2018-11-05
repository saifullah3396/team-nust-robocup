/**
 * @file Utils/include/OccupancyMap.h
 *
 * This file defines the struct OccupancyMap
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

/**
 * @struct OccupancyMap
 * @brief Holds the environment's occupancy map information.
 */
struct OccupancyMap
{
  OccupancyMap() :
    resolution(0)
  {
  }
  float resolution;
  Point3f originPose;
  Mat data;
};
