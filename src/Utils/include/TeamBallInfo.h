/**
 * @file Utils/include/TeamBallInfo.h
 *
 * This file defines the struct TeamBallInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

/**
 * @struct TeamBallInfo
 * @brief Holds information about the latest state of the ball extracted
 *   from teammates' data.
 */
struct TeamBallInfo
{
  TeamBallInfo() :
    found(false)
  {
  }

  bool found;
  Point2f posWorld;
  Point2f velWorld;
};
