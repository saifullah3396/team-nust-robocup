/**
 * @file Utils/include/ObstacleType.h
 *
 * This file defines the enum ObstacleType
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 6 Jan 2018
 */

#pragma once

/**
 * Enumeration for the possible type of obstacles
 *
 * @enum ObstacleType
 */
enum class ObstacleType
: unsigned int
{
  UNKNOWN = 0,
  GOALPOST,
  OPPONENT,
  TEAMMATE,
  OPPONENT_FALLEN,
  TEAMMATE_FALLEN,
  NUM_TYPES
};
