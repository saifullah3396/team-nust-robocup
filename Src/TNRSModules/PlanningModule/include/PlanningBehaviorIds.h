/**
 * @file PlanningModule/include/PlanningBehaviorIds.h
 *
 * This file declares the enumeration for all the behavior planning 
 * behaviors and their childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the planning behaviors
 *
 * @enum PBIds
 */
enum class PBIds
: unsigned int {
  STARTUP = 0,
  ROBOCUP,
  KICK_SEQUENCE
};

/**
 * Enumeration for all possible types of startup behavior
 *
 * @enum PBStartupTypes
 */
enum class PBStartupTypes
: unsigned int
{
  REQUEST_BEHAVIOR
};

/**
 * Enumeration for all possible types of robocup behavior
 *
 * @enum PBRobocupTypes
 */
enum class PBRobocupTypes
: unsigned int
{
  ROBOCUP_SETUP = 0,
  GOAL_KEEPER,
  DEFENDER,
  ATTACKER,
  PENALTIES
};

/**
 * Enumeration for all possible types of kick sequence behavior
 *
 * @enum PBKickSequenceTypes
 */
enum class PBKickSequenceTypes
: unsigned int
{
  BALL_INTERCEPT
};

/**
 * Enumeration for all the goal keeper behavior types.
 *
 * @enum PBGoalKeeperTypes
 */
/*enum class PBGoalKeeperTypes
: unsigned int
{
  PASSIVE_GK,
  COUNT
};*/

/** 
 * Enumeration for all defender behavior types.
 *
 * @enum PBDefenderTypes
 */
/*enum class PBDefenderTypes
: unsigned int
{
  MAIN_DEF,
  SUPPORT_DEF,
  COUNT
};*/

/**
 * Enumeration for all attacker behavior types.
 *
 * @enum PBAttackerTypes
 */
/*enum class PBAttackerTypes
: unsigned int
{
  MAIN_ATT,
  SUPPORT_ATT,
  COUNT
};*/
