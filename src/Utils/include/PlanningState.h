/**
 * @file Utils/include/PlanningState.h
 *
 * This file declares the Enumeration PlanningState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the major states of the robot planning.
 *
 * @enum PlanningState
 */
enum class PlanningState
: unsigned int { // Update this cuz this has no purpose for now
  STARTUP = 0,
  ROBOCUP,
  ROBOCUP_PENALTIES,
  UNKNOWN,
  COUNT
};
