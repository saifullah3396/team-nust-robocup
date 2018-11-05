/**
 * @file Utils/include/StiffnessState.h
 *
 * This file declares the Enumeration StiffnessState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the states of the robot stiffnesses.
 *
 * @enum Stiffness
 */
enum class StiffnessState
: unsigned int {
  MIN,
  MAX,
  ROBOCUP,
  GETUP,
  UNKNOWN,
  NUM_STATES
};
