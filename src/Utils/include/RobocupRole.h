/**
 * @file Utils/include/RobocupRole.h
 *
 * This file declares the Enumeration RobocupRole
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
enum class RobocupRole
: unsigned int {
  GOALKEEPER = 0,
  DEFENDER,
  DEFENSE_SUPPORT,
  OFFENSE_SUPPORT,
  ATTACKER,
  COUNT
};
