/**
 * @file SBModule/include/StaticBehaviorIds.h
 *
 * This file defines the enumerations for all static behavior ids and
 * their child types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the available static behaviors
 *
 * @enum SBIds
 */
enum class SBIds
: unsigned int {
  STIFFNESS,
  LEDS,
  WHISTLE_DETECTOR,
};

/**
 * Enumeration for all possible stiffness behavior types
 *
 * @enum SBStiffnessTypes
 */
enum class SBStiffnessTypes
: unsigned int {
  STIFFNESS_INTERP
};

/**
 * Enumeration for all possible led behavior types
 *
 * @enum SBLedsTypes
 */
enum class SBLedsTypes
: unsigned int {
  DIRECT_LEDS,
  INTERPOLATE_LEDS
};

/**
 * Enumeration for all possible whistle detector behavior types
 *
 * @enum SBWDTypes
 */
enum class SBWDTypes
: unsigned int {
  AK_WHISTLE_DETECTOR
};
