/**
 * @file Utils/include/PostureState.h
 *
 * This file declares the Enumeration PostureState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the predefined robot postures.
 *
 * @enum Posture
 */
enum class PostureState
: unsigned int {
  CROUCH = 0,
  SIT,
  STAND,
  STAND_HANDS_BEHIND,
  STAND_WALK,
  STATIC_POSTURES,
  GETUP_READY = STATIC_POSTURES,
  FALL_FRONT,
  FALL_BACK,
  FALLING_FRONT,
  FALLING_BACK,
  FALL_SIT,
  DIVE_IN_PLACE,
  DIVE_LEFT,
  DIVE_RIGHT,
  DIVE_SUMO,
  UNKNOWN
};
