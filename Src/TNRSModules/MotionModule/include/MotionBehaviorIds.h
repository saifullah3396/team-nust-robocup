/**
 * @file MotionModule/include/MotionBehaviorIds.h
 *
 * This file declares the enumeration for all motion behaviors and their
 * child types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

/**
 * Enumeration for all motion behaviors
 *
 * @enum MBIds
 */
enum class MBIds
: unsigned int {
  POSTURE,
  KICK,
  MOVEMENT,
  BALL_THROW,
  BALANCE,
  HEAD_CONTROL,
  DIVE,
  GETUP,
  COUNT
};

/**
 * Enumeration for all possible posture behavior types
 *
 * @enum MBPostureTypes
 */
enum class MBPostureTypes
: unsigned int {
  INTERP_TO_POSTURE,
  COUNT
};

/**
 * Enumeration for all possible kick behavior types
 *
 * @enum MBKickTypes
 */
enum class MBKickTypes
: unsigned int {
  JOINT_SPACE_OPT_IMP_KICK,
  JOINT_SPACE_EST_2D_IMP_KICK,
  COUNT
};

/**
 * Enumeration for all possible ball throw behavior types
 *
 * @enum MBBallThrowTypes
 */
enum class MBBallThrowTypes
: unsigned int {
  WB_BALL_THROW,
  COUNT
};

/**
 * Enumeration for all possible balance behavior types
 *
 * @enum MBBalanceTypes
 */
enum class MBBalanceTypes
: unsigned int {
  MP_COM_CONTROL,
  PID_COM_CONTROL,
  ZMP_CONTROL,
  COUNT
};

/**
 * Enumeration for all possible movement behavior types
 *
 * @enum MBMovementTypes
 */
enum class MBMovementTypes
: unsigned int {
  GO_TO_TARGET,
  COUNT
};

/**
 * Enumeration for all possible head control behavior types
 *
 * @enum MBHeadControlTypes
 */
enum class MBHeadControlTypes
: unsigned int {
  HEAD_TARGET_TRACK,
  HEAD_TARGET_SEARCH,
  COUNT
};

/**
 * Enumeration for all possible dive behavior types
 *
 * @enum MBDiveTypes
 */
enum class MBDiveTypes
: unsigned int {
  KEY_FRAME_MOTION_DIVE,
  COUNT
};

/**
 * Enumeration for all possible getting up behavior types
 *
 * @enum MBGetupTypes
 */
enum class MBGetupTypes
: unsigned int {
  KEY_FRAME_MOTION_GETUP,
  COUNT
};
