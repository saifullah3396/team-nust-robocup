/**
 * @file MotionModule/include/KinematicsModule/JointStateType.h
 *
 * This file defines the enum JointStateType
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

/**
 * Type of joint states possible for computation
 *
 * @enum JointStateType
 */
enum class JointStateType : unsigned int
{
  ACTUAL,
  SIM,
  COUNT
};
