/**
 * @file MotionModule/include/KinematicsModule/TorsoState.h
 *
 * This file defines the struct TorsoState
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "Utils/include/MathsUtils.h"

/**
 * @struct TorsoState
 * @brief Robot torso state definition
 */
struct TorsoState
{
  /**
   * Constructor
   */ 
  TorsoState()
  {
    velocity.setZero();
    accel.setZero();
    rot.setIdentity();
  }
  
  //! Torso velocity
  Vector3f velocity;
  
  //! Torso acceleration
  Vector3f accel;
  
  //! Torso rotation
  Matrix3f rot;
};
