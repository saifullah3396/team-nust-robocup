/**
 * @file MotionModule/include/KinematicsModule/ComState.h
 *
 * This file defines the struct ComState
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "Utils/include/MathsUtils.h"

/**
 * @struct ComState
 * @brief Robot torso state definition
 */
struct ComState
{
  /**
   * Constructor
   */ 
  ComState()
  {
    position.setZero();
    velocity.setZero();
    accel.setZero();
    baseFrame = -1;
  }

  //! The inertial frame for com position.
  //! Currently either left or right foot is used
  unsigned baseFrame; 

  //! Torso velocity
  Vector3f position;

  //! Torso velocity
  Vector3f velocity;
  
  //! Torso acceleration
  Vector3f accel;
};
