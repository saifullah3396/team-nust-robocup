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
template<typename Scalar>
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
  Matrix<Scalar, 3, 1> velocity;
  
  //! Torso acceleration
  Matrix<Scalar, 3, 1> accel;
  
  //! Torso rotation
  Matrix<Scalar, 4, 4> rot;
};
template struct TorsoState<float>;
template struct TorsoState<double>;
