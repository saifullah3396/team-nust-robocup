/**
 * @file Utils/include/PositionInput.h
 *
 * This file defines the struct PositionInput
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include "Utils/include/RobotStateDefinitions.h"

/**
 * @struct PositionInput
 * @brief The struct for defining a position update of the robot
 *   in 2D environment.
 */
template<typename Scalar>
struct PositionInput : public RobotPose2D<Scalar>
{
  PositionInput(const Scalar& x, const Scalar& y, const Scalar& theta) :
    RobotPose2D<Scalar>(x, y, theta)
  {
  }
};
template struct PositionInput<float>;
template struct PositionInput<double>;
