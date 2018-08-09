/**
 * @file LocalizationModule/include/LandmarkDefinitions.h
 *
 * This file declares the landmark positions on the field.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017  
 */

#pragma once

#include "LocalizationModule/include/FieldLandmarkIds.h"

static const RobotPose2D<float> landmarksOnField[NUM_LANDMARKS] =
  { RobotPose2D<float>(4.5f, 0.8f, -M_PI_2), //!Left top goal post
  RobotPose2D<float>(4.5, -0.8, -M_PI_2), //!Right top goal post
  RobotPose2D<float>(-4.5, 0.8, M_PI_2), //!Left bottom goal post
  RobotPose2D<float>(-4.5, -0.8, M_PI_2), //!Right bottom goal post
  RobotPose2D<float>(0.0, 3.0, -M_PI_2), //!Middle T L
  RobotPose2D<float>(0.0, -3.0, M_PI_2), //!Middle T R
  RobotPose2D<float>(4.5, 1.1, M_PI), //!Box T L 1
  RobotPose2D<float>(4.5, -1.1, M_PI), //!Box T R 1
  RobotPose2D<float>(-4.5, 1.1, 0.f), //!Box T L 1
  RobotPose2D<float>(-4.5, -1.1, 0.f), //!Box T R 1
  RobotPose2D<float>(4.5, 3.0, 5 / 4.f * M_PI), //!Corner 1
  RobotPose2D<float>(4.5, -3.0, 3 / 4.f * M_PI), //!Corner 2
  RobotPose2D<float>(-4.5, 3.0, -M_PI / 4.f), //!Corner 3
  RobotPose2D<float>(-4.5, -3.0, M_PI / 4.f), //!Corner 4
  RobotPose2D<float>(3.9, 1.1, -M_PI / 4.f), //!Box Corner L 1
  RobotPose2D<float>(3.9, -1.1, M_PI / 4.f), //!Box Corner R 1
  RobotPose2D<float>(-3.9, 1.1, 5 / 4.f * M_PI), //!Box Corner L 1
  RobotPose2D<float>(-3.9, -1.1, 3 / 4.f * M_PI), //!Box Corner R 1
  RobotPose2D<float>(0.0, 0.0, M_PI_2), //!Center
  RobotPose2D<float>(3.2, 0.0, -M_PI_2), //!Penalty mark 1
  RobotPose2D<float>(-3.2, 0.0, M_PI_2), //!Penalty mark 2
  };
