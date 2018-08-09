/**
 * @file PlanningModule/MotionConfigs.h
 *
 * This file defines all the motion behavior configurations.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "Utils/include/RobotStateDefinitions.h"

/**
 * @struct MBMovementConfig
 * @brief Movement behavior configuration
 */
struct MBMovementConfig : MBConfig
{

  MBMovementConfig() :
    MBConfig(MBIds::MOVEMENT, 360.f, -1) // No child type for now
  {
    //this->type = MBMovementTypes::GO_TO_TARGET;
    this->goal = RobotPose2D<float>(0, 0, 0);
    this->ballTrack = false;
    this->reachClosest = true;
  }

  MBMovementConfig(const MBMovementTypes& type, const RobotPose2D<float>& goal) :
    MBConfig(MBIds::MOVEMENT, 45.f, -1) // No child type for now
  {
    //this->type = type;
    this->goal = goal;
    this->ballTrack = false;
    this->reachClosest = true;
  }

  bool ballTrack;
  bool reachClosest;
  RobotPose2D<float> goal;
  //MBMovementTypes type;
};
