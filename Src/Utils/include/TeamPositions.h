/**
 * @file PlanningModule/PlanningBehaviors/TeamPositions.h
 *
 * This file declares the robot positions on the field on game startup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Dec 2017  
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "Utils/include/RobocupRole.h"
#include "Utils/include/RobotStateDefinitions.h"
#include "Utils/include/MathsUtils.h"

static const RobotPose2D<float> sidePositionsDef[(unsigned)RobocupRole::COUNT] =
  { RobotPose2D<float>(-4.0, 3, -M_PI / 2), //!GoalKeeper
  RobotPose2D<float>(-3.5, -3, M_PI / 2), //!Defender
  RobotPose2D<float>(-3, 3, -M_PI / 2), //!Defense Support
  RobotPose2D<float>(-2.5, -3, M_PI / 2), //!Offense Support
  RobotPose2D<float>(-2.0, 3, -M_PI / 2), //!Attacker
  };

static const RobotPose2D<float> sidePositionsAtt[(unsigned)RobocupRole::COUNT] =
  { RobotPose2D<float>(-4.0, 3, -M_PI / 2), //!GoalKeeper
  RobotPose2D<float>(-3.5, -3, M_PI / 2), //!Defender
  RobotPose2D<float>(-3, 3, -M_PI / 2), //!Defense Support
  RobotPose2D<float>(-2.5, -3, M_PI / 2), //!Offense Support
  RobotPose2D<float>(-1.0, 3, -M_PI / 2), //!Attacker
  };

static const RobotPose2D<float> startPositionsDef[(unsigned)RobocupRole::COUNT] =
  { RobotPose2D<float>(-4.25, 0.0, 0.0), //!GoalKeeper
  RobotPose2D<float>(-2.5, -1.0, 0.0), //!Defender
  RobotPose2D<float>(-2.5, 1.0, 0.0), //!Defense Support
  RobotPose2D<float>(-1.0, -2.0, 0.0), //!Offense Support
  RobotPose2D<float>(-1.0, 1.0, 0.0), //!Attacker
  };

static const RobotPose2D<float> startPositionsAtt[(unsigned)RobocupRole::COUNT] =
  { RobotPose2D<float>(-4.25, 0.0, 0.0), //!GoalKeeper
  RobotPose2D<float>(-2.5, -1.0, 0.0), //!Defender
  RobotPose2D<float>(-2.5, 1.0, 0.0), //!Defense Support
  RobotPose2D<float>(-1.0, -2.0, 0.0), //!Offense Support
  RobotPose2D<float>(-0.75, 0.0, 0.0), //!Attacker
  };

static const RobotPose2D<float> keeperPositionInGame[1] =
  { RobotPose2D<float>(-4.25, 0.0, 0.0), //!GoalKeeper
  };

static const RobotPose2D<float> defPositionsInGame[2] =
  { RobotPose2D<float>(-2.5, -1.0, 0.0), //!Defender
  RobotPose2D<float>(-2.5, 1.0, 0.0), //!Defense Support
  };

static const RobotPose2D<float> offPositionsInGame[2] =
  { RobotPose2D<float>(-1.0, -2.0, 0.0), //!Offense Support
  RobotPose2D<float>(-1.0, 2.0, 0.0), //!Attacker
  };
