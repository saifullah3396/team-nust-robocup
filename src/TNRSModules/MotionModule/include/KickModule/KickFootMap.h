/**
 * @file MotionModule/include/KickModule/KickFootMap.h
 *
 * This file defines the kick foot mapping based on ball y coordinate and
 * target angle
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017
 */

#pragma once

#include "Utils/include/HardwareIds.h"

const float kickFootMap[14][5] =
{
  { CHAIN_L_LEG, 0.115, 0.125,   0,  90 },
  { CHAIN_L_LEG, 0.1, 0.115,   0,  45 },
  { CHAIN_L_LEG, 0.075, 0.1,   0,  15 },
  { CHAIN_L_LEG, 0.05,   0.075,  -15, 15 },
  { CHAIN_L_LEG, 0.025,  0.05,   -45, 0 },
  { CHAIN_L_LEG, 0.0,  0.025,   -75, -45 },
  { CHAIN_L_LEG, -0.025,   0.0, -90, -75 },
  { CHAIN_R_LEG, 0.0,   0.025, 75, 90 },
  { CHAIN_R_LEG, -0.025,  0.0,   45, 75 },
  { CHAIN_R_LEG, -0.05,  -0.025,   0, 45 },
  { CHAIN_R_LEG, -0.075,   -0.05,  -15, 15 },
  { CHAIN_R_LEG, -0.1, -0.075,   0,  15 },
  { CHAIN_R_LEG, -0.115, -0.1,   -45, 0},
  { CHAIN_R_LEG, -0.125, -0.115,   -90,  -30 }
};
