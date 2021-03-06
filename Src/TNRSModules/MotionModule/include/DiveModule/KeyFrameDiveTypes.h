#pragma once

#include "MotionModule/include/MTypeHeader.h"

enum class KeyFrameDiveTypes : unsigned int
{
  IN_PLACE,
  SUMO,
  LEFT,
  RIGHT
};

static const MType diveInPlace[1][25] =
  { // Time + joints Target
    1.0, 0, 16.0, 119.49952, 3.51911, 0.0, -44.97719, -89.95437, 119.49952,
    -3.51911, 0.0, 44.97719, 89.95437, -40, 5.0, -30, 90, -40, 0.0, -40, -5.0,
    -30, 90, -40, 0.0 };
      
static const MType diveSumo[1][25] =
  { // Time + joints Target
    2.0, 0.0, 16.0, 90.0, 75.0, -67.0, -23.0, 5.0, 90.0, -75.0, 67.0, 23.0,
    5.0, -63, 12.0, -85, 110, 5, -7.3, -63, -12.0, -85, 110, 5, 7.3 };
      
static const MType diveLeft[1][25] =
  { // Time + joints Target
    2.0, 0.0, 16.0, 90.0, 35, -90.0, -3.0, 0.0, 90.0, 0.0, 45.0, 63.0, 0.0,
    -14.2, 18.3, -45, 120, -67, 4.5, -14.2, 0.0, -30.0, 100.0, -50, 7.0 };
      
static const MType diveRight[1][25] =
  { // Time + joints Target
    2.0, 0.0, 16.0, 90.0, 0.0, -45.0, -63.0, 0.0, 90.0, -35, 90.0, 3.0, 0.0,
    -14.2, 0.0, -30.0, 100.0, -50, -7.0, -14.2, -18.3, -45, 120, -67, -4.5 };
