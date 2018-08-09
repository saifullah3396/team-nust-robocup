#pragma once

//! See Utils/include/PostureState.h
//! WARNING: These postures are defined according to the enumeration in
//! PostureState.h. For new postures, add the posture state in PostureState.h 
//! in the respective sequence 

static const float postureDefinitions[5][24] =
  {
    // Crouch
      { 0.0, 0.0, 80.0, 9.5, -46.0, -60.0, 7.5, 80.0, -9.5, 46.0, 60.0, -7.5,
        -14.5, -4.5, -40.7, 123.0, -68.7, 4.3, -14.5, 4.5, -40.7, 123.0, -68.7,
        -4.3 },
    // Sit
      { 0.0, 0.0, 90.0, 11.5, -90.0, 0.0, 90.0, 90.0, -11.5, 90.0, 0.0, -90.0,
        -14.5, -4.5, -40.7, 123.0, -68.7, 4.3, -14.5, 4.5, -40.7, 123.0, -68.7,
        -4.3 },
    // Stand
      { 0.0, 16.0, 90.0, 11.5, -90.0, 0.0, -90.0, 90.0, -11.5, 90.0, 0.0, 90.0,
        0.0, 0.0, -26.0, 40.0, -20.0, 0.0, 0.0, 0.0, -26.0, 40.0, -20.0, 0.0 },
    // Stand with hands behind
      { 0.0, 16.0, 119.5, 3.5, 0.0, -44.9, -90.0, 119.5, -3.5, 0.0, 44.9, 90.0,
        0.0, 0.0, -26.0, 40.0, -20.0, 0.0, 0.0, 0.0, -26.0, 40.0, -20.0, 0.0 },
    // Stand for walk with hands behind
      { 0.0, 16.0, 119.5, 3.5, 0.0, -44.9, -90.0, 119.5, -3.5, 0.0, 44.9, 90.0,
        0.0, 0.0, -20.0, 50.0, -30.0, 0.0, 0.0, 0.0, -20.0, 50.0, -30.0, 0.0 } };
