#pragma once

#include "Utils/include/MathsUtils.h"

enum class KeyFrameGetupTypes : unsigned int
{
  FRONT,
  BACK,
  //! GETUP FROM SIT NOT TESTED DO NO USE WITHOUT TESTING IN SIMULATIONS FIRST
  SIT
};

static const float getupFromFront[16][25] =
  { // Time + joints Target
    { 0.4, 0.0, 0.0, 107.8, 74.4, -35.7, -80.2, -90, 98.6, -73.7, 5.1, 73.4, 90,
      0, -0, 0, 0, 60, -0, 0, 0, 0, 0, 60, 0 },
    { 0.3, 0.0, 0.0, 107.8, 74.4, -35.7, -80.2, -90, 98.6, -73.7, 5.1, 73.4, 90,
      -67.8, 39.2, 28, -6.8, -11.4, 35.2, -67.8, -43.2, 28.4, -5.8, -9, -35.8 },
    { 0.4, 0.0, 0.0, 116.8, -0.4, 111, -89.2, -90, 120.9, 20, -75.6, 89.3, 90,
      -68.3, 41.5, -88.1, -3.8, 11.1, 32.4, -68.3, -46.1, 25.9, -4.7, 17.1,
      -31.1 },
    { 0.35, 0.0, 0.0, 118.6, -0.4, 111.6, -89.3, -90, 121.3, 20, -75.4, 89.3,
      90, -68.9, 40.1, -91.2, -6.6, 10.9, 32.6, -68.9, -39.2, -92.4, 121.9,
      13.6, -30.8 },
    { 0.4 * 2, 0.0, 0.0, 119.2, -0.4, 111.8, -89.3, -90, 119.8, 20, -75.4, 89.1,
      90, -69.3, 42.1, -91.3, -7, 11.9, 32.5, -69.3, -45.4, -92.3, -7, 14.7,
      -23.1 },
      { 0.8 * 2, 0.0, 0.0, 111.7, -4.2, 90.6, 0.4, -90, 110, 2.6, -76.9, 8.6,
        90, -29.3, -1.4, -68.1, -0.7, -22.6, 16.4, -29.3, -2.3, -74.4, 1.3, 0.1,
        -17.8 },
      { 0.5, 0.0, 0.0, 119.8, 7.8, -71.1, -31.4, -90, 121.6, -0.4, 87.5, 22.2,
        90, -56.3, 15.8, -50, -6.9, 8.8, -12.4, -56.3, -19, -49.1, -6.9, 16,
        -4.7 },
      { 0.5, 6.9, 30.0, 119.9, -4.8, -85.3, -18.6, -90, 121.6, -0.9, 87.4, 22.4,
        90, -65, 41.7, -80.2, 76.5, 49.3, -7.2, -65, -32.5, -84, 81.8, 47.1, 1.1 },
      { 0.5, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7, 99.4, 8.2,
        90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3, -89.4, 48.2,
        52.6, -1 },
      { 0.1, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7, 99.4, 8.2,
        90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3, -89.4, 48.2,
        52.6, -1 },
      { 0.5, 4.8, 29.6, 99.7, 21, -84.9, -15, -90, 107.1, -20.2, 99.1, 8.8, 90,
        -64.8, 4.1, -32.9, 124.6, -57.9, 10, -64.8, -15, -18.2, -5.9, 62.6, 4 },
      { 0.5, 5.3, 29.7, 89.6, 20.6, -84.9, -15, -90, 99, -0.4, 99, 8.5, 90,
        -43.5, 12.4, -55.3, 124.7, -55.8, 9.1, -43.5, 13.6, -44, 95.1, -15.9,
        15.1 },
      { 0.5, 5.0, 29.6, 87.4, 18.9, -85, -14.4, -90, 81, -11, 99.1, 7.8, 90,
        -32.9, -7.4, -60, 124.7, -48.8, 7.5, -32.9, 8.8, -57.1, 124.6, -50.7,
        -8.8 },
      { 0.3, 4.9, 30.0, 82, 12.8, -84.1, -12.1, -90, 75.4, -6.8, 99.2, 6.8, 90,
        0.2, -3.8, -53.3, 124.6, -69.8, 1.9, 0.2, 5.8, -52.2, 124.6, -70, -4.8 },
      { 0.5, 0.0, 30.0, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90, 0,
        -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 },
      { 0.1, 0.0, 30.0, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90, 0,
        -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 } };

static const float getupFromBack[19][25] =
    {
      { 0.300, 0, 30, 97.8, 13, 1.8, -2.1, -0, 96.1, -17.7, -19.8, 11.3, 0, 3.1,
        -0.4, -6.7, 13.1, 64.1, -4.2, 3.1, -2.6, -7.8, 16.4, 64.6, 4.6 },
      { 0.250, 0, 30, 97.7, 14.9, 1.6, -1.2, -0, 96.2, -17.1, -20.2, 11.2, 0,
        -4.8, 1.8, -31.9, 84, 41.9, 4, -4.8, -1.1, -36.4, 92.7, 37.5, -2.5 },
      { 0.250, 0, 30.3, 122.3, 70.3, 9.1, -90, -0, 121.3, -69.7, -9.1, 90, 0, 1,
        -7.9, 21.9, 96.9, -3.2, -1.1, 1, -7, 25.4, 102.7, -12, 1.2 },
        { 0.200, -0.4, 30.3, 123.6, 8, 9, -90, -0, 120.9, -12.3, -6.9, 85.3, 0,
          -0.2, -8.3, 19.1, 102.1, -5.2, 0.7, -0.2, -6.5, 24.4, 107.2, -15.3,
          1.4 },
        { 0.300 * 1.25, 0, 30, 117.2, 3.7, -1.9, -70.9, -0, 122.9, -11.4, 6.5,
          77.1, 0, 6, 2.5, 24, 7.9, 64.7, -4.7, 6, -0.1, 25.7, 7.1, 64.9, 4.9 },
        { 0.300, 0, 30, 116.5, 9.4, -0.8, -78.9, -0, 112.5, -11.3, -7.8, 78.4,
          0, 3, 3.5, -6.1, -7, 63.9, -4.3, 3, 4, -4.8, -6.9, 64.3, 4.5 },
        { 0.500, 0.2, 22.5, 123.4, 10.3, 18.7, -79.6, -0, 123.1, -13.4, -23.9,
          81.7, 0, 5.3, 4.6, -48.9, -5.2, 53, 1.6, 5.3, 1.9, -47.6, -6, 53.6,
          1.1 },
        { 0.350, 3.6, 29.8, 112.2, -8.6, -3.3, -15.9, -90, 115.3, 10.7, 45,
          15.6, 90, -26.7, -1.6, -91.7, 45, -70, 0.4, -26.7, 0.7, -92, -2, -70,
          -2.7 },
        { 0.300, 3.6, 29.8, 112.2, -8.6, -3.3, -15.9, -90, 115.3, 10.7, -2.6,
          15.6, 90, -26.7, -1.6, -91.7, -1.5, -70, 0.4, -26.7, 0.7, -92, -2,
          -70, -2.7 },
        { 0.500, 7.5, 30, 119.8, 7.8, -71.1, -31.4, -90, 121.6, -0.4, 87.5,
          22.2, 90, -56.3, 15.8, -50, -6.9, 8.8, -12.4, -56.3, -19, -49.1, -6.9,
          16, -4.7 },
        { 0.500, 6.9, 30, 119.9, -4.8, -85.3, -18.6, -90, 121.6, -0.9, 87.4,
          22.4, 90, -65, 41.7, -80.2, 76.5, 49.3, -7.2, -65, -32.5, -84, 81.8,
          47.1, 1.1 },
        { 0.500, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7, 99.4,
          8.2, 90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3, -89.4,
          48.2, 52.6, -1 },
        { 0.100 * 1, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7,
          99.4, 8.2, 90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3,
          -89.4, 48.2, 52.6, -1 },
        { 0.500 * 1, 4.8, 29.6, 99.7, 21, -84.9, -15, -90, 107.1, -20.2, 99.1,
          8.8, 90, -64.8, 4.1, -32.9, 124.6, -57.9, 10, -64.8, -15, -18.2, -5.9,
          62.6, 4 },
        { 0.500 * 1, 5.3, 29.7, 89.6, 20.6, -84.9, -15, -90, 99, -0.4, 99, 8.5,
          90, -43.5, 12.4, -55.3, 124.7, -55.8, 9.1, -43.5, 13.6, -44, 95.1,
          -15.9, 15.1 },
        { 0.500 * 1, 5, 29.6, 87.4, 18.9, -85, -14.4, -90, 81, -11, 99.1, 7.8,
          90, -32.9, -7.4, -60, 124.7, -48.8, 7.5, -32.9, 8.8, -57.1, 124.6,
          -50.7, -8.8 },
        { 0.300 * 1, 4.9, 30, 82, 12.8, -84.1, -12.1, -90, 75.4, -6.8, 99.2,
          6.8, 90, 0.2, -3.8, -53.3, 124.6, -69.8, 1.9, 0.2, 5.8, -52.2, 124.6,
          -70, -4.8 },
        { 0.500 * 1, 0, 30, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90,
          0, -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 },
        { 0.100 * 1, 0, 30, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90,
          0, -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 } };

//! NOT TESTED DO NO USE WITHOUT TESTING IN SIMULATIONS FIRST
static const float getupFromSit[10][25] =
    {
        { 0.5, 7.5, 30, 119.8, 7.8, -71.1, -31.4, -90, 121.6, -0.4, 87.5,
          22.2, 90, -56.3, 15.8, -50, -6.9, 8.8, -12.4, -56.3, -19, -49.1, -6.9,
          16, -4.7 },
        { 0.5, 6.9, 30, 119.9, -4.8, -85.3, -18.6, -90, 121.6, -0.9, 87.4,
          22.4, 90, -65, 41.7, -80.2, 76.5, 49.3, -7.2, -65, -32.5, -84, 81.8,
          47.1, 1.1 },
        { 0.5, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7, 99.4,
          8.2, 90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3, -89.4,
          48.2, 52.6, -1 },
        { 0.1 * 1, 5.5, 29.4, 115.8, -7.6, -84.6, -15.9, -90, 120.7, -39.7,
          99.4, 8.2, 90, -68.9, 11.7, -29.7, 124.6, -40.9, -18.9, -68.9, -20.3,
          -89.4, 48.2, 52.6, -1 },
        { 0.5 * 1, 4.8, 29.6, 99.7, 21, -84.9, -15, -90, 107.1, -20.2, 99.1,
          8.8, 90, -64.8, 4.1, -32.9, 124.6, -57.9, 10, -64.8, -15, -18.2, -5.9,
          62.6, 4 },
        { 0.5 * 1, 5.3, 29.7, 89.6, 20.6, -84.9, -15, -90, 99, -0.4, 99, 8.5,
          90, -43.5, 12.4, -55.3, 124.7, -55.8, 9.1, -43.5, 13.6, -44, 95.1,
          -15.9, 15.1 },
        { 0.5 * 1, 5, 29.6, 87.4, 18.9, -85, -14.4, -90, 81, -11, 99.1, 7.8,
          90, -32.9, -7.4, -60, 124.7, -48.8, 7.5, -32.9, 8.8, -57.1, 124.6,
          -50.7, -8.8 },
        { 0.3 * 1, 4.9, 30, 82, 12.8, -84.1, -12.1, -90, 75.4, -6.8, 99.2,
          6.8, 90, 0.2, -3.8, -53.3, 124.6, -69.8, 1.9, 0.2, 5.8, -52.2, 124.6,
          -70, -4.8 },
        { 0.5 * 1, 0, 30, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90,
          0, -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 },
        { 0.1 * 1, 0, 30, 90, 11.5, -90, -11.5, -90, 90, -11.5, 90, 11.5, 90,
          0, -0, -45, 71.4, -35.1, -0, 0, 0, -45, 71.4, -35.1, 0 } };

static const Vector3f frontPoseOffset = Vector3f(0.23, -0.04, -0.785398);
static const Vector3f backPoseOffset = Vector3f(-0.02, -0.05, -0.5);
