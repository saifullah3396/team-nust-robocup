/**
 * @file MotionModule/KinematicsModule/KinematicsConsts.h
 *
 * This file defines the constants for the robot kinematic detail.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 08 Feb 2017
 */

#pragma once

#define shoulderOffsetY  0.098
#define elbowOffsetY  0.015
#define upperArmLength  0.105
#define shoulderOffsetZ  0.100
#define lowerArmLength  0.05595
#define handOffsetX   0.05775
#define handOffsetZ   0.01231
#define hipOffsetZ   0.085
#define hipOffsetY   0.050
#define thighLength   0.100
#define tibiaLength   0.1029
#define footHeight   0.04519
#define neckOffsetZ   0.1265
#define cameraBottomX  0.05071
#define cameraBottomZ  0.01774
#define cameraTopX   0.05871
#define cameraTopZ   0.06364
#define cameraTopAngleY  0.020944
#define cameraBotAngleY  0.6929

#define headYawHigh   2.0857
#define headYawLow   -2.0857
#define headPitchHigh  0.5149
#define headPitchLow  -0.6720

#define lShoulderPitchHigh 2.0857
#define lShoulderPitchLow  -2.0857
#define lShoulderRollHigh 1.3265
#define lShoulderRollLow  -0.3142
#define lElbowYawHigh  2.0875
#define lElbowYawLow   -2.0875
#define lElbowRollHigh   -0.0349f
#define lElbowRollLow   -1.5446
#define lWristYawHigh  1.8238
#define lWristYawLow  -1.8238

#define rShoulderPitchHigh  2.0857
#define rShoulderPitchLow  -2.0857
#define rShoulderRollHigh 0.3142
#define rShoulderRollLow -1.3265
#define rElbowYawHigh   2.0875
#define rElbowYawLow   -2.0875
#define rElbowRollHigh  1.5446
#define rElbowRollLow  0.0349f
#define rWristYawHigh  1.8238
#define rWristYawLow  -1.8238

#define lHipYawPitchHigh 0.7408
#define lHipYawPitchLow  -1.1453
#define lHipRollHigh  0.7904
#define lHipRollLow   -0.3794
#define lHipPitchHigh  0.4840
#define lHipPitchLow  -1.7739
#define lKneePitchHigh  2.1125
#define lKneePitchLow  -0.0923
#define lAnklePitchHigh  0.9227
#define lAnklePitchLow  -1.1895
#define lAnkleRollHigh  0.7690
#define lAnkleRollLow  -0.3978

#define rHipYawPitchHigh 0.7408
#define rHipYawPitchLow  -1.1453
#define rHipRollHigh  0.4147
#define rHipRollLow   -0.7383
#define rHipPitchHigh  0.4856
#define rHipPitchLow  -1.7723
#define rKneePitchHigh  2.1201
#define rKneePitchLow  -0.1030
#define rAnklePitchHigh  0.9320
#define rAnklePitchLow  -1.1864
#define rAnkleRollHigh  0.3886
#define rAnkleRollLow  -1.1864

#define totalMassH25  (5.3053+0.345)

#define torsoMass   1.0496
#define torsoX    -0.00413
#define torsoY    0.0
#define torsoZ    0.04342

#define batteryMass   0.345
#define batteryX   -0.030
#define batteryY   0.00
#define batteryZ   0.039

#define headYawMass   0.07842
#define headYawX   -0.00001
#define headYawY   0.0//0.00014
#define headYawZ   -0.02742

#define headPitchMass  0.60533
#define headPitchX   -0.00112
#define headPitchY   0.0
#define headPitchZ   0.05258

#define rShoulderPitchMass 0.09304
#define rShoulderPitchX  -0.00165
#define rShoulderPitchY  0.02663
#define rShoulderPitchZ  0.00014

#define rShoulderRollMass 0.15777
#define rShoulderRollX  0.02455
#define rShoulderRollY  -0.00563
#define rShoulderRollZ  0.0032

#define rElbowYawMass  0.06483
#define rElbowYawX   -0.02744
#define rElbowYawY   0.00
#define rElbowYawZ   -0.00014

#define rElbowRollMass  0.07761
#define rElbowRollX   0.02556
#define rElbowRollY   -0.00281
#define rElbowRollZ   0.00076

#define rWristYawMass  0.18533
#define rWristYawX   0.03434
#define rWristYawY   0.00088
#define rWristYawZ   0.00308

#define rHipYawPitchMass 0.06981
#define rHipYawPitchX  -0.00781
#define rHipYawPitchY  0.01114
#define rHipYawPitchZ  0.02661

#define rHipRollMass  0.14053
#define rHipRollX   -0.01549
#define rHipRollY   -0.00029
#define rHipRollZ   -0.00515

#define rHipPitchMass  0.38968
#define rHipPitchX   0.00138
#define rHipPitchY   -0.00221
#define rHipPitchZ   -0.05373

#define rKneePitchMass  0.30142
#define rKneePitchX   0.00453
#define rKneePitchY   -0.00225
#define rKneePitchZ   -0.04936

#define rAnklePitchMass  0.13416
#define rAnklePitchX  0.00045
#define rAnklePitchY  -0.00029
#define rAnklePitchZ  0.00685

#define rAnkleRollMass  0.17184
#define rAnkleRollX   0.02542
#define rAnkleRollY   -0.0033
#define rAnkleRollZ   -0.03239

#define headYawVelLimit 8.26797
#define headPitchVelLimit 7.19407
#define lShoulderPitchVelLimit 8.26797
#define lShoulderRollVelLimit 7.19407
#define lElbowYawVelLimit 8.26797
#define lElbowRollVelLimit 7.19407
#define lWristYawVelLimit 24.6229
#define rShoulderPitchVelLimit 8.26797
#define rShoulderRollVelLimit 7.19407
#define rElbowYawVelLimit 8.26797
#define rElbowRollVelLimit 7.19407
#define rWristYawVelLimit 24.6229
#define lHipYawPitchVelLimit 4.16174
#define lHipRollVelLimit 4.16174
#define lHipPitchVelLimit 6.40239
#define lKneePitchVelLimit 6.40239
#define lAnklePitchVelLimit 6.40239
#define lAnkleRollVelLimit 4.16174
#define rHipYawPitchVelLimit 4.16174
#define rHipRollVelLimit 4.16174
#define rHipPitchVelLimit 6.40239
#define rKneePitchVelLimit 6.40239
#define rAnklePitchVelLimit 6.40239
#define rAnkleRollVelLimit 4.16174
