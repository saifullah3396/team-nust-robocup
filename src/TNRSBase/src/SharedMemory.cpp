/**
 * @file TNRSBase/src/SharedMemory.cpp
 *
 * This file implements the class SharedMemory
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "Utils/include/ConfigMacros.h"
#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/Multiplexer.h"
#include "TNRSBase/include/SharedMemory.h"
#include "BehaviorManager/include/BehaviorInfo.h"
#include "CommModule/include/ClientInfo.h"
#include "Utils/include/BallInfo.h"
#include "Utils/include/Camera.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/OccupancyMap.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/RoboCupGameControlData.h"
#include "Utils/include/RobotStateDefinitions.h"
#include "Utils/include/SPLStandardMessage.h"
#include "Utils/include/StiffnessState.h"
#include "Utils/include/TeamRobot.h"
#include "Utils/include/TeamBallInfo.h"
#include "Utils/include/Variable.h"

void
SharedMemory::init()
{
  variables.assign(numberOfVariables, NULL);
  int ctPeriod, mtPeriod, ptPeriod, sbtPeriod, vtPeriod, ltPeriod, commtPeriod;
  GET_CONFIG(
    "BaseModules",
    (int, ControlModule.period, ctPeriod),
    (int, MotionModule.period, mtPeriod),
    (int, PlanningModule.period, ptPeriod),
    (int, SBModule.period, sbtPeriod),
    (int, VisionModule.period, vtPeriod),
    (int, LocalizationModule.period, ltPeriod),
    (int, CommModule.period, commtPeriod),
  )
  DEFINE_VARIABLE(int, controlThreadPeriod, ctPeriod);
  DEFINE_VARIABLE(int, motionThreadPeriod, mtPeriod); //15
  DEFINE_VARIABLE(int, planningThreadPeriod, ptPeriod);
  DEFINE_VARIABLE(int, sbThreadPeriod, sbtPeriod);
  DEFINE_VARIABLE(int, visionThreadPeriod, vtPeriod);
  DEFINE_VARIABLE(int, localizationThreadPeriod, ltPeriod);
  DEFINE_VARIABLE(int, commThreadPeriod, commtPeriod);
  DEFINE_VARIABLE(int, heartBeat, 0);
  DEFINE_VARIABLE(
    vector<float>,
    jointPositionSensors,
    vector<float>(NUM_JOINTS));
  DEFINE_VARIABLE(
    vector<float>,
    jointStiffnessSensors,
    vector<float>(NUM_JOINTS));
  DEFINE_VARIABLE(
    vector<float>,
    jointTemperatureSensors,
    vector<float>(NUM_JOINTS));
  DEFINE_VARIABLE(
    vector<float>,
    jointCurrentSensors,
    vector<float>(NUM_JOINTS));
  DEFINE_VARIABLE(
    vector<float>,
    touchSensors,
    vector<float>(NUM_TOUCH_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    switchSensors,
    vector<float>(NUM_SWITCH_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    batterySensors,
    vector<float>(NUM_BATTERY_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    inertialSensors,
    vector<float>(NUM_INERTIAL_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    sonarSensors,
    vector<float>(NUM_SONAR_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    fsrSensors,
    vector<float>(NUM_FSR_SENSORS));
  DEFINE_VARIABLE(
    vector<float>,
    ledSensors,
    vector<float>(NUM_LED_ACTUATORS));
  //DECLARE_VARIABLE(vector<float>, imuDataFilterOutput, vector<float>(1));
  float ballRadius;
  GET_CONFIG(
    "EnvProperties",
    (float, ballRadius, ballRadius),
  )
  DEFINE_VARIABLE(BallInfo, ballInfo, BallInfo(TOP_CAM, ballRadius));
  DEFINE_VARIABLE(TeamBallInfo, teamBallInfo, TeamBallInfo());
  DEFINE_VARIABLE(GoalInfo, goalInfo, GoalInfo());
  DEFINE_VARIABLE(Point2f, kickTarget, Point2f(0, 0));
  DEFINE_VARIABLE(Matrix4f, upperCamInFeet, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, lowerCamInFeet, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, lFootOnGround, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, rFootOnGround, Matrix4f::Identity());
  DEFINE_VARIABLE(ObsObstacles, obstaclesObs, ObsObstacles());
  DEFINE_VARIABLE(ObsObstacles, obstaclesComm, ObsObstacles());
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    robotPose2D,
    RobotPose2D<float>(0.0, 0.0, 0.0));
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    lastKnownPose2D,
    RobotPose2D<float>(-1e3, -1e3, -1e3));
  OccupancyMap defMap;
  float fW, fH;
  GET_CONFIG(
    "PathPlanner",
    (float, Map.cellSize, defMap.resolution), (float, Map.fieldWidth, fW), (float, Map.fieldHeight, fH), )

  defMap.data = Mat(
    Size(fW / defMap.resolution, fH / defMap.resolution),
    CV_8UC1,
    Scalar(0));
  defMap.originPose = Point3f(
    fW / 2 / defMap.resolution,
    fH / 2 / defMap.resolution,
    0.0);
  DEFINE_VARIABLE(float, fieldWidth, fW);
  DEFINE_VARIABLE(float, fieldHeight, fH);
  DEFINE_VARIABLE(OccupancyMap, occupancyMap, defMap);
  DEFINE_VARIABLE(StiffnessState, stiffnessState, StiffnessState::UNKNOWN);
  DEFINE_VARIABLE(PostureState, postureState, PostureState::UNKNOWN);
  DEFINE_VARIABLE(PlanningState, planningState, PlanningState::UNKNOWN);
  DEFINE_VARIABLE(bool, robotFallen, false);
  DEFINE_VARIABLE(bool, robotInMotion, false);
  DEFINE_VARIABLE(bool, robotLocalized, false);
  DEFINE_VARIABLE(int, positionConfidence, 0);
  DEFINE_VARIABLE(int, sideConfidence, 0);
  DEFINE_VARIABLE(bool, robotOnSideLine, false);
  DEFINE_VARIABLE(bool, localizeWithLastKnown, false);
  DEFINE_VARIABLE(bool, landmarksFound, false);
  DEFINE_VARIABLE(int, robocupRole, -1); // None defined
  DEFINE_VARIABLE(int, robotIntention, 0);
  DEFINE_VARIABLE(int, nFootsteps, 0);
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    moveTarget,
    RobotPose2D<float>(0.0, 0.0, 0.0));
  DECLARE_VARIABLE(RoboCupGameControlData, gameData);
  int pNumber;
  int tNumber;
  int tPort;
  int tColor;
  GET_CONFIG(
    "TeamInfo",
    (int, playerNumber, pNumber), (int, teamNumber, tNumber), (int, teamPort, tPort), (int, teamColor, tColor), )
  DEFINE_VARIABLE(int, playerNumber, pNumber);
  DEFINE_VARIABLE(int, teamNumber, tNumber);
  DEFINE_VARIABLE(int, teamPort, tPort);
  DEFINE_VARIABLE(int, teamColor, tColor);
  DEFINE_VARIABLE(
    vector<TeamRobot>,
    teamRobots,
    vector<TeamRobot>(SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS));
  DEFINE_VARIABLE(bool, whistleDetected, false);
  DEFINE_VARIABLE(vector<RotatedRect>, footRects, vector<RotatedRect>());
  DEFINE_VARIABLE(BehaviorInfo, sBehaviorInfo, BehaviorInfo());
  DEFINE_VARIABLE(BehaviorInfo, mBehaviorInfo, BehaviorInfo());
  DEFINE_VARIABLE(BehaviorInfo, pBehaviorInfo, BehaviorInfo());
  DEFINE_VARIABLE(vector<ClientInfo>, clientsInfo, vector<ClientInfo>());
}

void
SharedMemory::getString(string& out)
{
  if (!variables.empty()) {
    size_t size = variables.size();
    size_t commaLimit = size - 1;
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(size);
    out += ':';
    for (size_t i = 0; i < variables.size(); ++i) {
      variables[i]->getString(out);
      if (i != commaLimit) out += ',';
    }
    DataUtils::getStringEndArray(out);
  } else {
    DataUtils::getStringStartArray(out);
    DataUtils::getStringEndArray(out);
  }
}

void
SharedMemory::getStringHeader(string& out)
{
  if (!variables.empty()) {
    size_t size = variables.size();
    size_t commaLimit = size - 1;
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(size);
    out += ':';
    for (size_t i = 0; i < variables.size(); ++i) {
      out += variables[i]->getVariableName();
      if (i != commaLimit) out += ',';
    }
    DataUtils::getStringEndArray(out);
  } else {
    DataUtils::getStringStartArray(out);
    DataUtils::getStringEndArray(out);
  }
}

ThreadSafeVariable*
SharedMemory::findVariableFromName(const string& name)
{
  for (size_t i = 0; i < variables.size(); ++i) {
    if (variables[i]->getVariableName() == name) return variables[i];
  }
  return NULL;
}
