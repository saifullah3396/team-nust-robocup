/**
 * @file PlanningModule/include/PlanningModule.h
 *
 * This file declares the class PlanningModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "PlanningModule/include/PBManager.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/BallInfo.h"
#include "BehaviorManager/include/BehaviorInfo.h"
#include "CommModule/include/ClientInfo.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/RoboCupGameControlData.h"
#include "Utils/include/StiffnessState.h"
#include "Utils/include/TeamRobot.h"
#include "Utils/include/TeamBallInfo.h"

/**
 * @class PlanningModule
 * @brief The class for behavior planning. All the functions and
 *   algorithms for adding intelligence to the robot are defined
 *   under this module.
 */
class PlanningModule : public BaseModule
{
  /**
   * Definition of input connector and variables for this module
   */ 
  CREATE_INPUT_CONNECTOR(PlanningInput,
    (int, planningThreadPeriod),
    (RoboCupGameControlData, gameData),
    (bool, landmarksFound),
    (unsigned, stiffnessState),
    (unsigned, postureState),
    (RobotPose2D<float>, robotPose2D),
    (BallInfo, ballInfo),
    (bool, robotLocalized),
    (vector<float>, jointPositionSensors),
    (vector<float>, jointStiffnessSensors),
    (vector<float>, jointTemperatureSensors),
    (vector<float>, jointCurrentSensors),
    (vector<float>, touchSensors),
    (vector<float>, switchSensors),
    (vector<float>, batterySensors),
    (vector<float>, inertialSensors),
    (vector<float>, sonarSensors),
    (vector<float>, fsrSensors),
    (vector<float>, ledSensors),
    (bool, whistleDetected),
    (bool, robotFallen),
    (int, playerNumber),
    (int, teamNumber),
    (GoalInfo, goalInfo),
    (vector<TeamRobot>, teamRobots),
    (TeamBallInfo, teamBallInfo),
    (ObsObstacles, obstaclesObs),
    (BehaviorInfo, sBehaviorInfo),
    (BehaviorInfo, mBehaviorInfo),
    (vector<ClientInfo>, clientsInfo),
  )
  /**
   * Definition of output connector and variables for this module
   */ 
  CREATE_OUTPUT_CONNECTOR(PlanningOutput,
    (int, controlThreadPeriod),
    (unsigned, planningState),
    (int, robocupRole),
    (int, robotIntention),
    (bool, robotOnSideLine),
    (bool, localizeWithLastKnown),
    (BehaviorInfo, pBehaviorInfo),
  )
public:
  /**
   * Constructor
   *
   * @param parent: Pointer to parent
   */
  PlanningModule(void* parent);

  /**
   * Destructor
   */
  ~PlanningModule()
  {
    delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();
private:
  //! Planning behaviors manager
  PBManagerPtr pbManager;
};
