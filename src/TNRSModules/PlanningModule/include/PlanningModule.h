/**
 * @file PlanningModule/include/PlanningModule.h
 *
 * This file declares the class PlanningModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <alproxies/almemoryproxy.h>
#include "PlanningModule/include/PBManager.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/BallInfo.h"
#include "BehaviorManager/include/BehaviorInfo.h"
#include "ControlModule/include/HardwareLayer.h"
#include "CommModule/include/ClientInfo.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/RoboCupGameControlData.h"
#include "Utils/include/StiffnessState.h"
#include "Utils/include/TeamRobot.h"
#include "Utils/include/TeamBallInfo.h"

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;

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
    (bool, landmarksFound),
    (unsigned, stiffnessState),
    (unsigned, postureState),
    (RobotPose2D<float>, robotPose2D),
    (BallInfo, ballInfo),
    (bool, robotLocalized),
    (vector<float>, jointPositionSensors),
    (vector<float>, jointStiffnessSensors),
    (vector<float>, inertialSensors),
    (vector<float>, fsrSensors),
    (vector<float>, ledSensors),
    (bool, whistleDetected),
    (bool, robotFallen),
    (int, playerNumber),
    (int, teamNumber),
    (int, teamColor),
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
    (unsigned, planningState),
    (int, robocupRole),
    (int, robotIntention),
    (bool, robotOnSideLine),
    (bool, localizeWithLastKnown),
    (BehaviorInfo, pBehaviorInfo),                  
    (vector<float>, jointTemperatureSensors),
    (vector<float>, jointCurrentSensors),
    (vector<float>, touchSensors),
    (vector<float>, switchSensors),
    (vector<float>, batterySensors),
    (vector<float>, sonarSensors),
    (RoboCupGameControlData, gameData),
  )
public:
  /**
   * Constructor
   *
   * @param parent: Pointer to parent
   */
  PlanningModule(void* parent, const ALMemoryProxyPtr& memoryProxy);

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
  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   */
  void sensorsUpdate();

  /**
   * Sets up the team configuration data to be inserted in ALMemory
   * for robocup game controller
   *
   * @return void
   */
  void
  setupRoboCupDataHandler();

  //! Planning behaviors manager
  PBManagerPtr pbManager;

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Pointer to NaoQi internal memory proxy
  ALMemoryProxyPtr memoryProxy;

  enum class PlanningSensors : unsigned {
    JOINT_TEMP,
    JOINT_CURRENT,
    TOUCH,
    SWITCH,
    BATTERY,
    COUNT
  };
};
