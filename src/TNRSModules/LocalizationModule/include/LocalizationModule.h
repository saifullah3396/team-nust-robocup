/**
 * @file LocalizationModule/include/LocalizationModule.h
 *
 * This file declares a class for robot localization and mapping.
 * All the functions and algorithms for robot state estimation and
 * pose determination will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 Feb 2017
 */

#pragma once

#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/ConfigMacros.h"
#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "LocalizationModule/include/LocalizerStates.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "Utils/include/BallInfo.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/OccupancyMap.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/RoboCupGameControlData.h"

class FieldMap;
class ParticleFilter;

/**
 * @class LocalizationModule
 * @brief The class for robot localization and mapping.
 */
class LocalizationModule : public BaseModule
{
  /**
   * Input connector variables for this module
   */ 
  CREATE_INPUT_CONNECTOR(LocalizationInput,
    (int, localizationThreadPeriod),
    (float, fieldWidth),
    (float, fieldHeight),
    (RoboCupGameControlData, gameData),
    (Matrix4f, upperCamInFeet),
    (Matrix4f, lowerCamInFeet),
    //(BehaviorAccepted, lastPBehaviorAccepted),
    (unsigned, planningState),
    (ObsObstacles, obstaclesObs),
    (GoalInfo, goalInfo),
    (BallInfo, ballInfo),
    (vector<RotatedRect>, footRects),
    (bool, robotOnSideLine),
    (bool, localizeWithLastKnown),
    (bool, robotInMotion),
  )
  
  /**
   * Output connector variables for this module
   */ 
  CREATE_OUTPUT_CONNECTOR(LocalizationOutput,
    (RobotPose2D<float>, robotPose2D),
    (RobotPose2D<float>, lastKnownPose2D),
    (OccupancyMap, occupancyMap),
    (bool, robotLocalized),
    (int, positionConfidence),
    (int, sideConfidence),
  )
public:
  /**
   * Initializes the vision module with its thread.
   *
   * @param processingModule: Pointer to base class which is
   *   in this case the ProcessingModule.
   */
  LocalizationModule(void* processingModule);

  /**
   * Defualt destructor for this class.
   */
  ~LocalizationModule()
  {
    delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void
  init();

  /**
   * Derived from BaseModule
   */
  void
  mainRoutine();

  /**
   * Derived from BaseModule
   */
  void
  handleRequests();

  /**
   * Derived from BaseModule
   */
  void
  initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void
  setThreadPeriod();

  boost::shared_ptr<ParticleFilter>
  getParticleFilter()
  {
    return particleFilter;
  }

private:
  bool runLocalization;

  boost::shared_ptr<ParticleFilter> particleFilter;
  boost::shared_ptr<FieldMap> fieldMap;
};
