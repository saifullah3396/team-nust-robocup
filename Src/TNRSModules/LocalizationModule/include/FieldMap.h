/**
 * @file LocalizationModule/include/FieldMap.h
 *
 * The class for generating the map for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 28 Sep 2017
 */

#pragma once

#include "TNRSBase/include/BaseIncludes.h"
#include "TNRSBase/include/DebugBase.h"
#include "Utils/include/ConfigMacros.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "LocalizationModule/include/LocalizationModule.h"

/**
 * @class FieldMap
 * @brief The class for generating the map for robot localization.
 */
class FieldMap : public MemoryBase, DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to enable any kind of debugging.
  (int, debug, 0),
  //! Option to send particles to the debugger.
  (int, sendParticles, 0),
  //! Option to send the obstacles map to the debugger.
  (int, sendObstacleMap, 0),
)

public:

  /**
   * Default constructor for this class.
   */
  FieldMap(LocalizationModule* localizationModule) :
    MemoryBase(localizationModule), 
    DebugBase("FieldMap", this), prevObsId(0), addBallObstacle(false)
  {
    initDebugBase();
    int tempDebug;
    int tempSendParticles;
    int tempSendObstacleMap;
    GET_CONFIG(
      "LocalizationDebug",
      (int, FieldMap.debug, tempDebug), (int, FieldMap.sendParticles, tempSendParticles), (int, FieldMap.sendObstacleMap, tempSendObstacleMap), )
    SET_DVAR(int, debug, tempDebug);
    SET_DVAR(int, sendParticles, tempSendParticles);
    SET_DVAR(int, sendObstacleMap, tempSendObstacleMap);
    mapResolution =
      OVAR(OccupancyMap, LocalizationModule::occupancyMap).resolution;
    originPose =
      OVAR(OccupancyMap, LocalizationModule::occupancyMap).originPose;
    fieldWidth = IVAR(float, LocalizationModule::fieldWidth);
    fieldHeight = IVAR(float, LocalizationModule::fieldHeight);
    mapWidth = fieldWidth / mapResolution;
    mapHeight = fieldHeight / mapResolution;

    particleMap = Mat(Size(mapWidth, mapHeight), CV_8UC3);
    mapDrawing = Mat(Size(mapWidth, mapHeight), CV_8UC3);
    drawField();

    OVAR(OccupancyMap, LocalizationModule::occupancyMap).data = Scalar(0);
    rectangle(
      OVAR(OccupancyMap, LocalizationModule::occupancyMap).data,
      Point(0.25 / mapResolution, 0.25 / mapResolution),
      Point(
        (fieldWidth - 0.25) / mapResolution,
        (fieldHeight - 0.25) / mapResolution),
      Scalar(255),
      -1);
    occupancyMap =
      OVAR(OccupancyMap, LocalizationModule::occupancyMap).data.clone();
    pF = localizationModule->getParticleFilter();
  }

  /**
   * Default destructor for this class.
   */
  ~FieldMap()
  {
  }

  void
  update();

  void
  updateParticleMap();
  void
  updateRobotPose2D();
  void
  updateOccupancyMap();
  void
  drawField();
  void
  drawRobot();

  void setBallObstacle(const bool& addBallObstacle) 
    { this->addBallObstacle = addBallObstacle; }

private:
  int mapWidth;
  int mapHeight;
  float fieldWidth;
  float fieldHeight;
  Mat particleMap;
  Mat mapDrawing;
  float mapResolution;
  Point3f originPose;
  Mat occupancyMap;

  bool addBallObstacle;

  RobotPose2D<float> rState;
  boost::shared_ptr<ParticleFilter> pF;

  int prevObsId;
};
