/**
 * @file LocalizationModule/src/FieldMap.cpp
 *
 * The class for generating the map for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 28 Sep 2017
 */

#include "LocalizationModule/include/FieldMap.h"
#include "Utils/include/VisionUtils.h"

using namespace Utils;

void
FieldMap::update()
{
  updateRobotPose2D();
  updateOccupancyMap();
  //drawRobot();
  //#ifdef DEBUG_BUILD
  updateParticleMap();
  //#endif
}

void
FieldMap::updateRobotPose2D()
{
  //cout << "avgState: " << endl;
  OVAR(RobotPose2D<float>, LocalizationModule::robotPose2D) =
    pF->getFilteredState();
  //cout << "xavg: " << OVAR(RobotPose2D<float>, LocalizationModule::robotPose2D).x << endl;
  //cout << "yavg: " << OVAR(RobotPose2D<float>, LocalizationModule::robotPose2D).y << endl;
  //cout << "theatavg: " << OVAR(RobotPose2D<float>, LocalizationModule::robotPose2D).theta << endl;
}

void
FieldMap::updateOccupancyMap()
{
  if (!pF->getLocalized()) return;

  occupancyMap = Scalar(0);
  rectangle(
    occupancyMap,
    Point(0.25 / mapResolution, 0.25 / mapResolution),
    Point(
      (fieldWidth - 0.25) / mapResolution,
      (fieldHeight - 0.25) / mapResolution),
    Scalar(255),
    -1);
  //VisionUtils::displayImage(map, "occupancyMapStart");
  /*vector<Obstacle> obstacles;
   obstacles.push_back(Obstacle(OBS_GOAL_POST, Point2f(3.0, 0.35), Point2f(3.0, 0.4), Point2f(3.0, 0.3)));
   obstacles.push_back(Obstacle(OBS_GOAL_POST, Point2f(3.0, 0.35), Point2f(3.0, -0.3), Point2f(3.0, -0.4)));*/
  auto& obstaclesObs = IVAR(ObsObstacles, LocalizationModule::obstaclesObs);
  if (obstaclesObs.id != 0 && obstaclesObs.id != prevObsId && !obstaclesObs.data.empty()) {
    auto& obstacles = obstaclesObs.data;
    RobotPose2D<float> robotPose = pF->getFilteredState();
    float cTheta = cos(robotPose.theta);
    float sTheta = sin(robotPose.theta);
    for (int i = 0; i < obstacles.size(); ++i) {
      Obstacle obs = obstacles[i];
      float tempLX =
        robotPose.x + obs.leftBound.x * cTheta - obs.leftBound.y * sTheta;
      float tempLY =
        robotPose.y + obs.leftBound.x * sTheta + obs.leftBound.y * cTheta;
      float tempRX =
        robotPose.x + obs.rightBound.x * cTheta - obs.rightBound.y * sTheta;
      float tempRY =
        robotPose.y + obs.rightBound.x * sTheta + obs.rightBound.y * cTheta;
      obs.leftBound.x = tempLX;
      obs.leftBound.y = tempLY;
      obs.rightBound.x = tempRX;
      obs.rightBound.y = tempRY;
      float obsDepth = 0.0;
      if (obs.type == ObstacleType::GOALPOST) {
        obsDepth = 0.1; //Diameter of a goalpost.
      } else if (obs.type == ObstacleType::OPPONENT || obs.type == ObstacleType::TEAMMATE) {
        obsDepth = 0.2; //Depth of the robot.
      } else if (obs.type == ObstacleType::OPPONENT_FALLEN || obs.type == ObstacleType::TEAMMATE_FALLEN) {
        obsDepth = 0.2; //Depth of the robot.
      }
      float obsSlope =
        (obs.rightBound.y - obs.leftBound.y) / (obs.rightBound.x - obs.leftBound.x);
      float obsSlopeInv = -1 / obsSlope;
      Point2f tempPoint(
        obs.leftBound.x + 0.1,
        0.1 * obsSlopeInv + obs.leftBound.y);
      Point2f diff = tempPoint - obs.leftBound;
      float mag = norm(diff);
      Point2f unit = Point2f(diff.x / mag, diff.y / mag);
      Point rectP1 = Point(
        originPose.x + (obs.leftBound.x + unit.x * obsDepth) / mapResolution,
        originPose.y - (obs.leftBound.y + unit.y * obsDepth) / mapResolution);
      Point rectP2 = Point(
        originPose.x + (obs.rightBound.x + unit.x * obsDepth) / mapResolution,
        originPose.y - (obs.rightBound.y + unit.y * obsDepth) / mapResolution);
      vector < Point > obstaclePoly(2);
      obstaclePoly =
      {
        Point(originPose.x + obs.leftBound.x / mapResolution,
          originPose.y - obs.leftBound.y / mapResolution),
        rectP1,
        rectP2,
        Point(originPose.x + obs.rightBound.x / mapResolution,
          originPose.y - obs.rightBound.y / mapResolution)
      };
      fillConvexPoly(
        occupancyMap,
        &obstaclePoly[0],
        (int) obstaclePoly.size(),
        0,
        8,
        0);
      //VisionUtils::drawPoints(obstaclePoly, map);
    }
  }
  prevObsId = obstaclesObs.id;

  if (addBallObstacle) {
    if (IVAR(BallInfo, LocalizationModule::ballInfo).found) {
      RobotPose2D<float> rP = pF->getFilteredState();
      Point2f posRel = IVAR(BallInfo, LocalizationModule::ballInfo).posRel;
      auto ct = cos(rP.theta);
      auto st = sin(rP.theta);
      Point2f ballWorld;
      ballWorld.x = rP.x + posRel.x * ct - posRel.y * st;
      ballWorld.y = rP.y + posRel.x * st + posRel.y * ct;
      circle(
        occupancyMap,
        Point(
          originPose.x + ballWorld.x / mapResolution,
          originPose.y - ballWorld.y / mapResolution),
        0.1 / mapResolution,
        Scalar(0, 0, 0));
    }
  }

  OVAR(OccupancyMap, LocalizationModule::occupancyMap).data =
    occupancyMap.clone();
  //VisionUtils::displayImage(occupancyMap, "occupancyMapUpdated");
}

void
FieldMap::updateParticleMap()
{

  if (GET_DVAR(int, debug)) {
    if (GET_DVAR(int, sendParticles)) {
      vector<Particle> particles = pF->getParticles();
      if (particles.size() > 0) {
        Mat_<float> particleData(particles.size(), 4);
        for (size_t i = 0; i < particles.size(); ++i) {
          particleData(i, 0) = particles[i].state.x;
          particleData(i, 1) = particles[i].state.y;
          particleData(i, 2) = particles[i].state.theta;
          particleData(i, 3) = particles[i].weight;
        }
        const unsigned char* ptr = particleData.data;
        int count = particles.size() * 16;
        string msg = DataUtils::bytesToHexString(ptr, count);
        // Change now to Comm message request
        //CommModule::addToSpecialMsgQueue(pair<int, string>(MSG_PF_STATES, msg));
      }
    }
  }
  particleMap = mapDrawing.clone();
  vector<Particle> particles = pF->getParticles();

  for (unsigned i = 0; i < particles.size(); ++i) {
    //cout << "particle[" << i << "]: " << "[" << particles[i].state.x << "," << particles[i].state.y << "," <<  particles[i].state.theta << endl;
    circle(
      particleMap,
      Point(
        mapWidth / 2 + particles[i].state.x * 100,
        mapHeight / 2 - particles[i].state.y * 100),
      10,
      Scalar(0, 0, 0));
    line(
      particleMap,
      Point(
        mapWidth / 2 + particles[i].state.x * 100,
        mapHeight / 2 - particles[i].state.y * 100),
      Point(
        mapWidth / 2 + particles[i].state.x * 100 + 15 * cos(
          particles[i].state.theta),
        mapHeight / 2 - particles[i].state.y * 100 + -15 * sin(
          particles[i].state.theta)),
      Scalar(255, 255, 255));
  }
  auto footRects = IVAR(vector<RotatedRect>, LocalizationModule::footRects);
  for (int i = 0; i < footRects.size(); ++i) {
    Point2f vertices[4];
    footRects[i].points(vertices);
    for (int j = 0; j < 4; j++) {
      Scalar color;
      if (i == 0) color = Scalar(255, 0, 0);
      else if (i == footRects.size() - 1) color = Scalar(0, 255, 0);
      else color = Scalar(255, 255, 255);
      line(particleMap, vertices[j], vertices[(j + 1) % 4], color);
    }
  }

  RobotPose2D<float> rP = pF->getFilteredState();
  if (IVAR(BallInfo, LocalizationModule::ballInfo).found) {
    Point2f ballPos = IVAR(BallInfo, LocalizationModule::ballInfo).posRel;
    auto ct = cos(rP.theta);
    auto st = sin(rP.theta);
    Point2f ballWorld;
    ballWorld.x = rP.x + ballPos.x * ct - ballPos.y * st;
    ballWorld.y = rP.y + ballPos.x * st + ballPos.y * ct;
    circle(
      particleMap,
      Point(
        mapWidth / 2 + ballWorld.x * 100,
        mapHeight / 2 - ballWorld.y * 100),
      5,
      Scalar(0, 0, 0));
  }

  auto& lastknown =
    OVAR(RobotPose2D<float>, LocalizationModule::lastKnownPose2D);
  if (lastknown.x != 1e3) {
    circle(
      particleMap,
      Point(
        mapWidth / 2 + lastknown.x * 100,
        mapHeight / 2 - lastknown.y * 100),
      10,
      Scalar(0, 0, 255));
    line(
      particleMap,
      Point(
        mapWidth / 2 + lastknown.x * 100,
        mapHeight / 2 - lastknown.y * 100),
      Point(
        mapWidth / 2 + lastknown.x * 100 + 15 * cos(lastknown.theta),
        mapHeight / 2 - lastknown.y * 100 + -15 * sin(lastknown.theta)),
      Scalar(0, 0, 255));
  }
  //cout << "ParticleMap: " << endl;
  VisionUtils::displayImage(particleMap, "Particle Map");
  //if (pF->getLocalized())
  //waitKey(0);
  //cout << "ParticleMap1: " << endl;
}

void
FieldMap::drawField()
{
  int width = mapWidth;
  int height = mapHeight;
  mapDrawing = Scalar(0, 100, 0); //BGR
  int borderDiff = 50;
  rectangle(
    mapDrawing,
    Point(borderDiff, borderDiff),
    Point(width - borderDiff, height - borderDiff),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff, height / 2 - 220 / 2),
    Point(borderDiff + 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(width - borderDiff, height / 2 - 220 / 2),
    Point(width - borderDiff - 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 6, height / 2 + 3),
    Point(borderDiff + 130 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 3, height / 2 + 6),
    Point(borderDiff + 130 + 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 6, height / 2 + 3),
    Point(width - borderDiff - 130 - 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 3, height / 2 + 6),
    Point(width - borderDiff - 130 - 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width / 2 - 6, height / 2 + 3),
    Point(width / 2 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  line(
    mapDrawing,
    Point(width / 2, 5),
    Point(width / 2, height - 5),
    Scalar(255, 255, 255),
    5);
  circle(
    mapDrawing,
    Point(width / 2, height / 2),
    75,
    Scalar(255, 255, 255),
    5);
}

void
FieldMap::drawRobot()
{
  RobotPose2D<float> rState = pF->getFilteredState();
  circle(mapDrawing, Point(rState.x, rState.y), 10, Scalar(255, 0, 0));
  line(
    mapDrawing,
    Point(rState.x, rState.y),
    Point(rState.x + 15 * cos(rState.theta), rState.y + 15 * sin(rState.theta)),
    Scalar(0, 0, 0));
}
