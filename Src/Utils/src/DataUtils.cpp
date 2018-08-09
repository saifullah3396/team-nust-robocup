/**
 * @file src/DataUtils.cpp
 *
 * This file implements the class DataUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "BehaviorManager/include/BehaviorInfo.h"
#include "BallInfo.h"
#include "Camera.h" 
#include "DataUtils.h"
#include "RobotStateDefinitions.h"
#include "GoalInfo.h"
#include "Obstacle.h"
#include "ObstacleType.h"
#include "OccupancyMap.h"
#include "PostureState.h"
#include "PlanningState.h"
#include "RoboCupGameControlData.h"
#include "StiffnessState.h"
#include "TeamBallInfo.h"
#include "TeamRobot.h"

namespace Utils
{

  void
  DataUtils::getString(string& out, const BallInfo& ballInfo)
  {
    getStringStartArray(out);
    getString(out, ballInfo.camera);
    out += ',';
    getString(out, ballInfo.found);
    out += ',';
    getString(out, ballInfo.posRel.x);
    out += ',';
    getString(out, ballInfo.posRel.y);
    out += ',';
    getString(out, ballInfo.velRel.x);
    out += ',';
    getString(out, ballInfo.velRel.y);
    out += ',';
    getString(out, ballInfo.posImage.x);
    out += ',';
    getString(out, ballInfo.posImage.y);
    out += ',';
    getString(out, ballInfo.ballAge);
    getStringEndArray(out);
  }
    
  void
  DataUtils::getString(string& out, const BehaviorInfo& behaviorInfo)
  {
    out += "BehaviorInfo";
  }  
    
  void
  DataUtils::getString(string& out, const Camera& camera)
  {
    getString(out, camera.name);
    out += ',';
    getString(out, camera.clientName);
    out += ',';
    getString(out, camera.width);
    out += ',';
    getString(out, camera.height);
    out += ',';
    getString(out, camera.fps);
    out += ',';
    getString(out, camera.fovX);
    out += ',';
    getString(out, camera.fovY);
    out += ',';
    getString(out, camera.focalX);
    out += ',';
    getString(out, camera.focalY);
    out += ',';
    getString(out, camera.centerOffX);
    out += ',';
    getString(out, camera.centerOffY);
  }

  void
  DataUtils::getString(string& out, const GoalInfo& goalInfo)
  {
    getStringStartArray(out);
    getString(out, goalInfo.found);
    out += ',';
    getString(out, goalInfo.leftPost.x);
    out += ',';
    getString(out, goalInfo.leftPost.y);
    out += ',';
    getString(out, goalInfo.rightPost.x);
    out += ',';
    getString(out, goalInfo.rightPost.y);
    out += ',';
    getString(out, goalInfo.ours);
    getStringEndArray(out);
  }

  void
  DataUtils::getString(string& out, const Obstacle& obstacle)
  {
    getString(out, (unsigned) obstacle.type);
    out += ',';
    getString(out, (float) obstacle.center.x);
    out += ',';
    getString(out, (float) obstacle.center.y);
    out += ',';
    getString(out, (float) obstacle.leftBound.x);
    out += ',';
    getString(out, (float) obstacle.leftBound.y);
    out += ',';
    getString(out, (float) obstacle.rightBound.x);
    out += ',';
    getString(out, (float) obstacle.rightBound.y);
  }

  void
  DataUtils::getString(string& out, const ObsObstacles& obsObstacles)
  {
    getStringStartArray(out);
    for (size_t i = 0; i < obsObstacles.data.size(); ++i) {
      getString(out, obsObstacles.data[i]);
      out += ',';
    }
    getStringEndArray(out);
  }

  void
  DataUtils::getString(string& out, const OccupancyMap& occupancyMap)
  {
    out += "OccupancyMap";
  }

  void
  DataUtils::getString(string& out, const RoboCupGameControlData& data)
  {
    out += "RoboCupGameControlData";
  }

  void
  DataUtils::getString(string& out, const TeamBallInfo& teamBallInfo)
  {
    getStringStartArray(out);
    getString(out, teamBallInfo.found);
    out += ',';
    getString(out, teamBallInfo.posWorld.x);
    out += ',';
    getString(out, teamBallInfo.posWorld.y);
    out += ',';
    getString(out, teamBallInfo.velWorld.x);
    out += ',';
    getString(out, teamBallInfo.velWorld.y);
    getStringEndArray(out);
  }

  void
  DataUtils::getString(string& out, const TeamRobot& teamRobot)
  {
    getString(out, (bool)teamRobot.dataReceived);
    out += ',';
    getString(out, (int)teamRobot.fallen);
    out += ',';
    getString(out, (int)teamRobot.intention);
    out += ',';
    getString(out, (int)teamRobot.suggestionToMe);
    out += ',';
    getString(out, teamRobot.pose.x);
    out += ',';
    getString(out, teamRobot.pose.y);
    out += ',';
    getString(out, teamRobot.pose.theta);
    out += ',';
    getString(out, teamRobot.walkingTo.x);
    out += ',';
    getString(out, teamRobot.walkingTo.y);
    out += ',';
    getString(out, teamRobot.shootingTo.x);
    out += ',';
    getString(out, teamRobot.shootingTo.y);
  }

  void
  DataUtils::getString(string& out, const Matrix<float, 2, 1, 0, 2, 1>& vec)
  {
    getStringStartArray(out);
    getString(out, vec[0]);
    out += ',';
    getString(out, vec[1]);
    getStringEndArray(out);
  }

  void
  DataUtils::getString(string& out, const RotatedRect& rRect)
  {
    out += "Rect";
  }
}
