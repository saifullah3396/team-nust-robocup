/**
 * @file PlanningModule/PlanningBehaviors/GoalKeeper.h
 *
 * This file declares the class GoalKeeper.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "MotionModule/include/MotionConfigs/MBDiveConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/GoalKeeper.h"
#include "SBModule/include/SBConfigs.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"

void
GoalKeeper::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < GoalKeeperConfig > (behaviorConfig));
}

boost::shared_ptr<GoalKeeperConfig>
GoalKeeper::getBehaviorCast()
{
  return boost::static_pointer_cast < GoalKeeperConfig > (behaviorConfig);
}

void
GoalKeeper::initiate()
{
  LOG_INFO("GoalKeeper behavior initiated...")
  inBehavior = true;
}

void
GoalKeeper::update()
{
  //LOG_INFO("Executing GoalKeeper.update()...")
  // IMPORTANT BUG
  // If robot loses its position estimate and cant find itself using landmarks
  // It gets stuck in the loop to scan the env forever. fix this.
  //
  //LOG_INFO("In Robocup Gameplay Loop...")
  updatePostureAndStiffness();
  setRobotIntention();
  /*if(penalisedRobot()) {
   LOG_INFO("RobotPenalized...")
   OVAR(bool, PlanningModule::runVisionModule) = false;
   OVAR(bool, PlanningModule::runLocalizationModule) = false;
   OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   wasPenalised = true;
   return;
   } else {
   if (wasPenalised) {
   behaviorState = getInPositions;
   OVAR(bool, PlanningModule::runVisionModule) = true;
   OVAR(bool, PlanningModule::runLocalizationModule) = true;
   OVAR(bool, PlanningModule::robotOnSideLine) = true;
   wasPenalised = false;
   //OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   }
   }*/
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState()) return;

  if (
    (IVAR(bool, PlanningModule::robotFallen) || 
    posture == PostureState::FALLING_FRONT || 
    posture == PostureState::FALLING_BACK) && !fallRecovery) 
  {
    fallenRobotAction();
  }

  // If the robot is recovering from a fall 
  if (fallRecovery) {
    if (lastMotionRequest) {
      LOG_INFO("lastMotionRequest->id: " << lastMotionRequest->id)
      LOG_INFO("LastMBFinished: " << lastMBFinished)
    }
    fallRecoveryAction();
  } else {
    goalKeeperAction();
  }
}

void
GoalKeeper::finishBehaviorSafely()
{
  inBehavior = false;
}

void
GoalKeeper::goalKeeperAction()
{
  //LOG_INFO("Executing GoalKeeper.goalKeeperAction()...")
  //LOG_INFO("BehaviorState: " << behaviorState)
  if (behaviorState == correctPosture) {
    correctPostureAction();
  } else if (behaviorState == findItself) {
    findItselfAction(); // If robot finds itself
  } else if (behaviorState == correctPosition) {
    correctPositionAction();
  } else if (behaviorState == waitForMove) {
    waitForMoveAction();
  } else if (behaviorState == searchBall) {
    searchBallAction();
  } else if (behaviorState == saveGoal) {
    saveGoalAction();
  } else if (behaviorState == interceptBall) {
    interceptBallAction();
  } else if (behaviorState == dive) {
    diveAction();
  } else if (behaviorState == diveFollowUp) {
    diveFollowUpAction();
  } else if (behaviorState == playBall) {
    playBallAction();
  } else if (behaviorState == alignToKick) {
    alignToKickAction();
  } else if (behaviorState == kickBall) {
    kickBallAction();
  }
}

void
GoalKeeper::correctPostureAction()
{
  LOG_INFO("Executing GoalKeeper.correctPostureAction()...")
  if (posture == PostureState::STAND_WALK) {
    behaviorState = findItself;
  } else {
    if (lastMBFinished) {
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND_WALK, 2.f);
      setupMBRequest(pConfig);
    }
  }
}

bool
GoalKeeper::shouldLocalize()
{
  return !IVAR(bool, PlanningModule::robotLocalized);
}

void
GoalKeeper::findItselfAction()
{
  LOG_INFO("Exeucting GoalKeeper.findItselfAction()")
  if (lastMBStarted && lastMotionRequest->id == (unsigned) MBIds::MOVEMENT) { // Kill any currently running behavior other than finding landmarks
    lastMotionRequest->kill();
  }

  if (shouldLocalize()) {
    LOG_INFO("Robot not localized. Trying to scan and get localized...")
    OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
    if (lastMBFinished) {
      auto mConfig =
        boost::make_shared <HeadTargetSearchConfig> (
          HeadTargetTypes::LANDMARKS);
      // If robot is on Sidelines alone. Better to find T or L corners too. 
      if (OVAR(bool, PlanningModule::robotOnSideLine)) // After penalizeRecovery
      mConfig->scanMaxYaw = 100.0 * M_PI / 180;
      else // If robot is in the field.
      mConfig->scanMaxYaw = 90 * M_PI / 180;
      setupMBRequest(mConfig);
    }
  } else {
    behaviorState = correctPosition;
  }
}

void
GoalKeeper::correctPositionAction()
{
  LOG_INFO("Exeucting GoalKeeper.findItselfAction()")
  if (
    lastMBStarted && lastMotionRequest->id == 
    (unsigned) MBIds::HEAD_CONTROL) 
  { // Kill any head tracking behavior
    lastMotionRequest->kill();
  }
  auto& robotPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  if (robotPose.x >= -4.5 && robotPose.x <= -4.0 && robotPose.y >= -0.75 && robotPose.y <= 0.75) {
    // Correct position found
    behaviorState = searchBall;
  } else {
    RobotPose2D<float> target;
    target.x = -4.25;
    if (robotPose.y < -0.75) target.y = -0.65;
    else if (robotPose.y > 0.75) target.y = 0.65;
    else target.y = robotPose.y;
    target.theta = 0.f;
    if (lastMBFinished) {
      if (goToTarget(target, 0.1, 0.1, 0.174444444, false)) {
        behaviorState = searchBall;
      }
      behaviorState = waitForMove;
    }
  }
}

void
GoalKeeper::waitForMoveAction()
{
  if (lastMBFinished && lastMotionRequest->id == (unsigned) MBIds::MOVEMENT) {
    behaviorState = findItself;
  }
}

bool
GoalKeeper::shouldSearchBall()
{
  auto& tbInfo = IVAR(TeamBallInfo, PlanningModule::teamBallInfo);
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  auto& rPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  if (!tbInfo.found) {
    //cout << "taem ball not foudn " << endl;
    return true;
  } else {
    if (!bInfo.found) { //cout << "Ball not found by this robot." << endl;
      auto ballWorld = tbInfo.posWorld;
      if (MathsUtils::dist(ballWorld.x, ballWorld.y, rPose.x, rPose.y) <= 2.5f) {
        //cout << "ball not foudn and in range" << endl;
        return true;
      }
    }
  }
  return false;
}

void
GoalKeeper::searchBallAction()
{
  //LOG_INFO("Exeucting GoalKeeper.searchBallAction()")
  if (shouldSearchBall()) {
    if (lastMBFinished) {
      //cout << "Setting find ball" << endl;
      auto mConfig =
        boost::make_shared <HeadTargetSearchConfig> (
          HeadTargetTypes::BALL);
      setupMBRequest(mConfig);
    }
  } else {
    auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
    auto& tbInfo = IVAR(TeamBallInfo, PlanningModule::teamBallInfo);
    //cout << "velMag: " << tbInfo.velWorld << endl;
    //cout << "velRel: " << bInfo.velRel << endl;
    //cout << "final" << endl;
    behaviorState = saveGoal;
  }
}

void
GoalKeeper::saveGoalAction()
{
  LOG_INFO("Exeucting GoalKeeper.saveGoalAction()")
  auto& tbInfo = IVAR(TeamBallInfo, PlanningModule::teamBallInfo);
  auto ballWorld = tbInfo.posWorld;
  if (ballWorld.x >= -4.5 && ballWorld.x <= -3.75 && ballWorld.y >= -1.15 && ballWorld.y <= 1.15 && shouldPlayBall()) {
    behaviorState = playBall;
  } else {
    auto ballVel = tbInfo.velWorld;
    auto velMag = norm(ballVel);
    Point2f unitVel;
    unitVel.x = ballVel.x / velMag;
    unitVel.y = ballVel.y / velMag;
    // Find if the ball is going in goal
    // Make a goal line 
    Vec4f goalLine;
    goalLine[0] = -4.5;
    goalLine[1] = 0.75;
    goalLine[2] = -4.5;
    goalLine[3] = -0.75;

    Vec4f ballLine;
    ballLine[0] = ballWorld.x;
    ballLine[1] = ballWorld.y;
    ballLine[2] = ballWorld.x + unitVel.x * 100.f; // A possibly infinite line
    ballLine[3] = ballWorld.y + unitVel.y * 100.f; // A possibly infinite line
    auto inter = VisionUtils::findIntersection(goalLine, ballLine);
    if (inter.x >= -4.75 && inter.x <= -4.25 && inter.y >= -0.75 && inter.y <= 0.75) {
      cout << "ballVel: " << ballVel << endl;
      cout << "velMag: " << ballVel << endl;
      cout << "unitVel: " << unitVel << endl;
      cout << "inter: " << inter << endl;
      // Means ball is going towards the goal
      // Find the time ball will reach the line 25 cm in front of the goal line. 
      // Use s=vit + 1/2at^2
      float timeToReact, dist, decel;
      float rollingFriction = 0.022;
      decel = rollingFriction * 9.80665;
      dist = norm(inter - ballWorld);
      float a, b, c;
      a = -1 / 2 * decel;
      b = velMag;
      c = -dist;
      float sqrtC = b * b - 4 * a * c;
      float timeToStop = velMag / decel;
      if (sqrtC < 0) { // Ball will stop before reaching the goal
        LOG_INFO("Ball will stop before reaching the goal...")
        dist = b * timeToStop + a * timeToStop * timeToStop;
        Point2f stopPoint;
        stopPoint.x = ballWorld.x + unitVel.x * dist;
        stopPoint.y = ballWorld.y + unitVel.y * dist;
        if (stopPoint.x >= -4.25 && stopPoint.x <= -3.75 && stopPoint.y >= -0.75 && stopPoint.y <= 0.75) {
          lastInterceptTarget = RobotPose2D<float>(
            stopPoint.x,
            stopPoint.y,
            0.f);
          behaviorState = interceptBall;
        } else {
          lastInterceptTarget.x = -4.25;
          lastInterceptTarget.y = ballWorld.y * 0.7 / 3;
          lastInterceptTarget.theta = 0.f;
          behaviorState = interceptBall;
        }
      } else { // Ball won't stop before reaching the goal
        LOG_INFO("Ball won't stop before reaching the goal...")
        timeToReact = (-b + sqrt(sqrtC)) / (2 * a);
        if (timeToReact > timeToStop) {
          timeToReact = (-b - sqrt(sqrtC)) / (2 * a);
        }

        lastInterceptTarget.x = -4.25;
        lastInterceptTarget.y = inter.y;
        lastInterceptTarget.theta = 0.f;
        cout << "timeToReact: " << timeToReact << endl;
        if (timeToReact <= 3.f) {
          if (divePossible()) { // if ball will reach within 3 secs, try to dive.
            behaviorState = dive;
          } else {
            // if ball is too far for dive, move to intercept.
            behaviorState = interceptBall;
          }
        } else {
          behaviorState = interceptBall;
        }
      }
    } else { // If ball is not targeted towards the goal.
      LOG_INFO(
        "Intercept by just moving in front of the goal based on ball/field position ratio...")
      lastInterceptTarget.x = -4.25;
      // 0.7m meaning from mid to goal width with a padding of 5 cm 
      // since the goal is 0.75 meters wide on one side
      // The position is defined by the ratio of 0.7 / 3, where 3m is 
      // the length of half the field in y
      lastInterceptTarget.y = ballWorld.y * 0.7 / 3;
      lastInterceptTarget.theta = 0.f;
      behaviorState = interceptBall;
    }
  }
}

void
GoalKeeper::interceptBallAction()
{
  LOG_INFO("Exeucting GoalKeeper.interceptBallAction()")
  //cout << "lastInterceptTarget.x: " << lastInterceptTarget.x << endl;
  //cout << "lastInterceptTarget.y: " << lastInterceptTarget.y << endl;
  //cout << "lastInterceptTarget.theta: " << lastInterceptTarget.theta << endl;
  if (lastMBFinished) {
    goToTarget(lastInterceptTarget, 0.05, 0.05, 0.087222222, true);
    behaviorState = waitForMove;
  }
}

bool
GoalKeeper::divePossible()
{
  auto& rPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  auto ct = cos(rPose.theta);
  auto st = sin(rPose.theta);
  Point2f wInRobot;
  //wInRobot.x = -rPose.x * ct - rPose.y * st;
  wInRobot.y = rPose.x * st - rPose.y * ct;
  Point2f ssInRobot;
  //ssInRobot.x = wInRobot.x + saveSpot.x * ct + saveSpot.y * st;
  ssInRobot.y =
    wInRobot.y - lastInterceptTarget.x * st + lastInterceptTarget.y * ct;
  if (ssInRobot.y > 0) {
    if (ssInRobot.y <= 0.1) {
      lastDiveTargetType = (unsigned) KeyFrameDiveTypes::IN_PLACE;
      return true;
    } else if (ssInRobot.y > 0.1 && ssInRobot.y <= 0.2) {
      lastDiveTargetType = (unsigned) KeyFrameDiveTypes::LEFT;
      return true;
    }
  } else {
    if (ssInRobot.y > -0.1) {
      lastDiveTargetType = (unsigned) KeyFrameDiveTypes::IN_PLACE;
      return true;
    } else if (ssInRobot.y < -0.1 && ssInRobot.y <= -0.2) {
      lastDiveTargetType = (unsigned) KeyFrameDiveTypes::RIGHT;
      return true;
    }
  }
  return false;
}

void
GoalKeeper::diveAction()
{
  LOG_INFO("Exeucting GoalKeeper.diveAction()")
  if (lastDiveTargetType != -1) {
    if (lastMBFinished) {
      auto dConfig =
        boost::make_shared <KFMDiveConfig> ((KeyFrameDiveTypes) lastDiveTargetType);
      setupMBRequest(dConfig);
      behaviorState = diveFollowUp;
      lastDiveTargetType = -1;
    }
  }
}

void
GoalKeeper::diveFollowUpAction()
{
  //if(lastMBFinished &&
  //   lastMotionRequest->id == MB_DIVE) 
  //{
  LOG_INFO("Executing GoalKeeper.diveFollowUpAction()...")
  if (posture == PostureState::STAND) {
    behaviorState = correctPosture;
  } else {
    if (lastMBFinished) {
      LOG_INFO("Setting posture to stand at startup.")
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND, 2.f);
      setupMBRequest(pConfig);
    }
  }
  //}
}

bool
GoalKeeper::shouldPlayBall()
{
  auto& tbInfo = IVAR(TeamBallInfo, PlanningModule::teamBallInfo);
  auto ballWorld = tbInfo.posWorld;
  if (ballWorld.x < -4.5 && ballWorld.x > -3.75 && ballWorld.y < -1.15 && ballWorld.y > 1.15) {
    return false;
  }
  // Find if any other teammate is following ball right now and 
  // is also closer.
  auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posRel;
  auto ballToRobot = norm(ballRel);
  auto& teamRobots = IVAR(vector<TeamRobot>, PlanningModule::teamRobots);
  for (int i = 0; i < teamRobots.size(); ++i) {
    // If any robot is intending to play the ball
    if (teamRobots[i].intention == 3) {
      float dist = MathsUtils::dist(
        ballWorld.x,
        ballWorld.y,
        teamRobots[i].pose.x,
        teamRobots[i].pose.y);
      // If that robot is closer to ball than this robot
      if (dist < ballToRobot) {
        return false;
      }
    }
  }
  return true;
}

void
GoalKeeper::playBallAction()
{
  if (shouldPlayBall()) {
    OVAR(int, PlanningModule::robotIntention) = 3;
    // Send robot at distance 0.2 from the ball and angle 0 degrees.
    if (goToBall(0.25, 0.f)) {
      // Reached ball 
      if (findBallKickTarget()) {
        behaviorState = alignToKick;
      } else {
        behaviorState = correctPosture;
      }
    }
  } else {
    OVAR(int, PlanningModule::robotIntention) = 1;
    behaviorState = correctPosture;
  }
}

bool
GoalKeeper::goToBall(const float& distFromBall, const float& angle)
{
  OVAR(bool, PlanningModule::addBallObstacle) = true;
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  auto& tBInfo = IVAR(TeamBallInfo, PlanningModule::teamBallInfo);
  auto ballImage = bInfo.posImage;
  auto ballRel = bInfo.posRel;
  Size imageCenter;
  if (bInfo.camera == TOP_CAM) imageCenter = Size(320, 240);
  else imageCenter = Size(80, 60);

  if (abs(ballImage.x - imageCenter.width) < 100 && abs(
    ballImage.y - imageCenter.height) < 100) {
    if (
      lastMBStarted && lastMotionRequest->id == 
      (unsigned) MBIds::HEAD_CONTROL) 
    { // Kill any head tracking behavior
      lastMotionRequest->kill();
    }
    float dist = norm(ballRel);
    if (dist > 0.2) { // If ball is greater than 20 cm far 
      auto ballWorld = tBInfo.posWorld;
      RobotPose2D<float> target;
      target.x = ballWorld.x - distFromBall * cos(angle);
      target.y = ballWorld.y - distFromBall * sin(angle);
      target.theta = angle;
      if (goToTarget(target, 0.05, 0.05, 0.087222222, true)) {
        OVAR(bool, PlanningModule::addBallObstacle) = false;
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  } else {
    if (lastMBFinished) {
      auto mConfig =
        boost::make_shared <HeadTargetTrackConfig> (
          HeadTargetTypes::BALL);
      setupMBRequest(mConfig);
    }
    return false;
  }
}

bool
GoalKeeper::findBallKickTarget()
{
  // First try to pass the ball to robot far away.
  auto& teamRobots = IVAR(vector<TeamRobot>, PlanningModule::teamRobots);
  auto& rPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  Point2f bestPassTarget;
  float bestDist = 100;
  for (int i = 0; i < teamRobots.size(); ++i) {
    // Find defender robots
    if (teamRobots[i].intention == 2) {
      float dist = MathsUtils::dist(
        rPose.x,
        rPose.y,
        teamRobots[i].pose.x,
        teamRobots[i].pose.y);
      if (dist < 3.f) {
        bestDist = dist < bestDist ? dist : bestDist;
        if (!behindObstacle(bestPassTarget)) {
          bestPassTarget.x = teamRobots[i].pose.x;
          bestPassTarget.y = teamRobots[i].pose.y;
        }
      }
    }
  }

  if (bestDist < 100) {
    lastKickTarget = bestPassTarget;
    return true;
  } else {
    bool occupiedRange[180];
    auto& obstaclesObs = IVAR(ObsObstacles, PlanningModule::obstaclesObs);
    float angleMax = 75;
    float arcLengthDiff = 0.1; // 10 cm arcLength
    for (int i = 0; i < obstaclesObs.data.size(); ++i) {
      auto obs = obstaclesObs.data[i];
      auto v1 = Point2f(obs.leftBound.x, obs.leftBound.y);
      auto v2 = Point2f(obs.rightBound.x, obs.rightBound.y);
      int angle1 = atan2(v1.y, v1.x) * 180 / M_PI + 90;
      int angle2 = atan2(v2.y, v2.x) * 180 / M_PI + 90;
      // Angle 1 always greater than angle 2.
      for (int j = angle2; j < angle1; ++j) {
        occupiedRange[j] = true;
      }
    }
    int bestLeft = 1000;
    int bestRight = -1000;
    for (int i = 0; i < angleMax; ++i) {
      int leftStart = 1000;
      int rightStart = -1000;
      if (!occupiedRange[i + 90]) {
        if (leftStart == 1000) {
          leftStart = i;
        } else {
          if (i - leftStart > 20) {
            bestLeft = i - 10;
          }
        }
      } else {
        leftStart = 1000;
      }
      if (!occupiedRange[-i + 90]) {
        if (rightStart == -1000) {
          rightStart = -i;
        } else {
          if (i - rightStart < -20) {
            bestRight = i + 10;
          }
        }
      } else {
        rightStart = -1000;
      }
    }

    int bestAngle = -1000;
    if (bestLeft != 1000) {
      if (bestRight != -1000) {
        bestAngle = abs(bestLeft) < abs(bestRight) ? bestLeft : bestRight;
      } else {
        bestAngle = bestLeft;
      }
    } else if (bestRight != -1000) {
      bestAngle = bestRight;
    }

    if (bestAngle != -1000) {
      float dist = 3.f;
      float angle = bestAngle * M_PI / 180;
      lastKickTarget.x = dist * cos(angle);
      lastKickTarget.y = dist * sin(angle);
      return true;
    } else {
      return false;
    }
  }
}

void
GoalKeeper::alignToKickAction()
{
  // Align to pass the ball to best target
  RobotPose2D<float> target;
  findBestBallAlignment(target);
  // tolerance of 5cm, 5 cm, and 5 degrees
  if (lastMBFinished) {
    goToTarget(target, 0.05, 0.05, 0.087222222, true);
    behaviorState = kickBall;
  }
}

void
GoalKeeper::kickBallAction()
{
  if (lastMBFinished) {
    auto kConfig = 
      boost::make_shared<JSOKickConfig>(
        IVAR(BallInfo, PlanningModule::ballInfo).posRel
      );
    kConfig->target = lastKickTarget;
    setupMBRequest(kConfig);
    behaviorState = correctPosture;
  }
}

void
GoalKeeper::findBestBallAlignment(RobotPose2D<float>& alignPosition)
{
  bool useLeftFoot = false;
  auto ballWorld = IVAR(TeamBallInfo, PlanningModule::teamBallInfo).posWorld;
  auto diff = lastKickTarget - ballWorld;
  auto angle = atan2(diff.y, diff.x);
  auto ct = cos(angle);
  auto st = sin(angle);
  if (angle < 0) {
    useLeftFoot = true;
  } else {
    useLeftFoot = false;
  }
  auto& rPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  if (fabsf(angle) < 30 * M_PI / 180) {
    alignPosition.x = ballWorld.x + -0.165;
    alignPosition.theta = rPose.theta;
    if (!useLeftFoot) { // Right foot in front of ball
      alignPosition.y = ballWorld.y + 0.05;
    } else { // Left foot in front of ball
      alignPosition.y = ballWorld.y - 0.05;
    }
  } else {
    alignPosition.x = ballWorld.x + -0.165;
    alignPosition.y = ballWorld.y;
    alignPosition.theta = rPose.theta;
  }
}

bool
GoalKeeper::behindObstacle(const Point2f& target)
{
  auto& obstaclesObs = IVAR(ObsObstacles, PlanningModule::obstaclesObs);
  for (int i = 0; i < obstaclesObs.data.size(); ++i) {
    auto obs = obstaclesObs.data[i];
    auto p1 = Point2f(obs.leftBound.x, obs.leftBound.y);
    auto p2 = Point2f(obs.rightBound.x, obs.rightBound.y);
    auto diff = p2 - p1;
    auto mag = norm(diff);
    Point2f unit;
    unit.x = diff.x / mag;
    unit.y = diff.y / mag;
    auto line = Vec4f(p1.x, p1.y, p2.x, p2.y);
    auto targetLine = Vec4f(0.f, 0.f, target.x, target.y);
    auto inter = VisionUtils::findIntersection(line, targetLine);
    if (inter.x != -100) {
      Point2f diffInter = inter - p1;
      auto magInter = norm(diffInter);
      auto unitInter = Point2f(diffInter.x / magInter, diffInter.y / magInter);
      if (unitInter.x / (float) unit.x < 0) {
        continue;
      } else {
        float r = magInter / (float) mag;
        if (r > 0.0 && r < 1.0) {
          return true;
        }
      }
    }
  }
}
