/**
 * @file TeamComm/TeamComm.h
 *
 * This file implements the class TeamComm
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Jan 2017
 */

#include "CommModule/include/TeamComm.h"
#include "MotionModule/include/MotionBehaviorIds.h"

void
TeamComm::startUp(const int& port, const char* subnet)
{
  this->port = port;
  LOG_INFO("port: " + DataUtils::varToString(port))
  LOG_INFO("subnet: " + DataUtils::varToString(subnet))
  udpComm.setBlocking(false);
  udpComm.setBroadcast(true);
  udpComm.bind("0.0.0.0", port);
  udpComm.setTarget(subnet, port);
  udpComm.setLoopback(false);
}

void
TeamComm::send()
{
  if (!port) return;
  SPLStandardMessage outMsg;
  updateMessage(outMsg);
  /*cout << "x: " << IVAR(RobotPose2D<float>, CommModule::robotPose2D).x << endl;
   cout << "y: " <<  IVAR(RobotPose2D<float>, CommModule::robotPose2D).y << endl;
   cout << "theta: " <<  IVAR(RobotPose2D<float>, CommModule::robotPose2D).theta << endl;
   cout << "outMsg: " << endl;
   cout << "pose1: " << outMsg.pose[0] << endl;
   cout << "pose2: " << outMsg.pose[1] << endl;
   cout << "pose3: " << outMsg.pose[2] << endl;*/
  udpComm.write(
    (const char*) &outMsg,
    offsetof(SPLStandardMessage, data) + outMsg.numOfDataBytes);
}

unsigned
TeamComm::receive()
{
  if (!port) return 0;
  SPLStandardMessage inMsg;
  int size;
  unsigned remoteIp = 0;
  unsigned receivedSize = 0;
  do {
    size = udpComm.read((char*) &inMsg, sizeof(SPLStandardMessage));
    if (size >= static_cast<int>(offsetof(SPLStandardMessage, data)) && size <= static_cast<int>(sizeof(SPLStandardMessage))) {
      receivedSize = static_cast<unsigned>(size);
      receiveQueue.pushToQueue(inMsg);
    }
  } while (size > 0);
  return receivedSize;
}

void
TeamComm::updateTeamData(const SPLStandardMessage& msg)
{
  if (int(msg.playerNum) == IVAR(int, CommModule::playerNumber)) return;
  TeamRobot teamRobot;
  teamRobot.fallen = msg.fallen;
  teamRobot.ballPos = Point2f(msg.ball[0] / 1000, msg.ball[1] / 1000);
  teamRobot.ballVel = Point2f(msg.ballVel[0] / 1000, msg.ballVel[1] / 1000);
  teamRobot.pose = RobotPose2D<float>(msg.pose[0], msg.pose[1], msg.pose[2]);
  teamRobot.walkingTo = RobotPose2D<float>(
    msg.walkingTo[0],
    msg.walkingTo[1],
    0.0);
  teamRobot.shootingTo = RobotPose2D<float>(
    msg.shootingTo[0],
    msg.shootingTo[0],
    0.0);
  teamRobot.intention = msg.intention;
  teamRobot.suggestionToMe =
    msg.suggestion[IVAR(int, CommModule::playerNumber) - 1];
  teamRobot.dataReceived = true;
  teamRobot.positionConfidence = msg.currentPositionConfidence;
  teamRobot.sideConfidence = msg.currentSideConfidence;
  //teamRobot.print();
  //cout << "Player number: " << (int)msg.playerNum << endl;
  OVAR(vector<TeamRobot>, CommModule::teamRobots)[(int) msg.playerNum - 1] =
    teamRobot;
}

void
TeamComm::processReceivedMsgs()
{
  for (int i = 0; i < OVAR(vector<TeamRobot>, CommModule::teamRobots).size();
    ++i)
    OVAR(vector<TeamRobot>, CommModule::teamRobots)[i].dataReceived = false;
  while (!receiveQueue.isEmpty()) {
    updateTeamData(receiveQueue.queueFront());
    receiveQueue.popQueue();
  }
}

void
TeamComm::updateTeamBallInfo()
{
  vector<float> measurements;
  auto& localized = IVAR(bool, CommModule::robotLocalized);
  auto& ballInfo = IVAR(BallInfo, CommModule::ballInfo);
  tBTracker->predict();
  if (localized && ballInfo.found) {
    auto posRel = IVAR(BallInfo, CommModule::ballInfo).posRel;
    auto velRel = IVAR(BallInfo, CommModule::ballInfo).velRel;
    auto rP = IVAR(RobotPose2D<float>, CommModule::robotPose2D);
    auto ct = cos(rP.theta);
    auto st = sin(rP.theta);
    Point2f posWorld;
    Point2f velWorld;
    // Position is translated & rotated with robot's angle
    posWorld.x = rP.x + posRel.x * ct - posRel.y * st;
    posWorld.y = rP.y + posRel.x * st + posRel.y * ct;
    measurements.push_back(posWorld.x);
    measurements.push_back(posWorld.y);
  }
  if (measurements.empty()) {
    auto& teamRobots = OVAR(vector<TeamRobot>, CommModule::teamRobots);
    for (int i = 0; i < teamRobots.size(); ++i) {
      if (!measurements.empty()) break;
      if (!teamRobots[i].dataReceived) continue;
      if (teamRobots[i].ballAge > 0 && teamRobots[i].ballAge < 0.50f) {
        if (teamRobots[i].positionConfidence >= 50 && teamRobots[i].ballAge <= 1.f) {
          auto posRel = teamRobots[i].ballPos;
          auto velRel = teamRobots[i].ballVel;
          auto rP = teamRobots[i].pose;
          auto ct = cos(rP.theta);
          auto st = sin(rP.theta);
          Point2f teamPosWorld;
          Point2f teamVelWorld;
          // Position is translated & rotated with robot's angle
          teamPosWorld.x = rP.x + posRel.x * ct - posRel.y * st;
          teamPosWorld.y = rP.y + posRel.x * st + posRel.y * ct;
          measurements.push_back(teamPosWorld.x / 1000);
          measurements.push_back(teamPosWorld.y / 1000);
        }
      }
    }
  }
  tBTracker->updateFilter(measurements);
  Mat ballState = tBTracker->getEstimatedState();
  TeamBallInfo teamBallInfo;
  teamBallInfo.found = tBTracker->getBallFound();
  teamBallInfo.posWorld.x = ballState.at<float>(0);
  teamBallInfo.posWorld.y = ballState.at<float>(1);
  teamBallInfo.velWorld.x = ballState.at<float>(2);
  teamBallInfo.velWorld.y = ballState.at<float>(3);
  OVAR(TeamBallInfo, CommModule::teamBallInfo) = teamBallInfo;
}

void
TeamComm::updateMessage(SPLStandardMessage& msg)
{
  msg.playerNum = IVAR(int, CommModule::playerNumber);
  msg.teamNum = IVAR(int, CommModule::teamNumber);
  msg.fallen = IVAR(bool, CommModule::robotFallen);
  RobotPose2D<float> rState = IVAR(RobotPose2D<float>, CommModule::robotPose2D);
  msg.pose[0] = rState.x * 1000;
  msg.pose[1] = rState.y * 1000;
  msg.pose[2] = rState.theta;
  /*auto lastMB = IVAR(BehaviorAccepted, CommModule::lastMBehaviorAccepted);
  if (lastMB.state->id == (unsigned) MBIds::MOVEMENT && lastMB.state->started && !lastMB.state->finished) {
    auto targetState = IVAR(RobotPose2D<float>, CommModule::moveTarget);
    msg.walkingTo[0] = targetState.x * 1000;
    msg.walkingTo[1] = targetState.y * 1000;
  } else {
    msg.walkingTo[0] = rState.x * 1000;
    msg.walkingTo[1] = rState.y * 1000;
  }

  if (lastMB.state->id == (unsigned) MBIds::KICK && lastMB.state->started && !lastMB.state->finished) {
    auto target = IVAR(Vector2f, CommModule::kickTarget);
    msg.shootingTo[0] = target[0] * 1000;
    msg.shootingTo[1] = target[1] * 1000;
  } else {
    msg.shootingTo[0] = rState.x * 1000;
    msg.shootingTo[1] = rState.y * 1000;
  }*/
  //msg.ballAge TO BE FIXED
  auto ballInfo = IVAR(BallInfo, CommModule::ballInfo);
  if (ballInfo.found) {
    msg.ballAge = ballInfo.ballAge;
  } else {
    msg.ballAge = -1;
  }
  msg.ball[0] = ballInfo.posRel.x * 1000;
  msg.ball[1] = ballInfo.posRel.y * 1000;
  msg.ballVel[0] = ballInfo.velRel.x * 1000;
  msg.ballVel[1] = ballInfo.velRel.y * 1000;
  for (int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i) {
    msg.suggestion[i] = -1;
  }
  msg.intention = IVAR(int, CommModule::robotIntention);
  msg.averageWalkSpeed = 120;
  msg.maxKickDistance = 6000;
  msg.currentPositionConfidence = IVAR(int, CommModule::positionConfidence);
  msg.currentSideConfidence = IVAR(int, CommModule::sideConfidence);
  msg.numOfDataBytes = 0;
}
