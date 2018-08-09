/**
 * @file Utils/include/TeamRobot.h
 *
 * This file declares and implements the struct TeamRobot
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 Jan 2018
 */

#pragma once

/**
 * @struct TeamRobot
 * @brief The struct  for defining team robots data
 */
struct TeamRobot
{
  TeamRobot() :
    dataReceived(false), fallen(0), intention(4), suggestionToMe(0),
      positionConfidence(0), sideConfidence(0), ballAge(-1)
  {
  }

  void
  print()
  {
    cout << "dataReceived: " << dataReceived << endl;
    cout << "fallen: " << (int) fallen << endl;
    cout << "intention: " << (int) intention << endl;
    cout << "suggestionToMe: " << (int) suggestionToMe << endl;
    cout << "pose.x : " << pose.x << endl;
    cout << "pose.y: " << pose.y << endl;
    cout << "pose.theta: " << pose.theta << endl;
    cout << "positionConfidence: " << (int) positionConfidence << endl;
    cout << "sideConfidence: " << (int) sideConfidence << endl;
    cout << "ballPos: " << ballPos << endl;
    cout << "ballVel: " << ballVel << endl;
    cout << "ballAge: " << ballAge << endl;
  }

  bool dataReceived;
  int8_t fallen;
  int8_t intention;
  int8_t suggestionToMe;
  int8_t positionConfidence;
  int8_t sideConfidence;
  float ballAge;
  Point2f ballPos;
  Point2f ballVel;
  RobotPose2D<float> pose;
  RobotPose2D<float> walkingTo;
  RobotPose2D<float> shootingTo;
};
