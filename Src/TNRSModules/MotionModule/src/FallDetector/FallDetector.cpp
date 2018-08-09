/**
 * @file MotionModule/src/FallDetector/FallDetector.cpp
 *
 * This file implements the class FallDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017
 */

#include "MotionModule/include/FallDetector/FallDetector.h"

void FallDetector::update()
{
  Vector3f torsoAcceleration = kM->getTorsoAccel();
  torsoAccBuffer.push_back(torsoAcceleration);
  if (torsoAccBuffer.size() >= bufferSize) {
    Vector3f avgAcc = Vector3f::Zero();
    for (int i = 0; i < torsoAccBuffer.size(); ++i)
      avgAcc = avgAcc + torsoAccBuffer[i];
    avgAcc = avgAcc / bufferSize;
    float angleXZ(
      atan2(avgAcc[0], sqrt(avgAcc[1] * avgAcc[1] + avgAcc[2] * avgAcc[2])));
    float angleYZ(atan2(-avgAcc[1], -avgAcc[2]));
    //cout << "kM->getFootOnGround(): " << kM->getFootOnGround() << endl;
    //cout << "avgAcc: " << avgAcc << endl;
    //cout << "angleXZ: " << angleXZ * 180 / M_PI  << endl;
    //cout << "angleYZ: " << angleYZ * 180 / M_PI << endl;
    if (kM->getFootOnGround() == -1) {
      PostureState posture = OVAR(PostureState, MotionModule::postureState);
      if (angleXZ > M_PI / 4) { // 45 degrees
        if (angleXZ > M_PI / 3) {
          posture = PostureState::FALL_FRONT;
          OVAR(bool, MotionModule::robotFallen) = true;
        } else {
          posture = PostureState::FALLING_FRONT;
        }
      } else if (angleXZ < -M_PI / 4) { // -45 degrees
        if (angleXZ < -M_PI / 3) {
          posture = PostureState::FALL_BACK;
          OVAR(bool, MotionModule::robotFallen) = true;
        } else {
          posture = PostureState::FALLING_BACK;
        }
      } else {
        posture = PostureState::FALL_SIT;
        OVAR(bool, MotionModule::robotFallen) = true;
      }
      OVAR(PostureState, MotionModule::postureState) = posture;
    } else {
      OVAR(bool, MotionModule::robotFallen) = false;
    }
  } else {
    OVAR(bool, MotionModule::robotFallen) = false;
  }
}
