/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file implements the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadTargetSearch.h"

HeadTargetSearchConfigPtr HeadTargetSearch::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetSearchConfig> (config);
}

void
HeadTargetSearch::initiate()
{
	PRINT("HeadTargetSearch.initiate()")
  totalWaitTime = getBehaviorCast()->totalWaitTime;
  targetType = getBehaviorCast()->headTargetType;
  inBehavior = true;
}

void
HeadTargetSearch::update()
{
	PRINT("HeadTargetSearch.update()")
  float targetZ;
  Point2f targetXY;
  if (findTarget(targetType, targetXY, targetZ)) {
	  finish();
	  return;
    if (targetZ < 0.f) { // Used for killing behavior if a target is found
      PRINT("HeadTargetSearch.update(): Cannot move to target. Finishing behavior...")
      finish();
    }
    Vector4f posCam, posWorld;
    posWorld = Vector4f(targetXY.x, targetXY.y, targetZ, 1.f);
    if (
      norm(targetXY) < 0.6 &&
      targetZ < 0.5) 
    { // If targetXY is within 60 cm and z is reasonably low, use lower cam
      posCam = kM->getWorldToCam(BOTTOM_CAM, posWorld);
    } else { // else use upper cam
      posCam = kM->getWorldToCam(TOP_CAM, posWorld);  
    }
    if (moveHeadToTarget(posCam)) { // Target reached
      PRINT("HeadTargetSearch.update(): Target reached. Finishing behavior...")
      finish();
    }
  } else {
    scanEnv();
  }
}

bool HeadTargetSearch::moveHeadToTarget(const Vector4f& posCam)
{
  Vector2f error;
  error[HEAD_YAW] = atan2(posCam[2], posCam[0]) - M_PI / 2;
  error[HEAD_PITCH] = -(atan2(posCam[2], posCam[1]) - M_PI / 2);
  float headYaw = kM->getJointPosition(HEAD_YAW);
  float headPitch = kM->getJointPosition(HEAD_PITCH);
  headYaw += error[HEAD_YAW];
  headPitch += error[HEAD_PITCH];
  AL::ALValue names = AL::ALValue::array("HeadYaw", "HeadPitch");
  AL::ALValue angles = AL::ALValue::array(headYaw, headPitch);
  float fractionMaxSpeed = 0.1f;
  if (
    abs(error[HEAD_YAW]) > 0.087222222 || 
    abs(error[HEAD_PITCH]) > 0.087222222) 
  { // 5 degrees
    motionProxy->setAngles(names, angles, fractionMaxSpeed);
    return false;
  } else {
    return true;
  }
}

void
HeadTargetSearch::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}

void
HeadTargetSearch::scanEnv()
{
  float leftScanMax = getBehaviorCast()->scanMaxYaw;
  float rightScanMax = -getBehaviorCast()->scanMaxYaw;
  float headYaw = kM->getJointPosition(HEAD_YAW);
  float headPitch = kM->getJointPosition(HEAD_PITCH);
  AL::ALValue nameYaw = AL::ALValue::array("HeadYaw");
  AL::ALValue angleYaw = AL::ALValue::array(0.f);
  AL::ALValue namePitch = AL::ALValue::array("HeadPitch");
  AL::ALValue anglePitch = AL::ALValue::array(0.f);
  float fractionMaxSpeed = 0.1f;
  if (behaviorState == midScan) {
    if (getBehaviorCast()->scanLowerArea)
      anglePitch[0] = getBehaviorCast()->scanMaxPitch;
    angleYaw[0] = 0.f;
    motionProxy->setAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = midWait;
    waitTime = 0;
  } else if (behaviorState == midWait) {
    //cout << "Mid wait..." << endl;
    //cout << "MidWaitTime: " << waitTime << endl;
    if (abs(headYaw) < 2.5 * M_PI / 180) {
      waitTime += cycleTime;
      if (waitTime > totalWaitTime) behaviorState = leftScan;
    }
  } else if (behaviorState == leftScan) {
    //cout << "Left scan..." << endl;
    angleYaw[0] = leftScanMax;
    motionProxy->setAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = leftWait;
    waitTime = 0;
  } else if (behaviorState == leftWait) {
    //cout << "Left wait..." << endl;
    //cout << "LeftWaitTime: " << waitTime << endl;
    if (abs(headYaw - leftScanMax) < 2.5 * M_PI / 180) {
      waitTime += cycleTime;
      if (waitTime > totalWaitTime) behaviorState = rightScan;
    }
  } else if (behaviorState == rightScan) {
    //cout << "Right scan..." << endl;
    angleYaw[0] = rightScanMax;
    motionProxy->setAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = rightWait;
    waitTime = 0;
  } else if (behaviorState == rightWait) {
    //cout << "Right wait..." << endl;
    //cout << "RightWaitTime: " << waitTime << endl;
    if (abs(headYaw - rightScanMax) < 2.5 * M_PI / 180) {
      waitTime += cycleTime;
      if (waitTime > totalWaitTime) behaviorState = finishState;
    }
  } else if (behaviorState == finishState) {
    angleYaw[0] = 0.f;
    anglePitch[0] = 16.f * M_PI / 180;
    motionProxy->setAngles(nameYaw, angleYaw, fractionMaxSpeed);
    motionProxy->setAngles(namePitch, anglePitch, fractionMaxSpeed);
    if (
      abs(headYaw) < 2.5 * M_PI / 180 && 
      abs(headPitch - 16.0 * M_PI / 180) < 2.5 * M_PI / 180) 
    {
      inBehavior = false;
    }
  }
}
