/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file implements the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadTargetSearch.h"

#define TARGET_TOL 0.043631944 // radians in 2.5deg

template <typename Scalar>
HeadTargetSearchConfigPtr HeadTargetSearch<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetSearchConfig> (this->config);
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::initiate()
{
	//LOG_INFO("HeadTargetSearch.initiate()")
  this->totalWaitTime = this->getBehaviorCast()->totalWaitTime;
  this->targetType = getBehaviorCast()->headTargetType;
  this->inBehavior = true;
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::update()
{
  Scalar targetZ;
  Point_<Scalar> targetXY;
  if (this->findTarget(targetType, targetXY, targetZ)) {
    if (targetZ < 0.f) { // Used for killing behavior if a target is found
      LOG_INFO("HeadTargetSearch.update(): Cannot move to target. Finishing behavior...")
      finish();
    }
    Matrix<Scalar, 4, 1> posCam, posWorld;
    posWorld = Matrix<Scalar, 4, 1>(targetXY.x, targetXY.y, targetZ, 1.f);
    if (
      norm(targetXY) < 0.6 &&
      targetZ < 0.5) 
    { // If targetXY is within 60 cm and z is reasonably low, use lower cam
      posCam = this->kM->getWorldToCam(BOTTOM_CAM, posWorld);
    } else { // else use upper cam
      posCam = this->kM->getWorldToCam(TOP_CAM, posWorld);  
    }
    if (moveHeadToTarget(posCam)) { // Target reached
      LOG_INFO("HeadTargetSearch.update(): Target reached. Finishing behavior...")
      finish();
    }
  } else {
    scanEnv();
  }
}

template <typename Scalar>
bool HeadTargetSearch<Scalar>::moveHeadToTarget(const Matrix<Scalar, 4, 1>& posCam)
{
  Matrix<Scalar, 2, 1> error;
  error[HEAD_YAW] = atan2(posCam[2], posCam[0]) - M_PI / 2;
  error[HEAD_PITCH] = -(atan2(posCam[2], posCam[1]) - M_PI / 2);
  Scalar headYaw = this->kM->getJointState(HEAD_YAW)->position;
  Scalar headPitch = this->kM->getJointState(HEAD_PITCH)->position;
  headYaw += error[HEAD_YAW];
  headPitch += error[HEAD_PITCH];
  AL::ALValue names = AL::ALValue::array("HeadYaw", "HeadPitch");
  AL::ALValue angles = AL::ALValue::array(headYaw, headPitch);
  Scalar fractionMaxSpeed = 0.02;
  if (
    abs(error[HEAD_YAW]) > 0.087222222 || 
    abs(error[HEAD_PITCH]) > 0.087222222) 
  { // 5 degrees
    this->naoqiSetAngles(names, angles, fractionMaxSpeed);
    return false;
  } else {
    return true;
  }
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::finish()
{
  this->killAllMotions();
  this->inBehavior = false;
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::scanEnv()
{
  Scalar leftScanMax = getBehaviorCast()->scanMaxYaw;
  Scalar rightScanMax = -getBehaviorCast()->scanMaxYaw;
  Scalar headYaw = this->kM->getJointState(HEAD_YAW)->position;
  Scalar headPitch = this->kM->getJointState(HEAD_PITCH)->position;
  AL::ALValue nameYaw = AL::ALValue::array("HeadYaw");
  AL::ALValue angleYaw = AL::ALValue::array(0.f);
  AL::ALValue namePitch = AL::ALValue::array("HeadPitch");
  AL::ALValue anglePitch = AL::ALValue::array(0.f);
  Scalar fractionMaxSpeed = 0.1f;
  if (getBehaviorCast()->scanLowerArea) {
    anglePitch[0] = getBehaviorCast()->scanMaxPitch;
    this->naoqiSetAngles(namePitch, anglePitch, fractionMaxSpeed);
  }
  if (behaviorState == midScan) {
    angleYaw[0] = 0.f;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = midWait;
    waitTime = 0;
  } else if (behaviorState == midWait) {
    //cout << "Mid wait..." << endl;
    //cout << "MidWaitTime: " << waitTime << endl;
    if (abs(headYaw) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) behaviorState = leftScan;
    }
  } else if (behaviorState == leftScan) {
    //cout << "Left scan..." << endl;
    angleYaw[0] = leftScanMax;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = leftWait;
    waitTime = 0;
  } else if (behaviorState == leftWait) {
    //cout << "Left wait..." << endl;
    //cout << "LeftWaitTime: " << waitTime << endl;
    if (abs(headYaw - leftScanMax) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) behaviorState = rightScan;
    }
  } else if (behaviorState == rightScan) {
    //cout << "Right scan..." << endl;
    angleYaw[0] = rightScanMax;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = rightWait;
    waitTime = 0;
  } else if (behaviorState == rightWait) {
    //cout << "Right wait..." << endl;
    //cout << "RightWaitTime: " << waitTime << endl;
    if (abs(headYaw - rightScanMax) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) behaviorState = finishState;
    }
  } else if (behaviorState == finishState) {
    angleYaw[0] = 0.f;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    if (abs(headYaw) < TARGET_TOL) 
    {
      this->inBehavior = false;
    }
  }
}

template class HeadTargetSearch<MType>;
