/**
 * @file MotionModule/HeadControl/Types/HeadTargetTrack.h
 *
 * This file implements the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"

Vector3f HeadTargetTrack::pidGains;

HeadTargetTrackConfigPtr HeadTargetTrack::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetTrackConfig> (config);
}

void
HeadTargetTrack::initiate()
{
	PRINT("HeadTargetTrack.initiate()")
  targetType = getBehaviorCast()->headTargetType;
  inBehavior = true;
}

void
HeadTargetTrack::update()
{
	PRINT("HeadTargetTrack.update()")
  float targetZ;
  Point2f targetXY;
  if (findTarget(targetType, targetXY, targetZ)) {
    if (targetZ < 0.f) { // Used for killing behavior if a target is found and cannot be tracked
      PRINT("HeadTargetTrack.update(): Cannot track target. Finishing behavior...")
      finish();
    }
    Vector4f posCam, posWorld;
    posWorld = Vector4f(targetXY.x, targetXY.y, targetZ, 1.f);
    if (
      norm(targetXY) < 0.6 &&
      targetZ < 0.5f) 
    { // If targetXY is within 60 cm and z is reasonably low, use lower cam
      posCam = kM->getWorldToCam(BOTTOM_CAM, posWorld);
    } else { // else use upper cam
      posCam = kM->getWorldToCam(TOP_CAM, posWorld);  
    }
    followTarget(posCam);
  } else {
    targetLostTime += cycleTime;
    if (targetLostTime > 5.f) {
      PRINT("HeadTargetTrack.update(): Target lost. Finishing behavior...")
      finish();
    }
  }
}

void
HeadTargetTrack::followTarget(const Vector4f& posCam)
{
  // TODO: Fix this. Too messy. Also add the use of pid somewhere e.e 
  Vector2f dAngles, aAngles;
  Vector2f error, command;
  aAngles[HEAD_YAW] = kM->getJointPosition(HEAD_YAW);
  aAngles[HEAD_PITCH] = kM->getJointPosition(HEAD_PITCH);
  error[HEAD_YAW] = //0;//-35 * M_PI / 180 - aAngles[HEAD_YAW];
    atan2(posCam[2], posCam[0]) - M_PI / 2;
  error[HEAD_PITCH] = //0;//-35 * M_PI / 180 - aAngles[HEAD_PITCH];
    atan2(posCam[2], posCam[1]) - M_PI / 2;
  command = pidGains[0] * error + pidGains[1] * intError;
  command[HEAD_PITCH] = -command[HEAD_PITCH];
  /*command =
   prevCommand +
   (pidKp + pidKi * cycleTime / 2 + pidKd / cycleTime) * error +
   (-pidKp + pidKi * cycleTime / 2 - 2 * pidKd / cycleTime) * errorK1 +
   (pidKd / cycleTime) * errorK2;
   prevCommand = command;
   errorK2 = errorK1;
   errorK1 = error;
   command[HEAD_YAW] += aAngles[HEAD_YAW];
   command[HEAD_PITCH] += aAngles[HEAD_PITCH];*/
  intError += error;
  Vector2f fAngle;
  fAngle[HEAD_YAW] = aAngles[HEAD_YAW] + command[HEAD_YAW];
  fAngle[HEAD_PITCH] = aAngles[HEAD_PITCH] - command[HEAD_PITCH];
  if (fAngle[HEAD_YAW] >= headYawHigh) {
    command[HEAD_YAW] = 0.f;
  } else if (fAngle[HEAD_YAW] <= headYawLow) {
    command[HEAD_YAW] = 0.f;
  }
  if (fAngle[HEAD_PITCH] >= headPitchHigh) {
    command[HEAD_PITCH] = 0.f;
  } else if (fAngle[HEAD_PITCH] <= headPitchLow) {
    command[HEAD_PITCH] = 0.f;
  }
  AL::ALValue names = AL::ALValue::array("HeadYaw", "HeadPitch");
  AL::ALValue angles = AL::ALValue::array(
    command[HEAD_YAW],
    command[HEAD_PITCH]);
  float fractionMaxSpeed = 0.5f;
  if (
    abs(error[HEAD_YAW]) > 0.087222222 || 
    abs(error[HEAD_PITCH]) > 0.087222222) 
  { // 5 degrees
    motionProxy->changeAngles(names, angles, fractionMaxSpeed);
  }
}

void
HeadTargetTrack::finish()
{
	PRINT("HeadTargetTrack.finish()")
	AL::ALValue nameYaw = AL::ALValue::array("HeadYaw");
	AL::ALValue angleYaw = AL::ALValue::array(0.f);
	AL::ALValue namePitch = AL::ALValue::array("HeadPitch");
	AL::ALValue anglePitch = AL::ALValue::array(16.f * M_PI / 180);
	float fractionMaxSpeed = 0.1f;
	motionProxy->setAngles(nameYaw, angleYaw, fractionMaxSpeed);
	motionProxy->setAngles(namePitch, anglePitch, fractionMaxSpeed);
  float headYaw = kM->getJointPosition(HEAD_YAW);
  float headPitch = kM->getJointPosition(HEAD_PITCH);
  cout << "headYaw: " << headYaw << endl;
	cout << "headPitch: " << headPitch << endl;
	if (
		abs(headYaw) < 2.5 * M_PI / 180 && 
		abs(headPitch - 16.0 * M_PI / 180) < 2.5 * M_PI / 180) 
	{
		cout << "headYaw: " << headYaw << endl;
		cout << "headPitch: " << headPitch << endl;
		motionProxy->killAll();
		inBehavior = false;
	}
}

void 
HeadTargetTrack::loadExternalConfig()
{
  GET_CONFIG("MotionBehaviors",
    (float, HeadTargetTrack.kp, pidGains[0]),
    (float, HeadTargetTrack.ki, pidGains[1]),
    (float, HeadTargetTrack.kd, pidGains[2]),
  )
}
