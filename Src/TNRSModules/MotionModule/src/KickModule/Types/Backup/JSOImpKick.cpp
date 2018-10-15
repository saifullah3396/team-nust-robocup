/**
 * @file MotionModule/src/KickModule/JSOImpKick.cpp
 *
 * This file implements the class JSOImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/JSOImpKick.h"

JSOImpKickConfigPtr JSOImpKick::getBehaviorCast()
{
  return boost::static_pointer_cast <JSOImpKickConfig> (config);
}

void
JSOImpKick::initiate()
{
  //PRINT("JSOImpKick.initiate()")
  if (!setupKickBase()) return;
  if (getBehaviorCast()->postureConfig)
    behaviorState = posture;
  else if (getBehaviorCast()->balanceConfig)
    behaviorState = balance;
  else
	behaviorState = kick;
  inBehavior = true;
}

void
JSOImpKick::update()
{
  //PRINT("JSOImpKick.update()")
  if (behaviorState == posture) {
    //PRINT("behaviorState posture")
    if (lastChildCfg && 
        lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      behaviorState = balance;
    } else {
      setupPosture();
    }
  } else if (behaviorState == balance) {
    //PRINT("behaviorState balance")
    if (lastChildCfg && 
        lastChildCfg->id == (unsigned)MBIds::BALANCE) 
    {
      static float wait = 0.f;
      if (wait > 1.f)
        behaviorState = kick;
      else 
        wait += cycleTime;
    } else {
      setupBalance();
    }
  } else if (behaviorState == kick) {
    if (!kickSetup) {
	  //PRINT("kick setup")
      // After the robot has gone into balance
      setTransformFrames();
      defineTrajectory();
      //plotKick();
      requestExecution();
      kickSetup = true;
    } else {
      if (kickTime > totalTimeToKick + cycleTime / 2) {
        behaviorState = postKickPosture;
      } else
        kickTime += cycleTime;
    }
  } else if (behaviorState == postKickPosture) {
    //PRINT("behaviorState postKickPosture")
    setTransformFrames();
    auto eeTrans = supportToKick * endEffector;
    //cout << "eeTrans\n" << eeTrans << endl;
    if (lastChildCfg && 
      lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      finish();
    } else {
      setupPosture();
    }
  }
}

void
JSOImpKick::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}

bool
JSOImpKick::setupKickBase() throw (BehaviorException)
{
  PRINT("JSOImpKick.setupKickBase()")
  try {
    auto& ball = getBehaviorCast()->ball;
    auto& target = getBehaviorCast()->target;
    auto& reqVel = getBehaviorCast()->reqVel;
    cout << "ball: " << ball << endl;
    cout << "target: " << target << endl;
    cout << "reqVel: " << reqVel << endl;
    if (reqVel.x != -1.f || target.x != -1.f) {
      ballPosition = Vector3f(ball.x, ball.y, -footHeight + ballRadius);
      //cout << "ballPosition:\n" << ballPosition << endl;
      if (target.x != -1.f) { // if target is defined use this
        cout << "Using target." << endl;
        targetPosition = Vector3f(target.x, target.y, -footHeight + ballRadius);
        auto ballToTarget = targetPosition - ballPosition;
        targetDistance = ballToTarget.norm();
        ballToTargetUnit = ballToTarget / targetDistance;
        targetAngle = atan2(ballToTargetUnit[1], ballToTargetUnit[0]);
        desImpactVelKnown = false;
      } else if (reqVel.x != -1) { // if des velocity is defined do this
        //cout << "Using reqVel." << endl;
        desImpactVel = Vector3f(reqVel.x, reqVel.y, 0.f);
        auto velMag = norm(reqVel);
        ballToTargetUnit = Vector3f(reqVel.x / velMag, reqVel.y / velMag, 0.f);
        targetAngle = atan2(reqVel.y, reqVel.x);
        desImpactVelKnown = true;
        //cout << "desImpactVel:\n" << desImpactVel << endl;
        //cout << "ballToTargetUnit:\n" << ballToTargetUnit << endl;
        //cout << "targetAngle: " << targetAngle << endl;
      }
      if (!setKickSupportLegs()) {
        throw BehaviorException(
          this,
          "Unable to decide kick and support legs for the given ball position.",
          false,
          EXC_INVALID_BEHAVIOR_SETUP
        );
      }
      //cout << "KickLeg: " << kickLeg << endl;
      //cout << "supportLeg: " << supportLeg << endl;
      if (!setTransformFrames())
        return false;
      //cout << "supportToKick:\n" << supportToKick << endl;
      //cout << "torsoToSupport:\n" << torsoToSupport << endl;
      float footSpacing = supportToKick(1, 3) / 2;
      // Sending ball from feet center frame to base support leg frame
      ballPosition[1] += footSpacing;
      if (!desImpactVelKnown)
        targetPosition[1] += footSpacing;
      //cout << "ballPosition updated:\n" << ballPosition << endl;
      //cout << "targetPosition updated:\n" << targetPosition << endl;
      if (!setEndEffectorXY(targetAngle)) {
        PRINT("Unable to set end effector for kick.")
        return false;
      }
      //cout << "endEffector:\n" << endEffector << endl;
      return true;
    } else {
      throw BehaviorException(
        this,
        "Required kick parameters 'ball', 'reqVel' or 'target are not well-defined",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    return false;
  }
}

void
JSOImpKick::findBestEEAndImpactPose()
{
  maxMomentumEEOpt->optDef();
}
