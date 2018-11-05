/**
 * @file MotionModule/src/KickModule/JSE2DImpKick.cpp
 *
 * This file implements the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"

JSE2DImpKickConfigPtr JSE2DImpKick::getBehaviorCast()
{
  return boost::static_pointer_cast <JSE2DImpKickConfig> (config);
}

void
JSE2DImpKick::initiate()
{
  LOG_INFO("JSE2DImpKick.initiate()")
  if (!setupKickBase()) return;
  //if (getBehaviorCast()->postureConfig)
  //  behaviorState = posture;
  //else if (getBehaviorCast()->balanceConfig)
  //  behaviorState = balance;
  //else
  //  behaviorState = kick;
  //inBehavior = true;
}

void
JSE2DImpKick::update()
{
  LOG_INFO("JSE2DImpKick.update()")
  if (behaviorState == posture) {
    LOG_INFO("behaviorState posture")
    if (lastChildCfg && 
        lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      behaviorState = balance;
    } else {
      setupPosture();
    }
  } else if (behaviorState == balance) {
    LOG_INFO("behaviorState balance")
    if (lastChildCfg && 
        lastChildCfg->id == (unsigned)MBIds::BALANCE) 
    {
      behaviorState = kick;
    } else {
      setupBalance();
    }
  } else if (behaviorState == kick) {
    if (!kickSetup) {
	  LOG_INFO("kick setup")
      // After the robot has gone into balance
      setTransformFrames();
      defineTrajectory();
      //plotKick();
      requestExecution();
      kickSetup = true;
    } else {
      if (kickTime > totalTimeToKick + cycleTime / 2)
        behaviorState = postKickPosture;
      else
        kickTime += cycleTime;
    }
  } else if (behaviorState == postKickPosture) {
    LOG_INFO("behaviorState postKickPosture")
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
JSE2DImpKick::finish()
{
  this->killAllMotions();
  inBehavior = false;
}

bool
JSE2DImpKick::setupKickBase() throw (BehaviorException)
{
  LOG_INFO("JSE2DImpKick.setupKickBase()")
  try {
    auto& ball = getBehaviorCast()->ball;
    auto& target = getBehaviorCast()->target;
    auto& reqVel = getBehaviorCast()->reqVel;
    if (target.x != -1.f) { // if target is defined
      ballPosition = Vector3f(ball.x, ball.y, -footHeight + ballRadius);
	  ballVelocity = 
		Vector3f(getBehaviorCast()->ballVel.x, getBehaviorCast()->ballVel.y, 0.f);
			targetPosition = Vector3f(target.x, target.y, -footHeight + ballRadius);
			auto ballToTarget = targetPosition - ballPosition;
			targetDistance = ballToTarget.norm();
			ballToTargetUnit = ballToTarget / targetDistance;
			targetAngle = atan2(ballToTargetUnit[1], ballToTargetUnit[0]);
			cout << "targetAngle: " << targetAngle << endl;
      if (!setKickSupportLegs()) {
        throw BehaviorException(
          this,
          "Unable to decide kick and support legs for the given ball position.",
          false,
          EXC_INVALID_BEHAVIOR_SETUP
        );
      }
      if (!setTransformFrames())
        return false;
      float footSpacing = supportToKick(1, 3) / 2;
      // Sending ball from feet center frame to base support leg frame
      ballPosition[1] += footSpacing;
	  targetPosition[1] += footSpacing;
	  setDesBallVel();
	  solveForImpact();
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
JSE2DImpKick::solveForImpact()
{
  kickImpact2DSolver->optDef();
}
