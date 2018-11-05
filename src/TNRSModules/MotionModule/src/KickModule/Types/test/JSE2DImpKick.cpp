/**
 * @file MotionModule/src/KickModule/JSE2DImpKick.cpp
 *
 * This file implements the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/JointCmdsRecorder.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"
#include "MotionModule/include/JointCmdsRecorder.h"

template <typename Scalar>
JSE2DImpKick<Scalar>::~JSE2DImpKick()
{
  if (jcr) {
    delete jcr;
    jcr = NULL;
  }
}

template <typename Scalar>
JSE2DImpKickConfigPtr JSE2DImpKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSE2DImpKickConfig> (config);
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::initiate()
{
  if (!setupKickBase()) return;
  jcr = 
    new 
    JointCmdsRecorder(
      ConfigManager::getLogsDirPath() + 
      "KickModule/JSE2DImpKick/data.txt",
      getBehaviorCast()->timeAtEstimation
    );
  MotionBehavior::jcr = jcr;
  if (getBehaviorCast()->postureConfig)
    behaviorState = posture;
  else if (getBehaviorCast()->balanceConfig)
    behaviorState = balance;
  else
    behaviorState = kick;
  inBehavior = true;
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::update()
{
  if (behaviorState == posture) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      behaviorState = balance;
    } else {
      setupPosture();
    }
  } else if (behaviorState == balance) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::BALANCE) 
    {
      behaviorState = kick;
    } else {
      setupBalance();
    }
  } else if (behaviorState == kick) {
    static bool kickExecuted = false;
    if (!kickSetup) {
      // After the robot has gone into balance
      setTransformFrames();
      solveForImpact();
      defineTrajectory();
      //plotKick();
      kickSetup = true;
      //cout << "TimeStart:"  << timeStart << endl;
    } else if (!kickExecuted) {
      double timeFromStart = 
        duration<double>(
          high_resolution_clock::now() - 
          getBehaviorCast()->timeAtEstimation
        ).count();
      double timeToKick = 
        getBehaviorCast()->timeUntilImpact - kickTimeToImpact;
      if (MathsUtils::almostEqual(timeFromStart, timeToKick, (double)cycleTime)) {
        //cout << "kick starting" << endl;
        //cout << "timeFromStart:" << timeFromStart << endl;
        //cout << "timeToKick:" << timeToKick << endl;
        requestExecution();
        kickExecuted = true;
      }
    } else {
      if (kickTimeStep > totalTimeToKick + cycleTime / 2)
        behaviorState = postKickPosture;
      else
        kickTimeStep += cycleTime;
    }
  } else if (behaviorState == postKickPosture) {
    LOG_INFO("behaviorState postKickPosture")
    if (this->lastChildCfg && 
      this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      finish();
    } else {
      setupPosture();
    }
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::finish()
{
  this->killAllMotions();
  inBehavior = false;
}

template <typename Scalar>
bool JSE2DImpKick<Scalar>::setupKickBase() throw (BehaviorException)
{
  //LOG_INFO("JSE2DImpKick.setupKickBase()")
  try {
    auto& ball = getBehaviorCast()->ball;
    auto& target = getBehaviorCast()->target;
    auto& reqVel = getBehaviorCast()->reqVel;
    //cout << "ball: " << ball << endl;
    //cout << "target: " << target << endl;
    //cout << "reqVel: " << reqVel << endl;
    if (target.x != -1.f) { // if target is defined
      ballPosition = Vector3f(ball.x, ball.y, -footHeight + ballRadius);
      ballVelocity = 
        Vector3f(getBehaviorCast()->ballVel.x, getBehaviorCast()->ballVel.y, 0.f);
			targetPosition = Vector3f(target.x, target.y, -footHeight + ballRadius);
			auto ballToTarget = targetPosition - ballPosition;
			targetDistance = ballToTarget.norm();
			ballToTargetUnit = ballToTarget / targetDistance;
			targetAngle = atan2(ballToTargetUnit[1], ballToTargetUnit[0]);
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
      return true;
    } else {
      throw BehaviorException(
        this,
        "Required kick parameters 'ball' or 'target are not well-defined",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    return false;
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::solveForImpact()
{
  kickImpact2DSolver->optDef();
}
