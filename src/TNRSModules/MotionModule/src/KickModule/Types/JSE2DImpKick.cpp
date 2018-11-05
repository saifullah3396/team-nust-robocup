/**
 * @file MotionModule/src/KickModule/JSE2DImpKick.cpp
 *
 * This file implements the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

//#include "MotionModule/include/JointCmdsRecorder.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"

template <typename Scalar>
JSE2DImpKick<Scalar>::~JSE2DImpKick()
{
  if (kickImpact2DSolver) {
    delete kickImpact2DSolver;
    kickImpact2DSolver = NULL;
  }
}

template <typename Scalar>
JSE2DImpKickConfigPtr JSE2DImpKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSE2DImpKickConfig> (this->config);
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("MotionBehaviors",
      (bool, JSE2DImpKick.logData, this->logData),
    )
    
    GET_CONFIG("EnvProperties", 
      (Scalar, ballRadius, this->ballRadius), 
      (Scalar, ballMass, this->ballMass), 
      (Scalar, coeffSF, this->sf), 
      (Scalar, coeffRF, this->rf),
      (Scalar, coeffDamping, this->coeffDamping),
    )
    this->lFootContour = 
      new BSpline<Scalar>(
        ConfigManager::getConfigDirPath() + "left_foot_contour.xml");
    this->rFootContour = 
      new BSpline<Scalar>(
        ConfigManager::getConfigDirPath() + "right_foot_contour.xml");
    this->logFootContours();
    
    loaded = true;
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::initiate()
{
  setupKickBase();
  if (this->logData)
    MOTION_LOGGER->setRefTime(this->getBehaviorCast()->timeAtEstimation);
  if (this->getBehaviorCast()->postureConfig)
    this->behaviorState = this->posture;
  else if (this->getBehaviorCast()->balanceConfig)
    this->behaviorState = this->balance;
  else
    this->behaviorState = this->kick;
  this->inBehavior = true;
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::update()
{
  if (this->behaviorState == this->posture) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      this->behaviorState = this->balance;
    } else {
      this->setupPosture();
    }
  } else if (this->behaviorState == this->balance) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::BALANCE) 
    {
      this->behaviorState = this->kick;
    } else {
      this->setupBalance();
    }
  } else if (this->behaviorState == this->kick) {
    static bool kickExecuted = false;
    if (!this->kickSetup) {
      // After the robot has gone into this->balance
      this->setTransformFrames();
      solveForImpact();
      this->defineTrajectory();
      //plotKick();
      this->kickSetup = true;
      //cout << "TimeStart:"  << timeStart << endl;
    } else if (!kickExecuted) {
      double timeFromStart = 
        duration<double>(
          high_resolution_clock::now() - 
          this->getBehaviorCast()->timeAtEstimation
        ).count();
      double timeToKick = 
        this->getBehaviorCast()->timeUntilImpact - this->kickTimeToImpact;
      if (MathsUtils::almostEqual(timeFromStart, timeToKick, (double)this->cycleTime)) {
        //cout << "this->kick starting" << endl;
        //cout << "timeFromStart:" << timeFromStart << endl;
        //cout << "timeToKick:" << timeToKick << endl;
        this->requestExecution();
        kickExecuted = true;
      }
    } else {
      if (this->kickTimeStep > this->totalTimeToKick + this->cycleTime / 2)
        this->behaviorState = this->postKickPosture;
      else
        this->kickTimeStep += this->cycleTime;
    }
  } else if (this->behaviorState == this->postKickPosture) {
    LOG_INFO("this->behaviorState postKickPosture")
    if (this->lastChildCfg && 
      this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      finish();
    } else {
      this->setupPosture();
    }
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::finish()
{
  this->killAllMotions();
  this->inBehavior = false;
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::setupKickBase()
{
  try {
    //LOG_INFO("JSE2DImpKick.setupKickBase()")
    auto& ball = this->getBehaviorCast()->ball;
    auto& target = this->getBehaviorCast()->target;
    auto& reqVel = this->getBehaviorCast()->reqVel;
    //cout << "ball: " << ball << endl;
    //cout << "target: " << target << endl;
    //cout << "reqVel: " << reqVel << endl;
    if (target.x != -1.f) { // if target is defined
      this->ballPosition = Matrix<Scalar, 3, 1>(ball.x, ball.y, -footHeight + this->ballRadius);
      this->ballVelocity =
        Matrix<Scalar, 3, 1>(this->getBehaviorCast()->ballVel.x, this->getBehaviorCast()->ballVel.y, 0.f);
      this->targetPosition = Matrix<Scalar, 3, 1>(target.x, target.y, -footHeight + this->ballRadius);
      auto ballToTarget = this->targetPosition - this->ballPosition;
      this->targetDistance = ballToTarget.norm();
      this->ballToTargetUnit = ballToTarget / this->targetDistance;
      this->targetAngle = atan2(this->ballToTargetUnit[1], this->ballToTargetUnit[0]);
      if (!this->setKickSupportLegs()) {
        throw BehaviorException(
          this,
          "Unable to decide this->kick and support legs for the given ball position.",
          false,
          EXC_INVALID_BEHAVIOR_SETUP
        );
      }
      if (!this->setTransformFrames()) {
        throw BehaviorException(
          this,
          "Unable to set initial transformation frames for the kick.",
          false,
          EXC_INVALID_BEHAVIOR_SETUP
        );
      }
      float footSpacing = this->supportToKick(1, 3) / 2;
      // Sending ball from feet center frame to base support leg frame
      this->ballPosition[1] += footSpacing;
      this->targetPosition[1] += footSpacing;
    } else {
      throw BehaviorException(
        this,
        "Required this->kick parameters 'ball' or 'target are not well-defined",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    this->inBehavior = false;
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::solveForImpact()
{
  this->kickImpact2DSolver->optDef();
}

template class JSE2DImpKick<MType>;
