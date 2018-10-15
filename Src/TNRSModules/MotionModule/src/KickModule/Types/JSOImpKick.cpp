/**
 * @file MotionModule/src/KickModule/JSOImpKick.cpp
 *
 * This file implements the class JSOImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/JSOImpKick.h"
#include "MotionModule/include/MotionLogger.h" 

using namespace Utils;

template <typename Scalar>
JSOImpKick<Scalar>::~JSOImpKick()
{
  if (maxMomentumEEOpt) {
    delete maxMomentumEEOpt;
    maxMomentumEEOpt = NULL;
  }
}

template <typename Scalar>
JSOImpKickConfigPtr JSOImpKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSOImpKickConfig> (this->config);
}

template <typename Scalar>
void JSOImpKick<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("MotionBehaviors",
      (bool, JSOImpKick.logData, this->logData),
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
void JSOImpKick<Scalar>::initiate()
{
  try {
    setupKickBase();
  } catch (TNRSException& e) {
    cout << e.what();
    this->inBehavior = false;
    return;
  }
  if (this->logData) {
    MOTION_LOGGER->setLogJointStates(true);
    MOTION_LOGGER->setLogJointCmds(true);
  }
  if (this->getBehaviorCast()->postureConfig)
    this->behaviorState = this->posture;
  else if (this->getBehaviorCast()->balanceConfig)
    this->behaviorState = this->balance;
  else
    this->behaviorState = this->kick;
  this->inBehavior = true;
}

template <typename Scalar>
void JSOImpKick<Scalar>::update()
{
  //PRINT("JSOImpKick.update()")
  if (this->behaviorState == this->posture) {
    //PRINT("this->behaviorState this->posture")
    if (this->lastChildCfg &&
        this->lastChildCfg->id == (unsigned)MBIds::POSTURE)
    {
      this->behaviorState = this->balance;
    } else {
      this->setupPosture();
    }
  } else if (this->behaviorState == this->balance) {
    //PRINT("this->behaviorState this->balance")
    if (this->lastChildCfg &&
        this->lastChildCfg->id == (unsigned)MBIds::BALANCE)
    {
      static float wait = 0.f;
      if (wait > 999.f)
        this->behaviorState = this->kick;
      else 
        wait += this->cycleTime;
    } else {
      this->setupBalance();
    }
  } else if (this->behaviorState == this->kick) {
    if (!this->kickSetup) {
	  //PRINT("kick setup")
      // After the robot has gone into this->balance
      this->setTransformFrames();
      findBestEEAndImpactPose();
      this->defineTrajectory();
      //if (this->logData)
      this->plotKick();
      if (!this->kickFailed)
        this->requestExecution();
      else
        this->behaviorState = this->postKickPosture;
      this->kickSetup = true;
    } else {
      if (this->kickTimeStep > this->totalTimeToKick + 1.f + this->cycleTime / 2) {
        this->behaviorState = this->postKickPosture;
      } else {
        this->logEndEffectorActual();
        this->kickTimeStep += this->cycleTime;
      }
    }
  } else if (this->behaviorState == this->postKickPosture) {
    //PRINT("this->behaviorState postKickPosture")
    //this->setTransformFrames();
    //auto eeTrans = this->supportToKick * this->endEffector;
    //cout << "eeTrans\n" << eeTrans << endl;
    finish();
    return;
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
void JSOImpKick<Scalar>::finish()
{
  this->motionProxy->killAll();
  this->inBehavior = false;
}

template <typename Scalar>
void JSOImpKick<Scalar>::setupKickBase()
{
  PRINT("JSOImpKick.setupKickBase()")
  if (this->logData) {
    JSON_ASSIGN(
      this->dataLogger->getRoot(),
      "config",
      this->getBehaviorCast()->getJson()
    );
  }
  auto& ball = this->getBehaviorCast()->ball;
  auto& target = this->getBehaviorCast()->target;
  auto& reqVel = this->getBehaviorCast()->reqVel;
  if (reqVel.x != -1.f || target.x != -1.f) {
    this->ballPosition = Matrix<Scalar, 3, 1>(ball.x, ball.y, -footHeight + this->ballRadius);
    cout << "this->ballPosition:\n" << this->ballPosition << endl;
    if (target.x != -1.f) { // if target is defined use this
      this->targetPosition = Matrix<Scalar, 3, 1>(target.x, target.y, -footHeight + this->ballRadius);
      auto ballToTarget = this->targetPosition - this->ballPosition;
      this->targetDistance = ballToTarget.norm();
      this->ballToTargetUnit = ballToTarget / this->targetDistance;
      this->targetAngle = atan2(this->ballToTargetUnit[1], this->ballToTargetUnit[0]);
      this->desImpactVelKnown = false;
    } else if (reqVel.x != -1) { // if des velocity is defined do this
      //cout << "Using reqVel." << endl;
      this->desImpactVel = Matrix<Scalar, 3, 1>(reqVel.x, reqVel.y, 0.f);
      auto velMag = norm(reqVel);
      this->ballToTargetUnit = Matrix<Scalar, 3, 1>(reqVel.x / velMag, reqVel.y / velMag, 0.f);
      this->targetAngle = atan2(reqVel.y, reqVel.x);
      this->desImpactVelKnown = true;
      //cout << "desImpactVel:\n" << desImpactVel << endl;
      //cout << "this->ballToTargetUnit:\n" << this->ballToTargetUnit << endl;
      //cout << "this->targetAngle: " << this->targetAngle << endl;
    }
    if (!this->setKickSupportLegs()) {
      throw BehaviorException(
        this,
        "Unable to decide kick and support legs for the given ball position.",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
    cout << "KickLeg: " << this->kickLeg << endl;
    cout << "supportLeg: " << this->supportLeg << endl;
    if (!this->setTransformFrames()) {
      throw BehaviorException(
        this,
        "Unable to set initial transformation frames for the kick.",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
    //cout << "supportToKick:\n" << supportToKick << endl;
    //cout << "torsoToSupport:\n" << torsoToSupport << endl;
    float footSpacing = this->supportToKick(1, 3) / 2;
    // Sending ball from feet center frame to base support leg frame
    this->ballPosition[1] += footSpacing;
    cout << "ballPosition:" << this->ballPosition << endl;
    cout << "footSpacing; " << footSpacing << endl;
    if (!this->desImpactVelKnown)
      this->targetPosition[1] += footSpacing;
    //cout << "this->ballPosition updated:\n" << this->ballPosition << endl;
    //cout << "this->targetPosition updated:\n" << this->targetPosition << endl;
    if (!this->setEndEffectorXY(this->targetAngle)) {
      throw BehaviorException(
        this,
        "Unable to set end effector for kick.",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
    if (this->logData) {
      Json::Value jsonSetup;
      JSON_ASSIGN(jsonSetup, "targetDistance", this->targetDistance);
      JSON_ASSIGN(jsonSetup, "targetAngle", this->targetAngle * 180.0 / M_PI);
      JSON_ASSIGN(jsonSetup, "kickLeg", this->kickLeg);
      JSON_ASSIGN(jsonSetup, "supportLeg", this->supportLeg);
      JSON_ASSIGN(jsonSetup, "ballToTargetUnit", JsonUtils::MatrixToJson(this->ballToTargetUnit));
      JSON_ASSIGN(jsonSetup, "footSpacingInit", this->footSpacing);
      JSON_ASSIGN(jsonSetup, "torsoToSupportInit", JsonUtils::MatrixToJson(this->torsoToSupport));
      JSON_ASSIGN(jsonSetup, "supportToKickInit", JsonUtils::MatrixToJson(this->supportToKick));
      JSON_ASSIGN(jsonSetup, "endEffectorInit", JsonUtils::MatrixToJson(this->endEffector));
      JSON_ASSIGN(this->dataLogger->getRoot(), "setup", jsonSetup);
    }
    //cout << "endEffector:\n" << endEffector << endl;
  } else {
    throw BehaviorException(
      this,
      "Required kick parameters 'ball', 'reqVel' or 'target are not well-defined",
      false,
      EXC_INVALID_BEHAVIOR_SETUP
    );
  }
}

template <typename Scalar>
void JSOImpKick<Scalar>::findBestEEAndImpactPose()
{
  this->impactPose.setIdentity();
  this->impactPose(0, 3) = this->ballPosition[0] - this->ballToTargetUnit[0] * this->ballRadius;
  this->impactPose(1, 3) = this->ballPosition[1] - this->ballToTargetUnit[1] * this->ballRadius;
  this->impactPose(2, 3) = this->ballPosition[2];
  //cout << "impactPose" << endl;
  cout << this->impactPose << endl;
  //cout << "endEffector" << endl;
  cout << this->endEffector << endl;
  //maxMomentumEEOpt->optDef();
}

template class JSOImpKick<MType>;
