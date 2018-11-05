/**
 * @file MotionModule/include/Kicthis->kModule/JointSpaceKick.cpp
 *
 * This file implements the class JointSpaceKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "Utils/include/PlotEnv.h"
#include "Utils/include/JsonLogger.h"

using namespace GnuPlotEnv;
using namespace Utils;

template <typename Scalar>
JSKickConfigPtr JointSpaceKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSKickConfig> (this->config);
}

template <typename Scalar>
void JointSpaceKick<Scalar>::setupBalance()
{
  //LOG_INFO("JointSpaceKick.setupBalance()")
  if (this->getBehaviorCast()->balanceConfig) {
    this->getBehaviorCast()->balanceConfig->supportLeg = this->supportLeg;
    this->setupChildRequest(this->getBehaviorCast()->balanceConfig);
  } else {
    LOG_INFO("JointSpaceKiflBottomck.setupBalance(): No balance config found.")
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::setDesBallVel()
{
	//desBallVel = sqrt(targetDistance / (0.026 / rf + 0.02496 / sf));
  //! Solving ball distance equation which is made up of two parts;
  //! Distance covered by the ball until the ball starts to roll
  //! And the distance covered after rolling under damped motion
  //! targetDistance = posI + velI / damping * (1 - exp(-damping * time));
  //! posI is set to zero as we are finding distance relative to ball
  //! The quadratic equation for the ball distance becomes;
  Scalar vRatio = 5.0 / 7.0;
  Scalar a = (vRatio * vRatio - 1.0) / (-2.0 * this->sf * gConst);
  Scalar b = vRatio * (1.0 / this->coeffDamping);
  Scalar c = -targetDistance;
  Scalar sol1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  if (sol1 < 0) {
    Scalar sol2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    desBallVel = sol2;
  } else {
    desBallVel = sol1;
  }
}
  
template <typename Scalar>
void JointSpaceKick<Scalar>::computeDesImpactVel(const Matrix<Scalar, Dynamic, 1>& impJoints)
{
  Scalar coeffRest = 1.f; // Full elastic collision
  this->setDesBallVel();
	Matrix<Scalar, 3, 1> direction = 
		this->torsoToSupport.block(0, 0, 3, 3) * this->ballToTargetUnit;
	// impact pose joints
  this->kM->setChainPositions(this->kickLeg, impJoints, JointStateType::SIM);
  Scalar vm;
  this->kM->computeVirtualMass(
    this->kickLeg,
    direction,
    this->endEffector,
    vm,
    JointStateType::SIM);
   //cout << "Virtual mass in given direction and found pose: " << vm << endl;
   Scalar desEndEffVel = 
    this->desBallVel * (vm + this->ballMass) / ((1 + coeffRest) * vm);
    
   desImpactVel[0] = desEndEffVel * this->ballToTargetUnit[0];
   desImpactVel[1] = desEndEffVel * this->ballToTargetUnit[1];
   desImpactVel[2] = 0.f;
   
   if (this->logData) {
     Json::Value jsonImpact;
     JSON_ASSIGN(jsonImpact, "desBallVel", this->desBallVel);
     JSON_ASSIGN(jsonImpact, "ballMass", this->ballMass);
     JSON_ASSIGN(jsonImpact, "coeffRest", coeffRest);
     JSON_ASSIGN(jsonImpact, "virtualMass", vm);
     JSON_ASSIGN(jsonImpact, "desEndEffVel", desEndEffVel);
     JSON_ASSIGN(jsonImpact, "desImpactVel", JsonUtils::MatrixToJson(desImpactVel));
     JSON_ASSIGN(this->dataLogger->getRoot(), "impact", jsonImpact);
   } 
}

template <typename Scalar>
void JointSpaceKick<Scalar>::defineTrajectory()
{
  auto eeTrans = this->supportToKick * this->endEffector;
  
  //! Define via points for final trajectory.
  //cout << "ballRadius: " << ballRadius << endl;
  const Scalar balldx = this->ballRadius * this->ballToTargetUnit[0];
  const Scalar balldy = this->ballRadius * this->ballToTargetUnit[1];
  Matrix<Scalar, 4, 4> postImpactPose3, postImpactPose2, postImpactPose1;
  postImpactPose3.setIdentity();
  postImpactPose3.block(0, 3, 3, 1) = eeTrans.block(0, 3, 3, 1); // Same orientation as left foot
  postImpactPose2 = postImpactPose3;
  postImpactPose2(2, 3) = 0.0; // The height of the ankle
  postImpactPose1.setIdentity();
  postImpactPose1(0, 3) = this->ballPosition[0];
  postImpactPose1(1, 3) = this->ballPosition[1];
  postImpactPose1(2, 3) = this->ballPosition[2];
  //! Define via points for preImpactPose phase.
  Matrix<Scalar, 4, 4> preImpactPose3, preImpactPose2, preImpactPose1;
  //preImpactPose3.setIdentity();
  //const Scalar preImpTolX = 0.0 * this->ballToTargetUnit[0];
  //const Scalar preImpTolY = 0.0 * this->ballToTargetUnit[1];
  preImpactPose3.setIdentity();
  preImpactPose3(0, 3) = this->impactPose(0, 3);
  preImpactPose3(1, 3) = this->impactPose(1, 3);
  preImpactPose3(2, 3) = this->impactPose(2, 3);
  preImpactPose3(0, 3) =
    preImpactPose3(0, 3) - this->ballToTargetUnit[0] * this->ballRadius / 2;
  preImpactPose3(1, 3) =
    preImpactPose3(1, 3) - this->ballToTargetUnit[1] * this->ballRadius / 2;
  preImpactPose2.setIdentity();
  preImpactPose2(0, 3) = (preImpactPose3(0, 3) + eeTrans(0, 3)) / 2;
  preImpactPose2(1, 3) = (preImpactPose3(1, 3) + eeTrans(1, 3)) / 2;
  preImpactPose2(2, 3) = (preImpactPose3(2, 3) + eeTrans(2, 3)) / 2;
  preImpactPose2(0, 3) =
    preImpactPose2(0, 3) - balldx * 0.5;
  preImpactPose2(1, 3) =
    preImpactPose2(1, 3) - balldy * 0.5;
  preImpactPose1 = eeTrans;
  //this->impactPose.setIdentity();
  //this->impactPose(0, 3) = ballPosition[0] - balldx;
  //this->impactPose(1, 3) = ballPosition[1] - balldy;
  //this->impactPose(2, 3) = ballPosition[2];
  //cout << "EndEffector: " << this->endEffector << endl;
  //cout << "ImpactPose: " << this->impactPose << endl;
  //findBestEEAndImpactPose();
  //cout << "ImpactPose: " << this->impactPose << endl;
  auto linkChain = this->kM->getLinkChain(this->kickLeg);
  unsigned chainSize = linkChain->size;
  unsigned chainStart = linkChain->start;
  cPoses.clear();
  cPoses.push_back(preImpactPose1);
  cPoses.push_back(preImpactPose2);
  cPoses.push_back(preImpactPose3);
  cPoses.push_back(this->impactPose);
  //cPoses.push_back(postImpactPose1);
  //cPoses.push_back(postImpactPose2);
  //cPoses.push_back(postImpactPose3);
  vector<Matrix<Scalar, 4, 4>> cPosesT(cPoses.size());
  Matrix<Scalar, Dynamic, Dynamic> jointPos;
  jointPos.resize(cPosesT.size(), chainSize);
  jointPos.setZero();
  for (int i = 0; i < cPosesT.size(); ++i) {
    cPosesT[i] = this->torsoToSupport * cPoses[i];
    Matrix<Scalar, Dynamic, 1> angles;
    angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesT[i], 50, JointStateType::ACTUAL, true, 5e-2, 0.5);
    //cout << "cposes requested:\n" << cPoses[i] << endl;
    if (angles[0] != angles[0]) {
        //cout << "Requested kick cannot be performed." << endl;
        this->kickFailed = true;
        return;
    }
    int collisionCheckIterations = 10;
    if (this->checkFootCollision(angles)) {
      for (int j = 0; j < collisionCheckIterations; ++j) {
        //cout << "foot colliding: " << j << endl;
        cPoses[i](0, 3) += this->ballToTargetUnit[0] * 0.02; // Move towards the ball 2cm
        cPoses[i](1, 3) += this->ballToTargetUnit[1] * 0.02; // Move towards the ball 2cm
        cPosesT[i] = this->torsoToSupport * cPoses[i];
        Matrix<Scalar, Dynamic, 1> angles;
        angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesT[i], 50, JointStateType::ACTUAL, true, 5e-2, 0.5);
        if (!this->checkFootCollision(angles))
          break;
      }
    }
    jointPos.row(i) = angles.transpose();
  }

  //! Setting up pre-impact trajectory.
  //! Required cartesian velocities at initial and final poses
  Matrix<Scalar, Dynamic, 1> desCartesianVel;
  desCartesianVel.resize(6); //x-y-z, r-p-y
  if (!desImpactVelKnown) {
    computeDesImpactVel(jointPos.row(3));
  }
   
  //cout << "desImpactVel 2: " << desImpactVel << endl;
  desCartesianVel.block(0, 0, 3, 1) = desImpactVel; // x-y-z velocity
  //! Cartesian velocities in torso frame.
  desCartesianVel.block(0, 0, 3, 1) =
    this->torsoToSupport.block(0, 0, 3, 3) * desCartesianVel.segment(0, 3);
  //! Find last impact velocity in joints space using hitPose.
  Matrix<Scalar, Dynamic, 1> preImpJVel;
  this->kM->setJointPositions(chainStart, jointPos.row(2), JointStateType::SIM); // impact pose joints
  Matrix<Scalar, Dynamic, Dynamic> jacobian = this->kM->computeLimbJ(
    this->kickLeg,
    this->endEffector,
    JointStateType::SIM).block(0, 0, 3, 6);
  preImpJVel = MathsUtils::pseudoInverseSolve(
    jacobian,
    Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1)));

  Matrix<Scalar, Dynamic, Dynamic> jointBoundVels;
  jointBoundVels.resize(2, chainSize);
  jointBoundVels.setZero();
  jointBoundVels.row(1) = preImpJVel.transpose(); // Second row

  //LOG_INFO("Performing pre-impact trajectory optimization...")
  //! Pre-impact trajectory optimization
  Matrix<Scalar, Dynamic, 1> knots;
  knots.resize(3); // 3 poses for pre-impact/post-impact trajectory
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.2;
  auto cb1 = CubicSpline<Scalar>(
    chainSize,
    jointPos.block(0, 0, 4, chainSize),
    knots,
    this->cycleTime,
    jointBoundVels);
  cb1.setup();
  auto cbopt = CbOptimizer<Scalar>(this->motionModule, this->kickLeg, this->supportLeg, &cb1);
  cbopt.setZmpCons(true);
  cbopt.optDef();
  vector<Scalar> trajTime;
  cb1.evaluateSpline(jointTrajectories, trajTime, 0);
  
 //cout << "Joint points;" << endl;
  //cout << jointPos.block(0, 0, 3, chainSize) << endl;
  //cout << "Trajs" << endl;
  //for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
  //  for (size_t i = 0; i < jointTrajectories.size(); ++i) {
  //    cout << jointTrajectories[i][j] << " ";
  //  }
  //  cout << endl;
  //}
  //cb1.plotSpline(100, 0.0);
  kickTimeToImpact = trajTime.back();
  
  //! Constant velocity phase
  //cout << "this->cycleTime: " << this->cycleTime << endl;
  //Matrix<Scalar, Dynamic, Dynamic> postImpactJoint = jointPos.row(2) + preImpJVel.transpose() * this->cycleTime;
  Matrix<Scalar, Dynamic, Dynamic> prevjoints = jointPos.row(3).transpose();
  desCartesianVel.block(0, 0, 3, 1) = desImpactVel;
  Scalar distAfterImpact = 0.0;
  while(true) {
    this->kM->setJointPositions(chainStart, prevjoints, JointStateType::SIM); // impact pose joints
    Matrix<Scalar, 4, 4> eeTrans = MathsUtils::getTInverse(this->torsoToSupport) * this->kM->getForwardEffector(this->kickLeg, ANKLE, JointStateType::SIM) *  this->endEffector;
    eeTrans.block(0 , 3, 3, 1) = eeTrans.block(0, 3, 3, 1) + desCartesianVel.block(0, 0, 3, 1) * this->cycleTime;
    auto eeTransKick = this->torsoToSupport * eeTrans;
    Matrix<Scalar, Dynamic, 1> angles;
    angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, eeTransKick, 50, JointStateType::SIM, true, 5e-2, 0.5);
    if (angles[0] != angles[0]) {
        cout << "Requested kick cannot be performed." << endl;
        this->kickFailed = true;
        return;
    }
    for (size_t i = 0; i < jointTrajectories.size(); ++i)
      jointTrajectories[i].push_back(angles[i]);
    distAfterImpact += (eeTrans.block(0, 3, 3, 1) - this->impactPose.block(0, 3, 3, 1)).norm();
    //cout << "distAfterImpact: " << distAfterImpact << endl;
    if (distAfterImpact > 0.03)
      break;
    prevjoints = angles;
  //  cout << "i: " << i << endl;
    //Matrix<Scalar, 4, 4> pose = preImpactPose3;
    //pose(0,3) += diff * i;
    //pose = this->torsoToSupport * pose;
    //cout << "pose:\n" << endl;
    //cout << pose << endl;
    /*vector <Matrix<Scalar, Dynamic, 1>> angles;
    if (this->kickLeg == CHAIN_L_LEG) 
      angles = this->kM->inverseLeftLeg(this->endEffector, pose);
    else angles = 
      this->kM->inverseRightLeg(this->endEffector,pose);
    Matrix<Scalar, Dynamic, Dynamic> joints;
    if (angles.size() != 0) {
      joints = angles[0].transpose();
    } else {
      ERROR(
        "The required cartesian pose " << i << " is out of the configuration space of given chain.")
      return;
    }*/
    //Matrix<Scalar, Dynamic, 1> joints = prevjoints + prevjointsVel * this->cycleTime;
    /*if (!desImpactVelKnown) {
      computeDesImpactVel(prevjoints);
    }
    desCartesianVel.block(0, 0, 3, 1) = desImpactVel; // x-y-z velocity
    //! Cartesian velocities in torso frame.
    desCartesianVel.block(0, 0, 3, 1) =
      this->torsoToSupport.block(0, 0, 3, 3) * desCartesianVel.segment(0, 3);
    
    Matrix<Scalar, Dynamic, 1> jVel;
    this->kM->setJointPositions(chainStart, joints, JointStateType::SIM); // impact pose joints
    jacobian = this->kM->computeLimbJ(
      this->kickLeg,
      this->endEffector,
      JointStateType::SIM).block(0, 0, 3, 6);
    jVel = 
      MathsUtils::pseudoInverseSolve(
        jacobian,
        Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1))
      );
    for (size_t i = 0; i < jointTrajectories.size(); ++i)
      jointTrajectories[i].push_back(joints[i]);
    prevjoints = joints;*/
  }
  
  //cout << "end-effector: " << endl;  
  
  
  //! Constant momentum impact phase
  /*if (!desImpactVelKnown) {
		computeDesImpactVel(jointPos.row(3));  
  }
  desCartesianVel.block(0, 0, 3, 1) = desImpactVel; // x-y-z velocity
  //! Cartesian velocities in torso frame.
  desCartesianVel.block(0, 0, 3, 1) =
    this->torsoToSupport.block(0, 0, 3, 3) * desCartesianVel.segment(0, 3);
  
  Matrix<Scalar, Dynamic, 1> postImpJVel;
  this->kM->setJointPositions(chainStart, jointPos.row(3), JointStateType::SIM); // impact pose joints
  jacobian = this->kM->computeLimbJ(
    this->kickLeg,
    this->endEffector,
    JointStateType::SIM).block(0, 0, 3, 6);
  postImpJVel = 
    MathsUtils::pseudoInverseSolve(
      jacobian,
      Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1))
    );
  jointBoundVels.row(0) = preImpJVel.transpose();
  jointBoundVels.row(1) = postImpJVel.transpose();
  cout << "preImpJVel:\n" << preImpJVel << endl;
  cout << "postImpJVel:\n" << postImpJVel << endl;*/

  
  //! Second trajectory optimization
  /*LOG_INFO("Performing second trajectory optimization...")
  vector < vector<Scalar> > jointTrajectories2;
  jointBoundVels.setZero();
  jointBoundVels.row(0) = prevjointsVel.transpose(); // First row
  cout <<" prevjointsVel: " <<  prevjointsVel << endl;
  cout <<" prevjointsVel: " <<  jointBoundVels << endl;
  knots.resize(3); // 4 poses for post-impact trajectory
  Matrix<Scalar, Dynamic, Dynamic> requiredJoints;
  requiredJoints.resize(4, chainSize);
  requiredJoints.row(0) = prevjoints.transpose();
  requiredJoints.row(1) = jointPos.block(3, 0, 1, chainSize);
  requiredJoints.row(2) = jointPos.block(4, 0, 1, chainSize);
  requiredJoints.row(3) = jointPos.block(5, 0, 1, chainSize);
  cout << "requiredJoints:\n " << requiredJoints << endl;
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.2;
  auto cb2 = CubicSpline(
    chainSize,
    requiredJoints,
    knots,
    this->cycleTime,
    jointBoundVels);
  cb2.setup();
  Matrix<Scalar, Dynamic, 1> eeVelMax(3);
  eeVelMax = desCartesianVel.block(0, 0, 3, 1);
  cout << "eeVelMax: " << eeVelMax << endl;
  auto cbopt2 = CbOptimizer(this->motionModule, this->kickLeg, this->supportLeg, this->endEffector, eeVelMax, &cb2);
  cbopt2.optDef();
  cb2.evaluateSpline(jointTrajectories2, trajTime, 0);

  for (int i = 0; i < jointTrajectories.size(); ++i) {
    jointTrajectories[i].insert(
      jointTrajectories[i].end(),
      jointTrajectories2[i].begin(),
      jointTrajectories2[i].end());
  }
  for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
    Matrix<Scalar, Dynamic, 1> joints(chainSize);
    for (size_t i = 0; i < jointTrajectories.size(); ++i) {
      joints[i] = jointTrajectories[i][j];
    }
    this->kM->setJointPositions(chainStart, joints, JointStateType::SIM); // impact pose joints
    supportToKick = MathsUtils::getTInverse(this->torsoToSupport) * this->kM->getForwardEffector(this->kickLeg, ANKLE, JointStateType::SIM);
    auto eeTrans = supportToKick * this->endEffector;
    cout << eeTrans(0,3) << ", " << eeTrans(1,3) << ", " << eeTrans(2,3) << endl;
  } */
  if (this->logData) {
    Json::Value jsonPlanning, jsonTraj;
    JSON_ASSIGN(jsonPlanning, "torsoToSupport", JsonUtils::MatrixToJson(this->torsoToSupport));
    JSON_ASSIGN(jsonPlanning, "supportToKick", JsonUtils::MatrixToJson(this->supportToKick));
    JSON_ASSIGN(jsonPlanning, "endEffectorTransformed", JsonUtils::MatrixToJson(eeTrans));
    Json::Value jsoncPoses;
    Json::Value jsoncPosesT;
    for (int i = 0; i < cPoses.size(); ++i) {
      jsoncPoses.append(JsonUtils::MatrixToJson(cPoses[i]));
      jsoncPosesT.append(JsonUtils::MatrixToJson(cPosesT[i]));
    }
    JSON_ASSIGN(jsonPlanning, "cPoses", jsoncPoses);
    JSON_ASSIGN(jsonPlanning, "cPosesT", jsoncPosesT);
    JSON_ASSIGN(jsonTraj, "optknots", JsonUtils::MatrixToJson(cb1.getKnots()));
    JSON_ASSIGN(jsonTraj, "jointPoses", JsonUtils::MatrixToJson(jointPos));
    JSON_ASSIGN(jsonTraj, "jointBoundVels", JsonUtils::MatrixToJson(jointBoundVels));
    JSON_ASSIGN(jsonTraj, "kickTimeToImpact", kickTimeToImpact);
    
    Json::Value jsonEE;
    auto linkChain = this->kM->getLinkChain(this->kickLeg);
    unsigned chainStart = linkChain->start;
    unsigned chainSize = linkChain->size;
    //cout << "Setting joints" << endl;
    for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
      Matrix<Scalar, Dynamic, 1> joints(chainSize);
      for (size_t i = 0; i < jointTrajectories.size(); ++i) {
        joints[i] = jointTrajectories[i][j];
      }
      this->kM->setJointPositions(chainStart, joints, JointStateType::SIM); // impact pose joints
      //cout << joints.transpose() << endl;
      auto stA = 
        MathsUtils::getTInverse(this->torsoToSupport) * 
        this->kM->getForwardEffector(
          this->kickLeg, ANKLE, JointStateType::SIM);
      auto eeTrans = stA * this->endEffector;
      JSON_APPEND(
        jsonEE, 
        "endEffectorCmd", 
        JsonUtils::MatrixToJson(eeTrans)
      );
    }
    JSON_ASSIGN(this->dataLogger->getRoot(), "planning", jsonPlanning);
    JSON_ASSIGN(this->dataLogger->getRoot(), "trajectory", jsonTraj);
    JSON_ASSIGN(this->dataLogger->getRoot(), "endEffector", jsonEE);
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::requestExecution()
{
  auto linkChain = this->kM->getLinkChain(this->kickLeg);
  unsigned chainStart = linkChain->start;
  unsigned chainSize = linkChain->size;
  vector<unsigned> jointIds;
  for (size_t i = chainStart; i < chainStart + chainSize; ++i)
    jointIds.push_back(i);
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(chainSize);
  jointPositions.arraySetSize(chainSize);
  unsigned trajStep = 0;
  while (true) {
    for (int i = 0; i < chainSize; ++i) {
      jointPositions[i].arrayPush(jointTrajectories[i][trajStep]);
      jointTimes[i].arrayPush((trajStep + 1) * this->cycleTime);
    }
    ++trajStep;
    if (trajStep == jointTrajectories[0].size()) break;
  }

  //cout << "jointPositions: " << jointPositions << endl;
  //cout << "jointTimes: " << jointTimes << endl;
  this->totalTimeToKick = (trajStep + 1) * this->cycleTime;
  //cout << "totaltime : " << totalTimeToKick << endl;
  this->kickTimeStep = 0.f;
  this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  //plotJointTrajectories();
}

template <typename Scalar>
void JointSpaceKick<Scalar>::logEndEffectorActual()
{
  Json::Value& root = this->dataLogger->getRoot();
  Matrix<Scalar, 4, 4> supportToEE = 
    MathsUtils::getTInverse(this->torsoToSupport) * 
    this->kM->getForwardEffector(this->kickLeg, ANKLE) * 
    this->endEffector;
  JSON_APPEND(
    root["endEffector"], 
    "endEffectorActual", 
    JsonUtils::MatrixToJson(supportToEE)
  );
  //comState = this->kM->getComStateWrtFrame(this->supportLeg, FEET_BASE);
  //cout << "position: " <<comState.position.transpose() << endl;
  //cout << "velocity: " << comState.velocity.transpose() << endl;
  //cout << "accel: " << comState.accel.transpose() << endl;
}

template <typename Scalar>
void JointSpaceKick<Scalar>::plotKick()
{
  PlotEnv<Scalar>
    plotEnv(
      "JointSpaceKick", "x", "y", "z", 
      Matrix<Scalar, 2, 1>(-0.05, 0.1),
      Matrix<Scalar, 2, 1>(-.5, .5),
      Matrix<Scalar, 2, 1>(-.5, .5)
    );
  
  LOG_INFO("Plotting kick parameters...")
  //cout << "ballPosition: " << this->ballPosition << endl;
  Scalar centerSpacing = fabsf(this->supportToKick(1, 3) / 2);
  Scalar offset = this->supportLeg == CHAIN_L_LEG ? centerSpacing : -centerSpacing;
  //cout << "offset: " << offset << endl;
  //cout << "ballPosition-offset: " << this->ballPosition[1] - offset << endl;

  plotEnv.setSphere(Matrix<Scalar, 3, 1>(-(this->ballPosition[1] + offset), this->ballPosition[0], this->ballRadius), this->ballRadius);

  auto eeTrans = this->supportToKick * this->endEffector;
  for (int i = 0; i < cPoses.size(); ++i) {
    cout << cPoses[i] << endl;
    plotEnv.plotPoint(Matrix<Scalar, 3, 1>(-(cPoses[i](1, 3) + offset), cPoses[i](0, 3), cPoses[i](2, 3) + footHeight));
  }
  std::ostringstream cmdstr;
  string contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveLeftXY.txt");
  cmdstr << "replot '" << contourLog << "' using 2:1:3 with lines title '' lc rgb 'red'";
  plotEnv.setCmd(cmdstr);
  cmdstr.str(std::string());
  
  contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveRightXY.txt");
  cmdstr << "replot '" << contourLog << "' using 2:1:3 with lines title '' lc rgb 'red'";
  plotEnv.setCmd(cmdstr);
  cmdstr.str(std::string());
  if (this->supportLeg == CHAIN_R_LEG) {
    plotEnv.setFrame(
      Matrix<Scalar, 3, 1>(centerSpacing, 0.0, 0.0), 
      Matrix<Scalar, 3, 1>(0.0, 0.0, 0.0),
      0.05
    );
  } else {
    plotEnv.setFrame(
      Matrix<Scalar, 3, 1>(-centerSpacing, 0.0, 0.0), 
      Matrix<Scalar, 3, 1>(0.0, 0.0, 0.0),
      0.05
    );
  }
  vector<Scalar> eeX;
  vector<Scalar> eeY;
  vector<Scalar> eeZ;
  auto linkChain = this->kM->getLinkChain(this->kickLeg);
  unsigned chainStart = linkChain->start;
  unsigned chainSize = linkChain->size;
  //cout << "Setting joints" << endl;
  for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
    Matrix<Scalar, Dynamic, 1> joints(chainSize);
    for (size_t i = 0; i < jointTrajectories.size(); ++i) {
      joints[i] = jointTrajectories[i][j];
    }
    this->kM->setJointPositions(chainStart, joints, JointStateType::SIM); // impact pose joints
    //cout << joints.transpose() << endl;
    auto stA = MathsUtils::getTInverse(this->torsoToSupport) * this->kM->getForwardEffector(this->kickLeg, ANKLE, JointStateType::SIM);
    auto eeTrans = stA * this->endEffector;
    eeX.push_back(-(eeTrans(1,3) + offset)); // Plotting env has x = -y
    eeY.push_back(eeTrans(0,3)); // Plotting env has y = x
    eeZ.push_back(eeTrans(2,3) + footHeight);
    cout << "ee: " << " " << -(eeTrans(1,3) + offset) << " " << eeTrans(0,3) + footHeight << endl;
  }
  plotEnv.plot3D(eeX, eeY, eeZ);
}

template <typename Scalar>
void JointSpaceKick<Scalar>::plotEETrajectory()
{

}


template class JointSpaceKick<MType>;
/*void
JointSpaceKick<Scalar>::plotKick()
{
  LOG_INFO("Plotting kick parameters...")
  setupGnuPlotConfig();
  cout << "ballPosition: " << ballPosition << endl;
  Scalar centerSpacing = fabsf(supportToKick(1, 3) / 2);
  Scalar offset = this->supportLeg == CHAIN_L_LEG ? centerSpacing : -centerSpacing;
  cout << "offset: " << offset << endl;
  cout << "ballPosition-offset: " << ballPosition[1] - offset << endl;
  auto eeTrans = supportToKick * this->endEffector;
  for (int i = 0; i < cPoses.size(); ++i)
    plotPoint(-(cPoses[i](1, 3) + offset), cPoses[i](0, 3), "", 1);

  gp << "set object 1 circle at " << -(ballPosition[1] + offset) << "," << ballPosition[0] << " size 0.05 fs transparent solid 0.35 fc rgb 'red'\n";
  string contourLog = ConfigManager::getLogsDirPath() + string("Kicthis->kModule/FootCurveLeftXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";
  contourLog = ConfigManager::getLogsDirPath() + string("Kicthis->kModule/FootCurveRightXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";

  if (this->supportLeg == CHAIN_R_LEG) {
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing - 0.025 << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  } else {
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -(centerSpacing + 0.025) << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  }
  gp << "replot\n";
  cin.get();
}

void
JointSpaceKick<Scalar>::plotJointTrajectories()
{
  Gnuplot gp;
  vector < pair<Scalar, Scalar> > times_pos;
  gp << "set xrange [0:20]\nset yrange [0:20]\n";
  gp << "plot" << gp.file1d(times_pos) << "with lines title 'Joint Trajectories'" << endl;
  int chainSize = this->kM->getChainSize(this->kickLeg);
  for (int i = 1; i < chainSize; ++i) {
    int trajStep = 0;
    while (true) {
      gp << "set terminal wxt " << i << endl;
      Scalar pos = jointTrajectories[i][trajStep];
      Scalar t = (trajStep + 1) * this->cycleTime;
      times_pos.push_back(make_pair(t, pos));
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }
    gp << "plot" << gp.file1d(times_pos) << "with lines title 'joint Trajectories " << i << " position.'" << endl;
    cin.get();
  }
}
*/
