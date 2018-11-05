/**
 * @file MotionModule/BallThrow/Types/WBBallThrow.h
 *
 * This file implements the class WBBallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/BallThrow/Types/WBBallThrow.h"
#include "Utils/include/CubicSpline.h"

template <typename Scalar>
void WBBallThrow<Scalar>::initiate()
{
  LOG_INFO("WBBallThrow.initiate()")
  this->timeToThrow = this->getBehaviorCast()->timeToThrow;
  if (false) {//this->getBehaviorCast()->headTapToStart) {
    this->behaviorState = this->waitForHeadTap;
  } else {
    this->behaviorState = this->grabBall;
  }
  this->inBehavior = true;
}

template <typename Scalar>
void WBBallThrow<Scalar>::update()
{
  if (this->behaviorState == this->waitForHeadTap) {
    this->waitForHeadTapAction();
  } else if (this->behaviorState == this->grabBall) {
    grabBallAction();
  } else if (this->behaviorState == this->retract) {
    retractAction();
  } else if (this->behaviorState == this->throwBall) {
    throwBallAction();
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::executeArmsTrajs(
  const vector<vector<Scalar> >& jointTrajectories,
  const Scalar& stepTime) 
{
  auto size = jointTrajectories.size() * 2;
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  auto totalSteps = jointTrajectories[0].size();
  vector<unsigned> jointIds;
  for (int i = 0; i < size; ++i) {
    jointIds.push_back(L_SHOULDER_PITCH + i);
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  for (int j = 0; j < totalSteps; ++j) {
    for (int i = 0; i < size; ++i) {
      if (i >= 5) {
        if (i >= 6) jointPositions[i][j] = -jointTrajectories[i - 5][j];
        else jointPositions[i][j] = jointTrajectories[i - 5][j];
      } else {
        jointPositions[i][j] = jointTrajectories[i][j];
      }
      jointTimes[i][j] = (j + 1) * stepTime;
    }
  }
  this->execTime = 0.0;
  this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
}

template <typename Scalar>
void WBBallThrow<Scalar>::grabBallAction() {
  static bool executed = false;
  static Scalar timeToGrab = 1.0;
  if (!executed) {
    this->openHand(L_HAND);
    this->openHand(R_HAND);
    Scalar circleMidTheta = 25.0 * M_PI / 180.0;
    unsigned chainSize = this->kM->getLinkChain(CHAIN_L_ARM)->size;
    vector<vector<Scalar> > jointTrajectories;
    jointTrajectories.resize(chainSize);
    for (int i = 0; i < jointTrajectories.size(); ++i)
      jointTrajectories[i].resize(1);
    jointTrajectories[0][0] =  60.0 * M_PI / 180.0 - circleMidTheta;
    jointTrajectories[1][0] =  0.0 * M_PI / 180.0, 
    jointTrajectories[2][0] = -55.0 * M_PI / 180.0, 
    jointTrajectories[3][0] = -55.0 * M_PI / 180.0, 
    jointTrajectories[4][0] = 0.0 * M_PI / 180.0;
    executeArmsTrajs(jointTrajectories, timeToGrab);
    executed = true;
  } else {
    if (this->execTime > timeToGrab + 1.0 + this->cycleTime / 2) {
      this->behaviorState = this->retract;
    } else {
      this->execTime += this->cycleTime;
    }
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::retractAction() {
  static bool executed = false;
  static Scalar timeToRetract = 2.0;
  if (!executed) {
    unsigned chainStart = this->kM->getLinkChain(CHAIN_L_ARM)->start;
    unsigned chainSize = this->kM->getLinkChain(CHAIN_L_ARM)->size;
    unsigned nPoses = 2;
    Matrix<Scalar, Dynamic, 1> knots;
    Matrix<Scalar, Dynamic, Dynamic> desiredJoints;
    Matrix<Scalar, Dynamic, Dynamic> desiredBoundVels;
    desiredJoints.resize(nPoses, chainSize);
    desiredBoundVels.resize(2, chainSize);
    desiredBoundVels.setZero();
    for (size_t i = 0; i < chainSize; ++i) {
      for (size_t j = 0; j < nPoses; ++j) {
        desiredJoints(j, i) = this->kM->getJointState(chainStart + i)->position;
      }
    }
    desiredJoints(1, 0) = -118.0 * M_PI / 180.0;
    knots.resize(desiredJoints.rows() - 1);
    for (int i = 0; i < knots.size(); ++i)
      knots[i] = timeToRetract;
    vector < vector<Scalar> > jointTrajectories;
    auto cb1 = 
      CubicSpline<Scalar>(
        chainSize,
        desiredJoints,
        knots,
        this->cycleTime,
        desiredBoundVels
      );  
    cb1.setup();
    vector<Scalar> trajTime;
    cb1.evaluateSpline(jointTrajectories, trajTime, 0);
    cout << "calling retract execute" << endl;
    executeArmsTrajs(jointTrajectories, this->cycleTime);
    executed = true;
  } else {
    cout << "retractExec" << endl;
    cout << "timeToRetract: " << timeToRetract << endl;
    cout << "this->execTime: " << this->execTime << endl;
    if (this->execTime > timeToRetract + 1.0 + this->cycleTime / 2) {
      this->behaviorState = this->throwBall;
    } else {
      this->execTime += this->cycleTime;
    }
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::throwBallAction() {
    auto torsoState = this->kM->getTorsoState();
  auto lArmChain = this->kM->getLinkChain(CHAIN_L_ARM);
  unsigned chainStart = lArmChain->start;
  unsigned chainSize = lArmChain->size;
  // Equation of a circle
  auto torsoToEE = this->kM->getForwardEffector(CHAIN_L_ARM, 0);
  auto armToEE = MathsUtils::getTInverse(lArmChain->startT) * torsoToEE;
  auto armToEERotated = torsoState->rot * armToEE;
  //cout << "torsoToEE: " << torsoToEE << endl;
  //cout << "rotated torsoToEE: " << torsoState->rot * torsoToEE << endl;  
  //cout << "armToEE: " << armToEE << endl;
  auto radius = armToEERotated.block(0, 3, 3, 1).norm();
  cout << "radius: " << radius << endl;
  //cout << "torsoState->rot:\n" << torsoState->rot << endl;
  auto eulers = MathsUtils::matToEuler(Matrix<Scalar, 3, 3>(armToEERotated.block(0, 0, 3, 3)));
  
  auto angle = atan2(armToEERotated(0, 3), armToEERotated(2, 3));
  cout << "angle: " << angle * 180/ M_PI << endl;
  Matrix<Scalar, 4, 4> finalRot;
  MathsUtils::makeRotationY(finalRot, -angle);
  finalRot = finalRot * armToEERotated;
  
  cout << "armToEERotated :" << armToEERotated << endl;
  cout << "finalRot :" << finalRot << endl;
  
  cout << "euler angles: " << eulers * 180/ M_PI << endl;
  auto eulersFinal = MathsUtils::matToEuler(Matrix<Scalar, 3, 3>(finalRot.block(0, 0, 3, 3)));
  cout << "euler angles fnal: " << eulersFinal * 180/ M_PI << endl;
  
  return;
  
  static bool executed = false;
  static Scalar timeToThrow;
  if (!executed) {
    /*auto lArmChain = this->kM->getLinkChain(CHAIN_L_ARM);
    unsigned chainStart = lArmChain->start;
    unsigned chainSize = lArmChain->size;
    // Equation of a circle
    torsoToEE = kM->getForwardEffector(supportLeg, 0);
    armToEE = MathsUtils::getTInverse(lArmChain->startT)) * kM->getForwardEffector(supportLeg, 0);
    cout << "torsoToEE: " << torsoToEE << endl;
    cout << "armToEE: " << armToEE << endl;
    
    
    (x - h)^2 + (y - k)^2 = r^2
    
    
    for (size_t i = 0; i < chainSize; ++i) {
      for (size_t j = 0; j < nPoses; ++j) {
        desiredJoints(j, i) = this->kM->getJointState(chainStart + i)->position;
      }
    }
    desiredJoints(1, 0) = 25 * M_PI / 180.0 - 75.0 * M_PI / 180.0;
    knots.resize(desiredJoints.rows() - 1);
    for (int i = 0; i < knots.size(); ++i)
      knots[i] = 0.5;
    vector < vector<Scalar> > jointTrajectories;
    auto cb1 = 
      CubicSpline<Scalar>(
        chainSize,
        desiredJoints,
        knots,
        this->cycleTime,
        desiredBoundVels
      );  
    cb1.setup();
    vector<Scalar> trajTime;
    cb1.evaluateSpline(jointTrajectories, trajTime, 0);
    timeToThrow = trajTime.back();
    cout << "calling throw execute" << endl;
    executeArmsTrajs(jointTrajectories, this->cycleTime);
    executed = true;*/
  } else {
    /*cout << "throwExec" << endl;
    cout << "timeToThrow: " << timeToThrow << endl;
    cout << "this->execTime: " << this->execTime << endl;
    if (this->execTime > timeToThrow + this->cycleTime / 2) {
      finish();
    } else {
      this->execTime += this->cycleTime;
    }*/
  }
}
/* Matrix<Scalar, 6, 1> comVelocityD = Matrix<Scalar, 6, 1>::Zero();
comVelocityD[0] = 0.0; //(error[0] * 1.f + intError[0] * 0.001) / this->cycleTime; //PI control
comVelocityD[1] = updatedCom[4]; //(error[1] * 0.0015f + intError[1] * 0.001) / this->cycleTime; //PI control
//comVelocityD[4] = rollError;
intError = intError + error;
vector<unsigned> limbMotionSpace;
limbMotionSpace.push_back(0);
limbMotionSpace.push_back(0);
limbMotionSpace.push_back(0);
limbMotionSpace.push_back(1);
limbMotionSpace.push_back(1);
vector<Matrix<Scalar, Dynamic, 1>> limbVelocitiesD;
limbVelocitiesD.push_back(Matrix<Scalar, 2, 1>::Zero());
limbVelocitiesD.push_back(Matrix<Scalar, 5, 1>::Zero());
limbVelocitiesD.push_back(Matrix<Scalar, 5, 1>::Zero());
limbVelocitiesD.push_back(Matrix<Scalar, 6, 1>::Zero());
limbVelocitiesD.push_back(Matrix<Scalar, 6, 1>::Zero());
//limbVelocitiesD[4][2] = 0.001;
vector<int> eeIndices;
eeIndices.push_back(0);
eeIndices.push_back(0);
eeIndices.push_back(0);
eeIndices.push_back(FEET_BASE);
eeIndices.push_back(FEET_BASE);
high_resolution_clock::time_point tEnd1 = high_resolution_clock::now();
duration<double> time_span1 = tEnd1 - tStart;
LOG_INFO(
  "RobotExtraction time1: " + DataUtils::varToString(time_span1.count()) + " seconds.");
Matrix<Scalar, Dynamic, 1> jointsD = this->this->kM->solveComIK(
  KinematicsModule::SIM,
  supportLeg,
  comVelocityD,
  limbMotionSpace,
  limbVelocitiesD,
  eeIndices);
high_resolution_clock::time_point tEnd2 = high_resolution_clock::now();
duration<double> time_span2 = tEnd2 - tEnd1;
LOG_INFO(
  "RobotExtraction time2: " + DataUtils::varToString(time_span2.count()) + " seconds.");
runTime = runTime + this->cycleTime;
for (int i = 0; i < NUM_JOINTS; ++i) {
  jointPositions[i].arrayPush(jointsD[i]);
  jointTimes[i].arrayPush(runTime);
}*/

template <typename Scalar>
void WBBallThrow<Scalar>::finish()
{
  this->inBehavior = false;
}

template class WBBallThrow<MType>;
