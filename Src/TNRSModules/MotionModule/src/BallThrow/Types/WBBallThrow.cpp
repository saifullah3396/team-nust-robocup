/**
 * @file MotionModule/BallThrow/Types/WBBallThrow.h
 *
 * This file implements the class WBBallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/BallThrow/Types/WBBallThrow.h"

void
WBBallThrow::initiate()
{
  PRINT("WBBallThrow.initiate()")
  ERROR("No behavior definition found.")
  timeToThrow = getBehaviorCast()->timeToThrow;
  waitForHeadTap = getBehaviorCast()->headTapToStart;
  inBehavior = false;
  /*unsigned chainSize = kM->getChainSize(CHAIN_L_ARM);
  float circleMidTheta = 25 * M_PI / 180;
  motionProxy->post.openHand("LHand");
  motionProxy->openHand("RHand");
  int size = 10;
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  VectorXf knots;
  VectorXf startState;
  startState.resize(chainSize);
  startState << circleMidTheta, 0 * M_PI / 180, -55 * M_PI / 180, -55 * M_PI / 180, -35 * M_PI / 180;
  int totalSteps = 1;
  vector<unsigned> jointIds;
  for (int i = 0; i < size; ++i) {
    jointIds.push_back(L_SHOULDER_PITCH + i);
    jointPositions[i].arraySetSize(1);
    jointTimes[i].arraySetSize(1);
  }
  for (int j = 0; j < 1; ++j) {
    for (int i = 0; i < size; ++i) {
      if (i >= 5) {
        if (i >= 6) jointPositions[i][j] = -startState[i - 5];
        else jointPositions[i][j] = 60 * M_PI / 180 - startState[i - 5];
      } else {
        if (i == 0) {
          jointPositions[i][j] = 60 * M_PI / 180 - startState[i];
        } else {
          jointPositions[i][j] = startState[i];
        }
      }
      jointTimes[i][j] = 1;
    }
  }
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  sleep(5);
  //inBehavior = false;
  //return;
  cout << "done: " << endl;
  for (int j = 0; j < 1; ++j) {
    for (int i = 0; i < size; ++i) {
      if (i >= 5) {
        if (i >= 6) jointPositions[i][j] = -startState[i - 5];
        else jointPositions[i][j] = -118 * M_PI / 180;
      } else {
        if (i == 0) {
          jointPositions[i][j] = -118 * M_PI / 180;
        } else {
          jointPositions[i][j] = startState[i];
        }
      }
      jointTimes[i][j] = 2;
    }
  }
  cout << "done: " << endl;
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  unsigned nPoses = 2;
  MatrixXf desiredJoints;
  MatrixXf desiredBoundVels;
  desiredJoints.resize(nPoses, chainSize);
  desiredBoundVels.resize(2, chainSize);
  desiredBoundVels.setZero();
  desiredJoints.block(0, 0, 1, chainSize) = startState.transpose();
  desiredJoints.block(1, 0, 1, chainSize) = startState.transpose();
  desiredJoints(0, 0) = -118 * M_PI / 180;
  desiredJoints(1, 0) = desiredJoints(1, 0) - 75 * M_PI / 180;
  desiredJoints(0, 1) = 5 * M_PI / 180;
  knots.resize(desiredJoints.rows() - 1);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.5f;
  vector < vector<float> > jointTrajectories;
  trajectoryPlanner->jointsPlanner(
    jointTrajectories,
    CHAIN_L_ARM,
    desiredJoints,
    desiredBoundVels,
    knots);
  float totalTime = 1.f;
  totalSteps = jointTrajectories[0].size();
  for (int i = 0; i < size; ++i) {
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
      jointTimes[i][j] = (j + 1) * cycleTime;
    }
  }
  cout << "jointPositions:" << jointPositions << endl;
  cout << "jointTimes:" << jointTimes << endl;
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  this->inBehavior = true;
  return;*/
  /*unsigned nPoses = 4;
   unsigned chainSize = kM->getChainSize(CHAIN_L_ARM);
   float circleMidTheta = 17.5 * M_PI / 180;
   MatrixXf desiredJoints;
   MatrixXf desiredBoundVels;
   desiredJoints.resize(nPoses, chainSize);
   desiredBoundVels.resize(2, chainSize);
   desiredBoundVels.setZero();
   cout << kM->getJointPositions(
   KinematicsModule::ACTUAL,
   kM->getChainStart(CHAIN_L_ARM),
   kM->getChainSize(CHAIN_L_ARM)
   ) << endl;
   desiredJoints.block(0, 0, 1, chainSize) =
   kM->getJointPositions(
   KinematicsModule::ACTUAL,
   kM->getChainStart(CHAIN_L_ARM),
   chainSize
   ).transpose();
   VectorXf startState;
   startState.resize(chainSize);
   startState << circleMidTheta, 5 * M_PI / 180, -55 * M_PI / 180, -55 * M_PI / 180, -35 * M_PI / 180;
   for (int i = 1; i < desiredJoints.rows(); ++i) {
   desiredJoints.block(i, 0, 1, chainSize) = startState.transpose();
   }
   desiredJoints(1, 0) = 60 * M_PI / 180 - desiredJoints(1, 0);
   desiredJoints(2, 0) = -118 * M_PI / 180;
   desiredJoints(3, 0) = desiredJoints(3, 0) -90 * M_PI / 180;
   //desiredJoints(4, 0) += (-118) * M_PI / 180;
   cout << "desiredJoints:" << endl;
   cout << desiredJoints * 180 / M_PI << endl;
   vector<vector<float> > jointTrajectories;
   trajectoryPlanner->jointsPlanner(jointTrajectories, CHAIN_L_ARM, desiredJoints, desiredBoundVels);

   cout << "jointTrajectories[0].size(): " << jointTrajectories[0].size() << endl;

   int size = 5;
   AL::ALValue jointTimes;
   AL::ALValue jointPositions;
   jointTimes.clear();
   jointPositions.clear();
   jointTimes.arraySetSize(size);
   jointPositions.arraySetSize(size);
   float totalTime = 3.5f;
   int totalSteps = jointTrajectories[0].size();
   vector<unsigned> jointIds;
   for(int i = 0; i < size; ++i)
   {
   jointIds.push_back(L_SHOULDER_PITCH + i);
   jointPositions[i].arraySetSize(totalSteps);
   jointTimes[i].arraySetSize(totalSteps);
   }
   for (int j = 0; j < totalSteps; ++j) {
   cout << "j: " << j << endl;
   for (int i = 0; i < size; ++i) {
   cout << "i: " << i << endl;
   if (i >= 5) {
   jointPositions[i][j] = jointTrajectories[i - 5][j];
   } else {
   jointPositions[i][j] = jointTrajectories[i][j];
   }
   jointTimes[i][j] = (j + 1) * cycleTime;
   }
   }
   cout << "jointPositions: "  << jointPositions << endl;
   cout << "jointTimes: "  << jointTimes << endl;
   motionProxy->post.openHand("LHand");
   motionProxy->openHand("RHand");
   naoqiJointInterpolation(jointIds, jointTimes, jointPositions);
   this->inBehavior = true;*/
}

void
WBBallThrow::update()
{
  PRINT("WBBallThrow.update()")
  if(!headTapCheck())
    return;
}

void
WBBallThrow::finish()
{
  inBehavior = false;
}
