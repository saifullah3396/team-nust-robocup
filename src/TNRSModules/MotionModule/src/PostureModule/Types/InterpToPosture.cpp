/**
 * @file MotionModule/src/PostureModule/Type/InterpToPosture.cpp
 *
 * This file implements the class InterpToPosture
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/PostureModule/Types/InterpToPosture.h"
#include "MotionModule/include/KinematicsModule/Joint.h"

template <typename Scalar>
void InterpToPosture<Scalar>::initiate()
{
  LOG_INFO("InterpToPosture.initiate()")
  this->jointsToReach = this->getBehaviorCast()->jointsToReach.template cast<Scalar>() * M_PI / 180;
  this->timeToReachP = this->getBehaviorCast()->timeToReachP;
  this->targetPosture = this->getBehaviorCast()->targetPosture;
  LOG_INFO("InterpToPosture.initiate() : " + DataUtils::varToString((int)this->targetPosture))
  auto jointStates = this->kM->getJointStates(0, NUM_JOINTS);
  Matrix<Scalar, Dynamic, 1> jointsDelta(NUM_JOINTS);
  vector<unsigned> jointIds;
  for (int i = 0; i < jointStates.size(); ++i) {
    jointsDelta[i] = this->jointsToReach[i] - jointStates[i]->position;
    if (abs(jointsDelta[i]) < 0.0174533) // 1 degree
      continue;
    jointIds.push_back(i);
  }
  int size = jointIds.size();

  if (size == 0) {
    OVAR(PostureState, MotionModule::postureState) = 
      this->targetPosture;
    this->inBehavior = false;
    return;
  }

  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  int totalSteps = ceil(this->timeToReachP / this->cycleTime);
  //cout << "totalSteps: " << totalSteps << endl;
  //cout << "totalSteps: " << floor(totalSteps) << endl;
  for (int i = 0; i < size; ++i) {
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  float step = this->cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    float timeParam = step / this->timeToReachP;
    float multiplier =
      6 * pow(timeParam, 5) - 15 * pow(timeParam, 4) + 10 * pow(timeParam, 3);
    for (int i = 0; i < size; ++i) {
      int jointId = jointIds[i];
      jointPositions[i][j] =
        jointStates[jointId]->position + jointsDelta[jointId] * multiplier;
      jointTimes[i][j] = step;
    }
    step += this->cycleTime;
  }
  this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  this->execTime = 0.f;
  this->inBehavior = true;
}

template <typename Scalar>
void InterpToPosture<Scalar>::update()
{
  LOG_INFO("InterpToPosture.update()")
  LOG_INFO("InterpToPosture.runTime: " + DataUtils::varToString(this->runTime))
  LOG_INFO("InterpToPosture.execTime: " + DataUtils::varToString(this->execTime))
  LOG_INFO("InterpToPosture.timeToReachP: " + DataUtils::varToString(this->timeToReachP))
  LOG_INFO("InterpToPosture.targetPosture() : " + DataUtils::varToString((int)this->targetPosture))
  if (this->execTime > this->timeToReachP + this->cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = 
      this->targetPosture;
    finish();
  } else {
    this->execTime += this->cycleTime;
  }
}

template <typename Scalar>
void InterpToPosture<Scalar>::finish()
{
  LOG_INFO("InterpToPosture.finish()")
  this->killAllMotions();
  this->inBehavior = false;
}

template class InterpToPosture<MType>;
