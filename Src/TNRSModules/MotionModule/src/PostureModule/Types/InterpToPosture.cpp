/**
 * @file MotionModule/src/PostureModule/Type/InterpToPosture.cpp
 *
 * This file implements the class InterpToPosture
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/PostureModule/Types/InterpToPosture.h"

void
InterpToPosture::initiate()
{
  PRINT("InterpToPosture.initiate()")
  jointsToReach = getBehaviorCast()->jointsToReach * M_PI / 180;
  timeToReachP = getBehaviorCast()->timeToReachP;
  targetPosture = getBehaviorCast()->targetPosture;
  VectorXf joints = 
    kM->getJointPositions(0, NUM_JOINTS);
  VectorXf jointsDelta = jointsToReach - joints;
  vector<unsigned> jointIds;
  for (int i = 0; i < jointsDelta.size(); ++i) {
    if (abs(jointsDelta[i]) < 0.0174533) // 1 degree
    continue;
    jointIds.push_back(i);
  }
  int size = jointIds.size();

  if (size == 0) {
    inBehavior = false;
    return;
  }

  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  int totalSteps = ceil(timeToReachP / cycleTime);
  //cout << "totalSteps: " << totalSteps << endl;
  //cout << "totalSteps: " << floor(totalSteps) << endl;
  for (int i = 0; i < size; ++i) {
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  float step = cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    float timeParam = step / timeToReachP;
    float multiplier =
      6 * pow(timeParam, 5) - 15 * pow(timeParam, 4) + 10 * pow(timeParam, 3);
    for (int i = 0; i < size; ++i) {
      int jointId = jointIds[i];
      jointPositions[i][j] =
        joints[jointId] + jointsDelta[jointId] * multiplier;
      jointTimes[i][j] = step;
    }
    step += cycleTime;
  }
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  execTime = 0.f;
  inBehavior = true;
}

void
InterpToPosture::update()
{
  //PRINT("InterpToPosture.update()")
  //PRINT("InterpToPosture.runTime: " << runTime)
  //PRINT("InterpToPosture.execTime: " << execTime)
  //PRINT("InterpToPosture.timeToReachP: " << timeToReachP)
  if (execTime > timeToReachP + cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = targetPosture;
    finish();
  } else {
    execTime += cycleTime;
  }
}

void
InterpToPosture::finish()
{
  PRINT("InterpToPosture.finish()")
  motionProxy->killAll();
  inBehavior = false;
}
