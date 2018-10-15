/**
 * @file SBModule/src/StiffnessModule/Types/StiffnessInterp.cpp
 *
 * This file implements the class StiffnessInterp
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "SBModule/include/StiffnessModule/Types/StiffnessInterp.h"

void
StiffnessInterp::initiate()
{
  PRINT("StiffnessInterp.initiate()")
  sToReach = getBehaviorCast()->sToReach;
  timeToReachS = getBehaviorCast()->timeToReachS;
  targetState = getBehaviorCast()->targetState;
  
  auto stiffnessesI = vector<float>(NUM_JOINTS, NAN);
  auto stiffnessesDelta = vector<float>(NUM_JOINTS, NAN);
  vector<unsigned> jointIds;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (sToReach[i] != sToReach[i]) continue;
    stiffnessesI[i] = 
      IVAR(vector<float>, SBModule::jointStiffnessSensors)[i];
    float diff = sToReach[i] - stiffnessesI[i];
    if (abs(diff) < 0.005) continue;
    stiffnessesDelta[i] = diff;
    jointIds.push_back(i);
  }
  size_t size = jointIds.size();
  if (size == 0) {
    OVAR(StiffnessState, SBModule::stiffnessState) = targetState;
    inBehavior = false;
    return;
  }
  AL::ALValue jointTimes;
  AL::ALValue jointStiffnesses;
  jointTimes.clear();
  jointStiffnesses.clear();
  jointTimes.arraySetSize(size);
  jointStiffnesses.arraySetSize(size);
  int totalSteps = timeToReachS / cycleTime;
  for (int i = 0; i < size; ++i) {
    jointStiffnesses[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  float timeStep = cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    float timeParam = timeStep / timeToReachS;
    for (int i = 0; i < size; ++i) {
      int jointId = jointIds[i];
      float cStiffness =
        stiffnessesI[jointId] + stiffnessesDelta[jointId] * timeParam;
      cStiffness = cStiffness >= 0.99f ? 1.0f : cStiffness;
      cStiffness = cStiffness <= 0.01f ? 0.0f : cStiffness;
      jointStiffnesses[i][j] = cStiffness;
      jointTimes[i][j] = timeStep;
    }
    timeStep += cycleTime;
  }
  naoqiStiffnessInterpolation(jointIds, jointTimes, jointStiffnesses, true);
  execTime = 0.f;
  inBehavior = true;
}

void
StiffnessInterp::update()
{
  //PRINT("StiffnessInterp.update()")
  //cout << "execTime: " <<execTime << endl;
  if (execTime > timeToReachS + cycleTime / 2) {
    OVAR(StiffnessState, SBModule::stiffnessState) = targetState;
    finish();
  } else {
    execTime += cycleTime;
  }
}

void
StiffnessInterp::finish()
{
  PRINT("StiffnessInterp.finish()")
  inBehavior = false;
}
