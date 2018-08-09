/**
 * @file MotionModule/BalanceModule/Types/MPComControl.cpp
 *
 * This file implements the class MPComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/Types/MPComControl.h"

MPComControlConfigPtr MPComControl::getBehaviorCast()
{
  return boost::static_pointer_cast <MPComControlConfig> (config);
}

void
MPComControl::initiate()
{
  //PRINT("MPComControl.initiate()")
  #ifdef LOG_DATA
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("MPComControl/Com.txt")).c_str(), 
    std::ofstream::out | std::ofstream::trunc
  );
  comLog.close();
  #endif
  supportLeg = getBehaviorCast()->supportLeg;
  timeToReachB = getBehaviorCast()->timeToReachB;
  //cout << "supportLeg: " << supportLeg << endl;
  //cout << "timeToReachB: " << timeToReachB << endl;
  balanceMotionPrimitive();
}

void
MPComControl::update()
{
  //PRINT("MPComControl.update()")
  //PRINT("MPComControl.execTime: " << execTime)
  //PRINT("MPComControl.timeToReachB: " << timeToReachB)
  if (execTime > timeToReachB + cycleTime / 2) {
    finish();
  } else {
    execTime += cycleTime;
  }
}

void
MPComControl::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}

void
MPComControl::balanceMotionPrimitive()
{
  auto joints = 
    kM->getJointPositions(0, NUM_JOINTS);
  VectorXf jointsF(NUM_JOINTS);
  if (supportLeg == CHAIN_L_LEG) {
    jointsF = VectorXf::Map(
      &balanceDefs[0][0],
      sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
  } else {
    jointsF = VectorXf::Map(
      &balanceDefs[1][0],
      sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
  }
  auto jointsDelta = jointsF - joints;
  vector<unsigned> jointIds;
  for (int i = L_HIP_YAW_PITCH; i < R_LEG_END; ++i)
    jointIds.push_back(i);
  int size = jointIds.size();
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  int totalSteps = ceil(timeToReachB / cycleTime);
  for (int i = 0; i < size; ++i) {
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  float step = cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    float timeParam = step / timeToReachB;
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
  execTime = 0.f;
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  inBehavior = true;
}
