/**
 * @file MotionModule/BalanceModule/Types/MPComControl.cpp
 *
 * This file implements the class MPComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/Types/MPComControl.h"
#include "MotionModule/include/KinematicsModule/Joint.h"

template<typename Scalar>
MPComControlConfigPtr MPComControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <MPComControlConfig> (this->config);
}

template<typename Scalar>
void
MPComControl<Scalar>::initiate()
{
  PRINT("MPComControl.initiate()")
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

template<typename Scalar>
void
MPComControl<Scalar>::update()
{
  //PRINT("MPComControl.update()")
  //PRINT("MPComControl.execTime: " << execTime)
  //PRINT("MPComControl.timeToReachB: " << timeToReachB)
  if (execTime > timeToReachB + this->cycleTime * 5) {
    finish();
  } else {
    execTime += this->cycleTime;
  }
}

template<typename Scalar>
void
MPComControl<Scalar>::finish()
{
  this->motionProxy->killAll();
  this->inBehavior = false;
}

template<typename Scalar>
void
MPComControl<Scalar>::balanceMotionPrimitive()
{
  auto jointStates = 
    this->kM->getJointStates(0, NUM_JOINTS);
  Matrix<Scalar, Dynamic, 1> jointsF(NUM_JOINTS);
  if (supportLeg == CHAIN_L_LEG) {
    jointsF = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[0][0],
      sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
  } else {
    jointsF = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[1][0],
      sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
  }
  Matrix<Scalar, Dynamic, 1> jointsDelta(NUM_JOINTS);
  
  for (size_t i = 0; i < jointStates.size(); ++i)
   jointsDelta[i] = jointsF[i] - jointStates[i]->position;
   
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
  int totalSteps = ceil(timeToReachB / this->cycleTime);
  for (int i = 0; i < size; ++i) {
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  Scalar step = this->cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    Scalar timeParam = step / timeToReachB;
    Scalar multiplier =
      6 * pow(timeParam, 5) - 15 * pow(timeParam, 4) + 10 * pow(timeParam, 3);
    for (int i = 0; i < size; ++i) {
      int jointId = jointIds[i];
      jointPositions[i][j] =
        jointStates[jointId]->position + jointsDelta[jointId] * multiplier;
      jointTimes[i][j] = step;
    }
    step += this->cycleTime;
  }
  execTime = 0.f;
  this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  this->inBehavior = true;
}

template class MPComControl<MType>;
