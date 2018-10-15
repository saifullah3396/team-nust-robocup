/**
 * @file MotionModule/src/MotionBehavior.cpp
 *
 * This file implements the class MotionBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
JsonLoggerPtr MotionBehavior<Scalar>::makeLogger()
{
  return 
    boost::make_shared<MotionLogger>(
      this->logsDirPath + "/" + this->name + ".json"
    );
}

template <typename Scalar>
void MotionBehavior<Scalar>::updateDataLogger()
{
  vector<Scalar> joints(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joints[i] = kM->getJointState(i)->position;
  }
  MOTION_LOGGER->recordJointStates(joints);
}

template <typename Scalar>
void MotionBehavior<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists, 
  const AL::ALValue& positionLists,
  const bool& postCommand)
{
  ASSERT(
    ids.size() == timeLists.getSize() && 
    timeLists.getSize() == positionLists.getSize()
  );
  AL::ALValue names;
  names.clear();
  names.arraySetSize(ids.size());
  for (int i = 0; i < ids.size(); ++i)
    names[i] = jointNameConsts[ids[i]];

  if (this->logData) //! If data logging is turned on
    MOTION_LOGGER->
      recordJointCmds(timeLists, positionLists, ids);
    
  try {
    if (postCommand) {
      motionProxy->post.angleInterpolation(
        names,
        positionLists,
        timeLists,
        true
      );
    } else {
      motionProxy->angleInterpolation(
        names,
        positionLists,
        timeLists,
        true
      );
    }
  } catch (exception &e) {
    ERROR(e.what())
  }
}

template <typename Scalar>
Scalar MotionBehavior<Scalar>::runKeyFrameMotion(
  const vector<Matrix<Scalar, Dynamic, 1>>& targetJoints, const Matrix<Scalar, Dynamic, 1>& times)
{
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(NUM_JOINTS);
  jointPositions.arraySetSize(NUM_JOINTS);
  
  Scalar time = 0;
  vector<unsigned> jointIds;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    jointIds.push_back(i);
    jointPositions[i].arraySetSize(targetJoints.size());
    jointTimes[i].arraySetSize(targetJoints.size());
    time = 0;
    for (size_t j = 0; j < targetJoints.size(); ++j) {
      time += times[j];
      jointPositions[i][j] = targetJoints[j][i] * M_PI / 180.f;
      jointTimes[i][j] = time;
    }
  }
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  return time;
}

template class MotionBehavior<MType>;
