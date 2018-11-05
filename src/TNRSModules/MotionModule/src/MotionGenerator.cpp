/**
 * @file MotionModule/src/MotionGenerator.cpp
 *
 * This file implements the class MotionGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/JointRequest.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
void MotionGenerator<Scalar>::update()
{
  if (motionTasks.empty())
    return;
  Matrix<Scalar, Dynamic, 1> joints = this->kM->solveTasksIK(motionTasks, 100);
  auto jr = boost::make_shared<JointRequest>();
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    jr->setValue(joints[i], i);
  }
  BaseModule::publishModuleRequest(jr);
}

template <typename Scalar>
void MotionGenerator<Scalar>::naoqiSetAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
{
  motionProxy->setAngles(names, angles, fractionMaxSpeed);
}

template <typename Scalar>
void MotionGenerator<Scalar>::naoqiChangeAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
{
  motionProxy->changeAngles(names, angles, fractionMaxSpeed);
}

template <typename Scalar>
void MotionGenerator<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists,
  const AL::ALValue& positionLists,
  const bool& postCommand,
  const MotionLoggerPtr& logger)
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

  if (logger)
    logger->recordJointCmds(timeLists, positionLists, ids);

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
void MotionGenerator<Scalar>::addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  if (validateTask(task))
    motionTasks.push_back(task);
}

template <typename Scalar>
bool MotionGenerator<Scalar>::validateTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  if (motionTasks.empty()) {
    return true;
  } else {
    for (size_t i = 0; i < motionTasks.size(); ++i) {
      if (motionTasks[i]->checkConflict(task))
        return false;
    }
  }
  return true;
}

template <typename Scalar>
Scalar MotionGenerator<Scalar>::runKeyFrameMotion(
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

template class MotionGenerator<MType>;
