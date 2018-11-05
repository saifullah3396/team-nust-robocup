/**
 * @file MotionModule/include/MotionBehavior.h
 *
 * This file declares the class MotionBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <boost/filesystem.hpp>
#include "BehaviorManager/include/Behavior.h"
#include "MotionModule/include/MotionBehaviorIds.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/MotionLogger.h"
#include "Utils/include/MathsUtils.h"

#define MOTION_LOGGER static_pointer_cast<MotionLogger>(this->dataLogger)

/**
 * @class MotionBehavior
 * @brief A base class for all kinds of motion behaviors
 */
template <typename Scalar>
class MotionBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  MotionBehavior(
		MotionModule* motionModule,
		const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    Behavior(config, name),
    MemoryBase(motionModule),
    motionModule(motionModule)
  {
    kM = motionModule->getKinematicsModule();
    mG = motionModule->getMotionGenerator();
    cycleTime = motionModule->getPeriodMinMS() / 1000.f;
  }

  /**
   * Destructor
   */
  virtual
  ~MotionBehavior()
  {
  }

  /**
   * @derived Creates a motion logger
   */
  virtual JsonLoggerPtr makeLogger()
    { return boost::make_shared<MotionLogger>(this->logsDirPath + "/" + this->name + ".json"); }

  /**
   * @derived
   */
  void updateDataLogger()
  {
    vector<Scalar> joints(NUM_JOINTS);
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      joints[i] = kM->getJointState(i)->position;
    }
    MOTION_LOGGER->recordJointStates(joints);
  }

  /**
   * Wrapper for MotionGenerator::killAllMotions()
   */
  void killAllMotions() {
    mG->killAllMotions();
  }

  /**
   * Wrapper for MotionGenerator::openHand()
   */
  void openHand(const unsigned& handIndex) {
    mG->openHand(handIndex);
  }

  /**
   * Wrapper for MotionGenerator::closeHand()
   */
  void closeHand(const unsigned& handIndex) {
    mG->closeHand(handIndex);
  }

  /**
   * Wrapper for MotionGenerator::naoqiSetAngles()
   */
  void naoqiSetAngles(
    const AL::ALValue& names,
    const AL::ALValue& angles,
    const float& fractionMaxSpeed) {
    mG->naoqiSetAngles(names, angles, fractionMaxSpeed);
  }

  /**
   * Wrapper for MotionGenerator::naoqiChangeAngles()
   */
  void naoqiChangeAngles(
    const AL::ALValue& names,
    const AL::ALValue& angles,
    const float& fractionMaxSpeed) {
    mG->naoqiChangeAngles(names, angles, fractionMaxSpeed);
  }

  /**
   * Returns the kinematics module
   * 
   * @return KinematicsModulePtr
   */  
  boost::shared_ptr<KinematicsModule<Scalar> > getKinematicsModule() 
    { return kM; }

protected:
  /**
   * Takes a 2D vector of key frames of size [times][joints], maps
   * it into eigen based vectors and calls the overloaded
   * runKeyFrameMotion();
   *
   * @param keyFrames: A 2D-vector consisting of times and joint values.
   *   This vector is of size (1 + numJoints) x (numKeyFrames). A single
   *   key frame must be defined as... (time, q1, q2, ... qN).
   */
  template<typename type, std::size_t times, std::size_t joints>
  Scalar runKeyFrameMotion(
    const type (&keyFrames)[times][joints])
  {
    vector<Matrix<Scalar, Dynamic, 1> > targetJoints(times);
    Matrix<Scalar, Dynamic, 1> targetTimes(times);
    for (int i = 0; i < times; ++i) {
      targetJoints[i] = Matrix<Scalar, Dynamic, 1>::Map(
        &keyFrames[i][0] + 1,
        (sizeof(keyFrames[i]) - sizeof(keyFrames[i][0])) / sizeof(keyFrames[i][0]));
      targetTimes[i] = keyFrames[i][0];
    }
    return mG->runKeyFrameMotion(targetJoints, targetTimes);
  }

  /**
   * Wrapper for MotionGenerator::naoqiJointInterpolation()
   */
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& positionLists,
    const bool& postCommand)
  {
    if (this->logData)
      mG->naoqiJointInterpolation(ids, timeLists, positionLists, postCommand, MOTION_LOGGER);
    else
      mG->naoqiJointInterpolation(ids, timeLists, positionLists, postCommand);
  }

  /**
   * Wrapper for MotionGenerator::addMotionTask()
   */
  void addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
    { mG->addMotionTask(task); }

  //! Cycle time of this motion behavior
  Scalar cycleTime;

  //! Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

  //! Motion generator module object
  boost::shared_ptr<MotionGenerator<Scalar> > mG;
  
  //! Base MotionModule object pointer
  MotionModule* motionModule;
};
