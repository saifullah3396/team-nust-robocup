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
#include "Utils/include/MathsUtils.h"

//! Forward declaration
class MotionLogger;
typedef boost::shared_ptr<MotionLogger> MotionLoggerPtr;

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
    motionProxy = motionModule->getSharedMotionProxy();
    kM = motionModule->getKinematicsModule();
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
  virtual JsonLoggerPtr makeLogger();
  
  /**
   * @derived
   */ 
  virtual void updateDataLogger();
  
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
    return runKeyFrameMotion(targetJoints, targetTimes);
  }

  /**
   * Takes a vector of joint positions to be called at given times
   * 
   * @param joints: Required joint positions at given times
   * @param times: Times vector
   * 
   * @return The cumulative time of this keyframe motion
   */
  Scalar runKeyFrameMotion(
    const vector<Matrix<Scalar, Dynamic, 1> >& joints, const Matrix<Scalar, Dynamic, 1>& times);

  /**
   * Interpolates joints using naoqi joint interpolation
   *
   * @param ids: Ids of all the joints under consideration
   * @param timeLists: Times for interpolation of each joint
   * @param positionLists: List of joint angular positions
   * @param postCommand: Whether this command is to be run in a
   *   separate thread or as a blocking call
   */
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& positionLists,
    const bool& postCommand);

  //! Cycle time of this motion behavior
  Scalar cycleTime;
  
  //! Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

  //! NaoQi's motion proxy 
  ALMotionProxyPtr motionProxy;
  
  //! Base MotionModule object pointer
  MotionModule* motionModule;
};
