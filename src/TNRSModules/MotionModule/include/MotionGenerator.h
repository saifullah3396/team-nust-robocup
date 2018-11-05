/**
 * @file MotionModule/include/MotionGenerator.h
 *
 * This file declares the class MotionGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <boost/filesystem.hpp>
#include "BehaviorManager/include/Behavior.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/MathsUtils.h"

//! Forward declaration
template <typename Scalar>
class MotionTask;

/**
 * @class MotionGenerator
 * @brief A class that recieves tasks from several motion modules and
 *   generates feasible joint requests
 */
template <typename Scalar>
class MotionGenerator : public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   */
  MotionGenerator(MotionModule* motionModule) :
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
  ~MotionGenerator()
  {
  }

  /**
   * Transforms requested tasks to joint requests and sends it to dcm
   */
  void update();

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
    const bool& postCommand,
    const MotionLoggerPtr& logger = MotionLoggerPtr());

  /**
   * Stops all naoqi generated motions
   */
  void killAllMotions() {
    motionProxy->killAll();
    motionProxy->stopMove();
  }

  /**
   * Uses naoqi api to open the given robot hand
   *
   * @handIndex: Hand index defined in Utils/Hardwareids.h
   */
  void openHand(const unsigned& handIndex) {
    if (handIndex == L_HAND)
      motionProxy->post.openHand("LHand");
    else if (handIndex == R_HAND)
      motionProxy->post.openHand("RHand");
  }

  /**
   * Uses naoqi api to close the given robot hand
   *
   * @handIndex: Hand index defined in Utils/Hardwareids.h
   */
  void closeHand(const unsigned& handIndex) {
    if (handIndex == L_HAND)
      motionProxy->post.closeHand("LHand");
    else if (handIndex == R_HAND)
      motionProxy->post.closeHand("RHand");
  }

  /**
   * Uses naoqi setAngles() to set the desired angles for given joints
   *
   * @param names: Joint names as defined in naoqi
   * @param angles: Joint angles as requested
   * @param fractionMaxSpeed: Maximum speed limit
   */
  void naoqiSetAngles(
    const AL::ALValue& names,
    const AL::ALValue& angles,
    const float& fractionMaxSpeed);

  /**
   * Uses naoqi changeAngles() to set the desired angles for given joints
   *
   * @param names: Joint names as defined in naoqi
   * @param angles: Joint angles as requested
   * @param fractionMaxSpeed: Maximum speed limit
   */
  void naoqiChangeAngles(
    const AL::ALValue& names,
    const AL::ALValue& angles,
    const float& fractionMaxSpeed);

  /**
   * Adds the given task to motion tasks vector if it is not conflicting with other tasks
   *
   * @param task: The desired task
   */
  void addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task);

  /**
   * Returns the kinematics module
   *
   * @return KinematicsModulePtr
   */
  boost::shared_ptr<KinematicsModule<Scalar> > getKinematicsModule()
    { return kM; }

private:
  /**
   * Validates the given motion task
   *
   * @param task: The validated task
   *
   * @return returns true if the given task is not conflicting with tasks already present
   */
  bool validateTask(const boost::shared_ptr<MotionTask<Scalar> >& task);

  //! Cycle time of this motion behavior
  Scalar cycleTime;

  //! Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

  //! NaoQi's motion proxy
  ALMotionProxyPtr motionProxy;

  //! Base MotionModule object pointer
  MotionModule* motionModule;

  //! Tasks vector containing all the tasks recieved from several motion modules.
  //! All the tasks are solved together to give a resultant joint motion for the
  //! given motion cycle.
  vector<boost::shared_ptr<MotionTask<Scalar> > > motionTasks;
};
