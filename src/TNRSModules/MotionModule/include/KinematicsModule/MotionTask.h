/**
 * @file MotionModule/include/KinematicsModule/MotionTask.h
 *
 * This file defines the struct MotionTask
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Utils/include/MathsUtils.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"

enum class MotionTaskType {
  POSTURE,
  COM,
  CARTESIAN,
  CONTACT,
  TORSO,
};

/**
 * @struct MotionTask
 * @brief A base class for all kinds of tasks to be provided to the task
 *   based ik solver
 */
template <typename Scalar>
class MotionTask
{
public:

  /**
   * Constructor
   * 
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   * @param taskType: Type of task
   */
  MotionTask(
    const Scalar& weight, 
    const Scalar& gain, 
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& jsType,
    const MotionTaskType& taskType) :
    weight(weight),
    gain(gain),
    activeJoints(activeJoints),
    km(km),
    jsType(jsType),
    nDof(km->getNJoints())
  {
    assert(activeJoints.size() == nDof);
  }
  
  virtual ~MotionTask() {}
  
  /**
   * Computes the jacobian matrix for the given task
   * 
   * @returns task jacobian
   */
  virtual Matrix<Scalar, Dynamic, Dynamic>
    computeJacobian() = 0;
  
  /**
   * Computes the residual for the given task
   */
  virtual Matrix<Scalar, Dynamic, Dynamic> 
    computeResidual(const Scalar& dt) = 0;
  
  /**
   * Computes the current cost based on residual
   * @param dt Time step for residual
   * @return task cost
   */
  Scalar computeCost(const Scalar& dt) {
    return weight * gain * computeResidual(dt).squaredNorm();
  }
  
  /**
   * Checks whether the given task has any conflict with this task
   * @return true if the conflict exists
   */
  virtual bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
    return false;
  }

  //! Setters
  void setGain(const Scalar& gain) { this->gain = gain; }
  void setWeight(const Scalar& weight) { this->weight = weight; }
  void setActiveJoints(const vector<bool>& activeJoints) { this->activeJoints = activeJoints; }
  
  //! Getters
  Scalar getGain() const { return gain; }
  Scalar getWeight() const { return weight; }
  MotionTaskType getType() const { return taskType; }
  vector<bool> getActiveJoints() const { return activeJoints; }
  Matrix<Scalar, Dynamic, Dynamic> getJacobian() 
  { 
    Matrix<Scalar, Dynamic, Dynamic> jacobian = computeJacobian();
    for (size_t i = 0; i < nDof; ++i) {
      if (!activeJoints[i])
        jacobian.col(i).setZero();
    }
    return jacobian; 
  }

  Matrix<Scalar, Dynamic, Dynamic> getResidual(const Scalar& dt)
  {
    return gain * computeResidual(dt);
  }
 
protected:  
  //! A pointer to robot kinematics
  boost::shared_ptr<KinematicsModule<Scalar> > km;
  
  //! Type of joint state to be used for kinematic computation
  JointStateType jsType;

  //! Number of degrees of freedom possible for this task
  unsigned nDof;

  //! Type of the task
  MotionTaskType taskType;

  //! activeJoints degrees of freedom
  vector<bool> activeJoints;
  
  //! MotionTask gain
  Scalar gain;
  
  //! MotionTask priority weight
  Scalar weight;
};
template class MotionTask<MType>;
typedef boost::shared_ptr<MotionTask<MType> > MotionTaskPtr;

/**
 * @struct PostureTask
 * @brief A task that defines whole-body posture tracking
 */
template <typename Scalar>
class PostureTask : public MotionTask<Scalar>
{
public:

  /**
   * Constructor
   * @param targetJoints: Target joints for the required posture
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  PostureTask(
    const Matrix<Scalar, Dynamic, 1>& targetJoints,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const JointStateType& jsType = JointStateType::SIM) :
    MotionTask<Scalar>(weight, gain, activeJoints, km, jsType, MotionTaskType::POSTURE),
    targetJoints(targetJoints)
  {
  }

  /**
   * Checks whether the given task has any conflict with this posture task
   * @return true if the given task is also a posture task and has conflicting active
   *   joints
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
    if (task->getType() == this->taskType) {
      auto taskAj = task->getActiveJoints();
      for (size_t i = 0; i < this->activeJoints.size(); ++i) {
        if (this->activeJoints[i] && taskAj[i])
          return true;
      }
    }
    return false;
  }

  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    return Matrix<Scalar, Dynamic, Dynamic>::Identity(NUM_JOINTS, NUM_JOINTS);
  }

  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    Matrix<Scalar, Dynamic, 1> actualJoints = this->km->getJointPositions(0, NUM_JOINTS, this->jsType);
    return (targetJoints - actualJoints) / dt;
  }

  //! Setters
  void setTargetPosture(
    const Matrix<Scalar, Dynamic, 1>& targetJoints)
    { this->targetJoints = targetJoints; }

private:
  //! Target joints for the required posture
  Matrix<Scalar, Dynamic, 1> targetJoints;
};
template class PostureTask<MType>;
typedef boost::shared_ptr<PostureTask<MType> > PostureTaskPtr;


/**
 * @struct ComTask
 * @brief A task that defines center of mass tracking
 */
template <typename Scalar>
class ComTask : public MotionTask<Scalar>
{
public:

  /**
   * Constructor
   * @param refFrame: Frame of reference for center of mass tracking.
   *   It should either be CHAIN_L_LEG or CHAIN_R_LEG
   * @param targetCom: Target center of mass position from ref frame
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  ComTask(
    const unsigned& refFrame,
    const Matrix<Scalar, 3, 1>& targetCom,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const JointStateType& jsType = JointStateType::SIM) :
    MotionTask<Scalar>(weight, gain, activeJoints, km, jsType, MotionTaskType::COM),
    refFrame(refFrame), targetCom(targetCom)
  {
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    return this->km->computeComJacobian(refFrame, this->jsType);
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    //cout << "targetCom:\n " << targetCom << endl;
    //cout << "actualCom:\n" << this->km->computeComWrtBase(refFrame, FEET_BASE, this->jsType) << endl;
    Matrix<Scalar, 3, 1> diff = (targetCom - this->km->computeComWrtBase(refFrame, FEET_BASE, this->jsType)) / dt;
    diff[2] = 0;
    return diff;
  }
  
  /**
   * Checks whether the given task has any conflict with this com task
   * @return true if the given task is also a com task
   *   joints
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
    if (task->getType() == this->taskType) {
      return true;
    }
    return false;
  }

  //! Setters
  void setTargetCom(
    const Matrix<Scalar, 3, 1>& targetCom) 
    { this->targetCom = targetCom; }

private:
  //! Reference frame for com computations
  unsigned refFrame;

  //! Target center of mass position
  Matrix<Scalar, 3, 1> targetCom;
};
template class ComTask<MType>;
typedef boost::shared_ptr<ComTask<MType> > ComTaskPtr;

/**
 * @struct CartesianTask
 * @brief A task that defines the tracking of a limb end-effector
 *   in cartesian space
 */
template <typename Scalar>
class CartesianTask : public MotionTask<Scalar>
{
public:
  /**
   * Constructor
   * 
   * @param chainIndex: Index of the chain for which this task is 
   *   defined
   * @param endEffector: Transformation frame of the end-effector to be 
   *   moved
   * @param target: Target transformation to be reached by the 
   *   end-effector
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   * @param taskType: Type of task
   */
  CartesianTask(
    const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Matrix<Scalar, 4, 4> target,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& jsType = JointStateType::SIM,
    const MotionTaskType& taskType = MotionTaskType::CARTESIAN) :
    MotionTask<Scalar>(weight, gain, activeJoints, km, jsType, taskType),
    endEffector(endEffector),
    target(target),
    chainIndex(chainIndex)
  {
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    Matrix<Scalar, 6, Dynamic> j;
    j.resize(6, this->nDof);
    j.setZero();
    auto ch = this->km->getLinkChain(chainIndex);
    //cout << "j.transpose()\n" << j.transpose() << endl;
    j.block(0, ch->start, 6, ch->size) =
      this->km->computeLimbJ(chainIndex, endEffector, this->jsType);
    Matrix<Scalar, 6, Dynamic> supportLimbJ = this->km->computeLimbJ(CHAIN_L_LEG, FEET_BASE, this->jsType);
    Matrix<Scalar, 3, 1> ee =
      (MathsUtils::getTInverse(this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, this->jsType)) *
      this->km->getForwardEffector(chainIndex, endEffector, this->jsType)).block(0, 3, 3, 1);
    j.block(0, this->km->getLinkChain(CHAIN_L_LEG)->start, 3, this->km->getLinkChain(CHAIN_L_LEG)->size) =
      -supportLimbJ.block(0, 0, 3, this->km->getLinkChain(CHAIN_L_LEG)->size) + MathsUtils::makeSkewMat(ee) * supportLimbJ.block(3, 0, 3, this->km->getLinkChain(CHAIN_L_LEG)->size);
    j.block(3, this->km->getLinkChain(CHAIN_L_LEG)->start, 3, this->km->getLinkChain(CHAIN_L_LEG)->size) =
      -supportLimbJ.block(3, 0, 3, this->km->getLinkChain(CHAIN_L_LEG)->size);
    return j;
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    Matrix<Scalar, 4, 4> pos = 
      MathsUtils::getTInverse(this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, this->jsType)) *
      this->km->getForwardEffector(chainIndex, endEffector, this->jsType);
    Matrix<Scalar, 6, 1> diff;
    diff.setZero();
    diff.block(0, 0, 3, 1) = target.block(0, 3, 3, 1) - pos.block(0, 3, 3, 1);
    diff.block(3, 0, 3, 1) = MathsUtils::getOrientationDiff(pos, target);
    return diff / dt;
  }

  /**
   * Checks whether the given task has any conflict with this cartesian task
   * @return true if the given task is also a cartesian task for the same chain
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
    if (task->getType() == this->taskType) {
      if (boost::static_pointer_cast<CartesianTask<Scalar> >(task)->chainIndex ==
          this->chainIndex)
        return true;
    }
    return false;
  }
  
  //! Setters
  void setTarget(
    const Matrix<Scalar, 4, 4>& target) 
    { this->target = this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, JointStateType::ACTUAL).inverse() * target; }
  
protected:
  //! Index of the chain for which this task is defined
  unsigned chainIndex;

  //! End effector transformation for this chain
  Matrix<Scalar, 4, 4> endEffector;

  //! Target end-effector position
  Matrix<Scalar, 4, 4> target;
};
template class CartesianTask<MType>;
typedef boost::shared_ptr<CartesianTask<MType> > CartesianTaskPtr;

/**
 * @struct ContactTask
 * @brief A task that defines the fixed end-effector contact for
 *   the given limb in cartesian space
 */
template <typename Scalar>
class ContactTask : public CartesianTask<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param chainIndex: Index of the chain for which this task is
   *   defined
   * @param endEffector: Transformation frame of the end-effector to be
   *   moved
   * @param target: Target transformation to be reached by the
   *   end-effector
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  ContactTask(
    const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& jsType = JointStateType::SIM) :
    CartesianTask<Scalar>(
      chainIndex,
      endEffector,
      Matrix<Scalar, 4, 4>::Identity(),
      weight,
      gain,
      activeJoints,
      km,
      jsType,
      MotionTaskType::CONTACT)
  {
    this->target = // Get first position using actual states
      this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, JointStateType::ACTUAL).inverse() *
      this->km->getForwardEffector(chainIndex, endEffector, JointStateType::ACTUAL);
  }
};
template class ContactTask<MType>;
typedef boost::shared_ptr<ContactTask<MType> > ContactTaskPtr;

/**
 * @struct TorsoTask
 * @brief A task that defines the tracking of desired torso pose
 */
template <typename Scalar>
class TorsoTask : public MotionTask<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param baseFrame: Index of the base chain
   * @param endEffector: Transformation frame of the end-effector to be
   *   used as base frame
   * @param target: Target transformation to be reached by the
   *   torso
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  TorsoTask(
    const unsigned& baseFrame,
    const Matrix<Scalar, 4, 4> endEffector,
    const Matrix<Scalar, 4, 4> target,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& jsType = JointStateType::SIM) :
    MotionTask<Scalar>(weight, gain, activeJoints, km, jsType, MotionTaskType::TORSO),
    endEffector(endEffector),
    target(target),
    baseFrame(baseFrame)
  {
  }


  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    Matrix<Scalar, 3, Dynamic> j;
    j.resize(3, this->nDof);
    j.setZero();
    auto ch = this->km->getLinkChain(baseFrame);
    j.block(0, ch->start, 3, ch->size) =
      -this->km->computeLimbJ(baseFrame, endEffector, this->jsType).block(3, 0, 3, ch->size);
    return j;
  }

  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    Matrix<Scalar, 4, 4> pose = MathsUtils::getTInverse(this->km->getForwardEffector(baseFrame, endEffector, this->jsType));
    LOG_INFO("Torso target pose:");
    LOG_INFO(target);
    LOG_INFO("Torso actual pose:");
    LOG_INFO(pose);
    return MathsUtils::getOrientationDiff(pose, target) / dt;
  }

  /**
   * Checks whether the given task has any conflict with this torso task
   * @return true if the given task is also a torso task
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
    if (task->getType() == this->taskType) {
      return true;
    }
    return false;
  }

  //! Setters
  void setTarget(
    const Matrix<Scalar, 4, 4>& target)
    { this->target = target; }

protected:
  //! Index of the base chain
  unsigned baseFrame;

  //! Transformation frame of the end-effector to be used as base frame
  Matrix<Scalar, 4, 4> endEffector;

  //! Target transformation to be reached by the torso
  Matrix<Scalar, 4, 4> target;
};
template class TorsoTask<MType>;
typedef boost::shared_ptr<TorsoTask<MType> > TorsoTaskPtr;
