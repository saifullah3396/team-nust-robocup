/**
 * @file MotionModule/include/KinematicsModule/Task.h
 *
 * This file defines the struct Task
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Utils/include/MathsUtils.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"

/**
 * @struct Task
 * @brief A base class for all kinds of tasks to be provided to the task
 *   based ik solver
 */
template <typename Scalar>
class Task
{
public:

  /**
   * Constructor
   * 
   * @param weight: Task priority weight
   * @param gain: Task gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param type: Type of joint state to be used for kinematic computation
   */
  Task(
    const Scalar& weight, 
    const Scalar& gain, 
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& type) :
    weight(weight),
    gain(gain),
    activeJoints(activeJoints),
    km(km),
    type(type),
    nDof(km->getNJoints())
  {
    assert(activeJoints.size() == nDof);
  }
  
  virtual ~Task() {}
  
  /**
   * Computes the jacobian matrix for the given task
   * 
   * @returns Task jacobian
   */
  virtual Matrix<Scalar, Dynamic, Dynamic>
    computeJacobian() = 0;
  
  /**
   * Computes the residual for the given task
   */
  virtual Matrix<Scalar, Dynamic, Dynamic> 
    computeResidual(const Scalar& dt) = 0;
  
  Scalar computeCost(const Scalar& dt) {
    return weight * gain * computeResidual(dt).squaredNorm();
  }
  
  //! Setters
  void setGain(const Scalar& gain) { this->gain = gain; }
  void setWeight(const Scalar& weight) { this->weight = weight; }
  void setActiveJoints(const vector<bool>& activeJoints) { this->activeJoints = activeJoints; }
  
  //! Getters
  Scalar getGain() const { return gain; }
  Scalar getWeight() const { return weight; }
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
  JointStateType type;

  //! Number of degrees of freedom possible for this task
  unsigned nDof;
private:
  //! activeJoints degrees of freedom
  vector<bool> activeJoints;
  
  //! Task gain
  Scalar gain;
  
  //! Task priority weight
  Scalar weight;
};
template class Task<MType>;
typedef boost::shared_ptr<Task<MType> > TaskPtr;


/**
 * @struct ComTask
 * @brief A task that defines center of mass tracking
 */
template <typename Scalar>
class ComTask : public Task<Scalar>
{
public:

  /**
   * Constructor
   * @param refFrame: Frame of reference for center of mass tracking.
   *   It should either be CHAIN_L_LEG or CHAIN_R_LEG
   * @param targetCom: Target center of mass position from ref frame
   * @param weight: Task priority weight
   * @param gain: Task gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param type: Type of joint state to be used for kinematic computation
   */
  ComTask(
    const unsigned& refFrame,
    const Matrix<Scalar, 3, 1>& targetCom,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const JointStateType& type = JointStateType::SIM) :
    Task<Scalar>(weight, gain, activeJoints, km, type),
    refFrame(refFrame), targetCom(targetCom)
  {
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    return this->km->computeComJacobian(refFrame, this->type);
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    //cout << "targetCom:\n " << targetCom << endl;
    //cout << "actualCom:\n" << this->km->computeComWrtBase(refFrame, FEET_BASE, this->type) << endl;
    Matrix<Scalar, 3, 1> diff = (targetCom - this->km->computeComWrtBase(refFrame, FEET_BASE, this->type)) / dt;
    diff[2] = 0;
    return diff;
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
class CartesianTask : public Task<Scalar>
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
   * @param weight: Task priority weight
   * @param gain: Task gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param type: Type of joint state to be used for kinematic computation
   */
  CartesianTask(
    const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Matrix<Scalar, 4, 4> target,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& type = JointStateType::SIM) :
    Task<Scalar>(weight, gain, activeJoints, km, type),
    endEffector(endEffector),
    target(target),
    chainIndex(chainIndex)
  {
    this->target = this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, JointStateType::ACTUAL).inverse() * target;
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian()
  {
    Matrix<Scalar, 6, Dynamic> j;
    j.resize(6, this->nDof);
    j.setZero();
    auto ch = this->km->getLinkChain(chainIndex);
    //cout << "j.transpose()\n" << j.transpose() << endl;
    j.block(0, ch->start, 6, ch->size) =
      this->km->computeLimbJ(chainIndex, endEffector, this->type);
    //cout << "actual j;\n" << this->km->computeLimbJ(chainIndex, endEffector, this->type) << endl;
    //cout << "j.transpose()\n" << j.transpose() << endl;
    return j;
  }
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt)
  {
    Matrix<Scalar, 4, 4> pos = 
      this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, this->type).inverse() *
      this->km->getForwardEffector(chainIndex, endEffector, this->type);
    Matrix<Scalar, 6, 1> diff;
    diff.setZero();
    diff.block(0, 0, 3, 1) = target.block(0, 3, 3, 1) - pos.block(0, 3, 3, 1);
    diff.block(3, 0, 3, 1) = MathsUtils::getOrientationDiff(pos, target);
    //cout << "target pose:\n" << target << endl;
    //cout << "actual pose:\n" << pos << endl;
    //cout << "diff: " << diff << endl;
    return diff / dt;
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

  //! Target center of mass position
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
   * @param weight: Task priority weight
   * @param gain: Task gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param type: Type of joint state to be used for kinematic computation
   */
  ContactTask(
    const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const KinematicsModulePtr& km,
    const JointStateType& type = JointStateType::SIM) :
    CartesianTask<Scalar>(
      chainIndex,
      endEffector,
      Matrix<Scalar, 4, 4>::Identity(),
      weight,
      gain,
      activeJoints,
      km,
      type)
  {
    this->target = // Get first position using actual states
      this->km->getForwardEffector(CHAIN_L_LEG, FEET_BASE, JointStateType::ACTUAL).inverse() *
      this->km->getForwardEffector(chainIndex, endEffector, JointStateType::ACTUAL);
  }
};
template class ContactTask<MType>;
typedef boost::shared_ptr<ContactTask<MType> > ContactTaskPtr;
