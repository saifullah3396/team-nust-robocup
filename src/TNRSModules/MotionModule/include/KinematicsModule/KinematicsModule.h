/**
 * @file MotionModule/KinematicsModule/KinematicsModule.h
 *
 * This file declares the class for solving the kinematics
 * of the robot.
 *
 * Some parts of this Module have been extracted from the
 * NaoKinematics provided by:
 * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
 * @author Vosk
 * @link https://github.com/kouretes/NAOKinematics
 * @cite Kofinas N., Orfanoudakis E., Lagoudakis M.: Complete Analytical
 *   Inverse Kinematics for NAO, Proceedings of the 13th International
 *   Conference on Autonomous Robot Systems and Competitions (ROBOTICA),
 *   Lisbon, Portugal, April 2013, pp. 1-6.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 08 Feb 2017
 */

#pragma once

#include <fstream>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include "TNRSBase/include/MemoryBase.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

using namespace Utils;

class MotionModule;
template <typename Scalar>
class ComEstimator;
template <typename Scalar>
class ProcessModel;
template <typename Scalar>
class ComState;
template <typename Scalar>
class TorsoState;
template <typename Scalar>
class Joint;
template <typename Scalar>
class JointState;
template <typename Scalar>
class LinkInfo;
template <typename Scalar>
class LinkChain;
template <typename Scalar>
class MotionTask;
template <typename Scalar>
class PostureTask;
template <typename Scalar>
class ContactTask;
template <typename Scalar>
class CartesianTask;
template <typename Scalar>
class ComTask;
template <typename Scalar>
class TorsoTask;

/**
 * @class KinematicsModule
 * @brief The class for kinematics and dynamics of the robot.
 */
template <typename Scalar>
class KinematicsModule : public MemoryBase
{
public:
  /**
   * Default constructor for this class
   *
   * @param motionModule: base class.
   */
  KinematicsModule(MotionModule* motionModule);

  /**
   * Default destructor fr this class
   */
  ~KinematicsModule();

  /**
   * Updates the kinematics model of the robot based
   * on position sensor data.
   */
  void update();

  /**
   * Sets the simulated joint states equal to actual joint state
   */
  void setStateFromTo(
    const JointStateType& from, 
    const JointStateType& to);

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void setJointPositions(
    const unsigned& startIndex, 
    const Matrix<Scalar, Dynamic, 1>& simPosition, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void setChainPositions(
    const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void setJointVelocities(
    const unsigned& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simVelocities,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void setChainVelocities(
    const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simVelocities,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void setJointAccelerations(
    const unsigned& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simAccelerations,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void setChainAccelerations(
    const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simAccelerations,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints 
   */
  void setJointState(
    const unsigned& startIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition, 
    const Matrix<Scalar, Dynamic, 1>& simVelocity,
    const Matrix<Scalar, Dynamic, 1>& simAcceleration,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints
   */
  void setChainState(
    const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, 1>& simPosition, 
    const Matrix<Scalar, Dynamic, 1>& simVelocity,
    const Matrix<Scalar, Dynamic, 1>& simAcceleration,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Converts the specified cartesian velocities to joint velocities
   * for the given chain.
   *
   * @param chainIndex: Index of the limb.
   * @param cVels: The cartesian space velocity vector.
   * @param endEffector: The end effector transformation
   *   matrix from the final frame of the chain
   * @param type: Type of joints
   *
   * @return Matrix<Scalar, Dynamic, 1>
   */
  Matrix<Scalar, Dynamic, 1> cartToJointVels(const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, 1> cVels, const Matrix<Scalar, 4, 4> endEffector,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 6, Dynamic> computeLimbJ(const unsigned& chainIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation
   *   matrix from the final frame of the chain
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 6, Dynamic> computeLimbJ(const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the Center of mass jacobian for the whole body
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 3, Dynamic> computeComJacobian(
    const unsigned& baseFrame,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the specified limb Center of mass jacobian with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, 3, Dynamic> computeLimbComJ(
    const unsigned& chainIndex, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the center of mass jacobian for the given link
   *
   * @param index: Link index
   * @param jacobianV: Com velocity jacobian to be updated
   * @param jacobianW: Com angular velocity jacobian to be updated
   * @param type: Type of joints
   * 
   * @return void
   */
  void computeLinkComJ(
    const unsigned& index, 
    Matrix<Scalar, 3, Dynamic>& jacobianV,
    Matrix<Scalar, 3, Dynamic>& jacobianW,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Solves the inverse kinematics for the center of mass X-Y while 
   * keeping the feet constrained on the ground.
   *
   * @param desCom: Desired center of mass position, only xy matters here
   * @param baseLimb: Support limb
   * @param eeIndex: Index of the support foot end-effector
   * @param type: Type of joints
   * 
   * @return Joint values of the whole body
   */
  Matrix<Scalar, Dynamic, 1> solveComIkTwoVar(
    const Matrix<Scalar, 3, 1>& desCom,
    const unsigned& baseLimb, 
    const unsigned& eeIndices, 
    const JointStateType& type);

  /**
   * Solves the inverse kinematics for the center of mass given the
   * required base limb to move it.
   *
   * @param baseLimb: Index of the support limb.
   * @param comVelocityD: Desired Com velocity.
   * @param limbMotionSpace: Desired limbs motion type, that is,
   *   cartesian or joint space.
   * @param limbVelocitiesD: Desired limb joint/cartesian space
   *   velocities depending upon the motion space.
   * @param eeIndices: Indices of desired end effectors for each limb.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of whole body joints.
   */
  Matrix<Scalar, Dynamic, 1> solveComIK(const unsigned& baseLimb,
    const Matrix<Scalar, 6, 1>& comVelocityD,
    const vector<unsigned>& limbMotionSpace,
    const vector<Matrix<Scalar, Dynamic, 1>>& limbVelocitiesD, 
    const vector<int>& eeIndices,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Solves the inverse kinematics for the given chain and end-effector
   * using pseudo inverse of jacobian.
   *
   * @param chainIndex: Index of the end-effector chain.
   * @param eeIndex: The end effector transformation index.
   * @param targetT: The desired configuration of the end effector.
   * @param startType: Type of joints to find the initial position of 
   *   end-effector.
   * 
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of the chain joints.
   */
  Matrix<Scalar, Dynamic, 1> solveJacobianIK(const unsigned& chainIndex,
    const unsigned& eeIndex, const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100,
    const JointStateType& startType = JointStateType::ACTUAL, const bool& solveForOrientation = true,
    const Scalar& pTol = 1e-2,
    const Scalar& oTol = 0.5);

  /**
   * Makes a posture task for the given active joints
   *
   * @param targetJoints: Target joints for the body
   *
   * @return PostureTaskPtr
   */
  boost::shared_ptr<PostureTask<Scalar> > makePostureTask(
    const Matrix<Scalar, Dynamic, 1>& targetJoints,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1e-6,
    const Scalar& gain = 0.85
  );

  /**
   * Makes a contact task for the given chain and end-effector
   *
   * @param chainIndex: Chain index
   * @param eeIndex: End-effector index
   *
   * @return ContactTaskPtr
   */
  boost::shared_ptr<ContactTask<Scalar> > makeContactTask(
    const unsigned& chainIndex,
    const unsigned& eeIndex,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 10,
    const Scalar& gain = 0.9
  );

  /**
   * Makes a cartesian task for the given chain and end-effector
   *
   * @param chainIndex: Chain index
   * @param eeIndex: End-effector index
   * @param targetT: Target transformation for the end-effector
   *
   * @return CartesianTaskPtr
   */
  boost::shared_ptr<CartesianTask<Scalar> > makeCartesianTask(
    const unsigned& chainIndex,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 4>& targetT,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1,
    const Scalar& gain = 0.9
  );

  /**
   * Makes a com task for the given active joints
   *
   * @param chainIndex: Chain index
   * @param eeIndex: End-effector index
   * @param targetT: Target transformation for the end-effector
   *
   * @return ComTaskPtr
   */
  boost::shared_ptr<ComTask<Scalar> > makeComTask(
    // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
    const unsigned& refFrame,
    const Matrix<Scalar, 3, 1>& comTarget,
    vector<bool> activeJoints = vector<bool>(),
    const Scalar& weight = 1,
    const Scalar& gain = 0.9
  );

  /**
   * Makes a torso task for the given active joints
   *
   * @param refFrame: Base frame chain index
   * @param eeIndex: Base end-effector index
   * @param target: Target transformation for the torso
   *
   * @return TorsoTaskPtr
   */
  boost::shared_ptr<TorsoTask<Scalar> > makeTorsoTask(
    const unsigned& refFrame,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 4>& target,
    vector<bool> activeJoints,
    const Scalar& weight,
    const Scalar& gain);

  /**
   * @brief solveCartesianIK: Solves the inverse kinematics for the
   *   given cartesian task using taskIkSolver
   * @param chainIndex: Index of the cartesian chain
   * @param eeIndex: Index of the end-effector
   * @param targetT: Target transformation for the end-effector
   * @param maxIterations: Max number of iterations to solve ik
   * @return ik-solved joint values
   */
  Matrix<Scalar, Dynamic, 1> solveCartesianIK(
    const unsigned& chainIndex,
    const unsigned& eeIndex,
    const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100);

  /**
   * @brief solveTasksIK: Solves the inverse kinematics for the
   *   all the given weighted tasks using taskIkSolver
   * @param tasks: Tasks
   * @param maxIterations: Max number of iterations to solve ik
   * @return ik-solved joint values
   */
  Matrix<Scalar, Dynamic, 1> solveTasksIK(
    const vector<boost::shared_ptr<MotionTask<Scalar> > >& tasks,
    const unsigned& maxIterations);

  /**
   * Solves the inverse kinematics for the given chain and end-effector
   * using pseudo inverse of jacobian.
   *
   * @param chainIndex: Index of the end-effector chain.
   * @param endEffector: The end effector transformation.
   * @param targetT: The desired configuration of the end effector.
   * @param startType: Type of joints to find the initial position of 
   *   end-effector.
   * 
   * @return Matrix<Scalar, Dynamic, 1>: Ik values of the chain joints.
   */
  Matrix<Scalar, Dynamic, 1> solveJacobianIK(const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector, const Matrix<Scalar, 4, 4>& targetT,
    const unsigned& maxIterations = 100,
    const JointStateType& startType = JointStateType::ACTUAL, const bool& solveForOrientation = true,
    const Scalar& pTol = 1e-2,
    const Scalar& oTol = 0.5);

  /**
   * Finds the specified limb mass matrix with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, Dynamic, Dynamic> computeMassMatrix(
    const unsigned& chainIndex, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Finds the specified limb joint torques by performing newton euler
   * recursion with given external force and torques.
   *
   * @param chainIndex: Index of the limb.
   * @param extForces: External forces.
   * @param extMoments: External moments.
   * @param totalForces: Container for joint forces.
   * @param totalMoments: Container for joint forces.
   * @param supportLeg: The leg base to be used as the inertial frame 
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, Dynamic, 1> returns the joint torques
   */
  Matrix<Scalar, Dynamic, 1> newtonEulerForces(const unsigned& chainIndex,
    const Matrix<Scalar, 3, 1>& extForces, const Matrix<Scalar, 3, 1>& extMoments,
    Matrix<Scalar, 3, 1>& totalForces, Matrix<Scalar, 3, 1>& totalMoments, 
    const unsigned& supportLeg,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Computes the Zmp of the robot by solving the system dynamics using
   * Newton Euler approach.
   *
   * @param supportLeg: Index of the support leg.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, 2, 1>
   */
   
  Matrix<Scalar, 2, 1> computeZmp(
    const unsigned& supportLeg,
    const JointStateType& type = JointStateType::ACTUAL
  );

  /**
   * Computes the Zmp of the robot from feet force sensors
   *
   * @param refFrame: Index of the reference foot frame 
   * 
   * @return Matrix<Scalar, 2, 1>
   */
  Matrix<Scalar, 2, 1> computeFsrZmp(const unsigned& refFrame);

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getForwardEffector(const unsigned& chainIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation matrix from
   *   the final frame of the chain.
   * @param type: Type of joints
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getForwardEffector(const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4>& endEffector,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns the required joint
   *
   * @param index: Joint Index
   * 
   * @return Joint
   */
  boost::shared_ptr<Joint<Scalar> > getJoint(const unsigned& index);
  
  /**
   * Returns the state of the required joint
   *
   * @param index: Joint Index
   * @param type: Type of joints
   * 
   * @return Joint
   */
  boost::shared_ptr<JointState<Scalar> > getJointState(
    const unsigned& index, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns the current position of the joint
   *
   * @param index: Joint Index
   * @param type: Type of joints
   *
   * @return Joint position
   */
  Scalar getJointPosition(
    const unsigned& index,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns all the required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   * 
   * @return Vector<boost::shared_ptr<Joint<Scalar> > >
   */
  vector<boost::shared_ptr<Joint<Scalar> > > getJoints(
    const unsigned& startIndex,
    const unsigned& nElements);
  
  /**
   * Returns all the states of all required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   * 
   * @return Vector<boost::shared_ptr<JointState<Scalar> > >
   */
  vector<boost::shared_ptr<JointState<Scalar> > > getJointStates(
    const unsigned& startIndex,
    const unsigned& nElements,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns all the positions of all required joints
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   *
   * @return Vector<Scalar, Dynamic, 1>
   */
  Matrix<Scalar, Dynamic, 1> getJointPositions(
    const unsigned& startIndex,
    const unsigned& nElements,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Gets the joint positions for the required chain
   *
   * @param chainIndex: Chain of which the values are required
   * @param type: Type of joints.
   * 
   * @return a vector containing the joint values.
   */
  vector<boost::shared_ptr<JointState<Scalar> > > getChainStates(
    const unsigned& chainIndex, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns the required link
   *
   * @param index: Index of the link
   *
   * @return LinkInfo
   */
  boost::shared_ptr<LinkInfo<Scalar> > 
  getLink(const unsigned& index);
  
  /**
   * Returns the required chain
   *
   * @param index: Index of the chain
   *
   * @return LinkChain
   */
  boost::shared_ptr<LinkChain<Scalar> > 
  getLinkChain(const unsigned& index);

  /**
   * Returns the end effectors of the required chain
   *
   * @param chain: Chain index
   * @param index: End effector index
   *
   * @return Matrix<typename Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4>  getEndEffector(
    const unsigned& chain, const unsigned& index);

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Analytical inverse kinematics for left leg of the robot.
   *
   * @param endEffector: The end effector transformation.
   * @param targetT: The desired configuration of the end effector
   *
   * @return vector<Matrix<Scalar, Dynamic, 1>>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<Matrix<Scalar, Dynamic, 1>> inverseLeftLeg(
    const Matrix<Scalar, 4, 4>& endEffector, 
    const Matrix<Scalar, 4, 4>& targetT);

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Analytical inverse kinematics for right leg of the robot.
   *
   * @param endEffector: The end effector transformation.
   * @param targetpoint: The desired configuration of
   *   the end effector
   *
   * @return vector<Matrix<Scalar, Dynamic, 1>>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<Matrix<Scalar, Dynamic, 1>> inverseRightLeg(
    const Matrix<Scalar, 4, 4>& endEffector, 
    const Matrix<Scalar, 4, 4>& targetT);

  /**
   * This function sets the required end-effector for the left and
   * right legs for now. Update if needed.
   *
   * @param chain: Chain whose end-effector is
   *   to be setup.
   * @param eeIndex: Index of the saved end-effector to
   *   be used.
   * @param ee: The position of the end-effector from the last frame
   *   of the chain.
   */
  void setEndEffector(
    const unsigned& chain, 
    const unsigned& eeIndex, 
    const Matrix<Scalar, 4, 1>& ee);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle).
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector index
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void computeComWrtBase(
    const unsigned& limbIndex,
    const unsigned& eeIndex, 
    Matrix<Scalar, 3, 1>& comVector,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle) in X-Y plane.
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector index
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void computeComWrtBase(
    const unsigned& limbIndex,
    const unsigned& eeIndex, 
    Matrix<Scalar, 2, 1>& comVector,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * This function gets the center of mass with respect to
   * a base frame (Torso, left ankle or right ankle).
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector.
   * @param type: Type of joints
   *
   * @return Output center of mass vector
   */
  Matrix<Scalar, 3, 1> computeComWrtBase(
    const unsigned& limbIndex,
    const unsigned& eeIndex,
    const JointStateType& type = JointStateType::ACTUAL);

  bool computeVirtualMass(
    const unsigned& chainIndex,
    const Matrix<Scalar, 3, 1>& direction, 
    const Matrix<Scalar, 4, 4>& endEffector, 
    Scalar& virtualMass,
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * Returns the tranformation matrix from torso to center
   *   of the feet.
   *
   * @return Matrix<Scalar, 4, 4>
   */
  Matrix<Scalar, 4, 4> getFeetCenterT();

  /**
   * Returns the transformed vector from foot frame to camera frame
   * 
   * @param camIndex: top or bottom cam index
   * @param posInFoot: position vector in foot frame
   * 
   * @return Matrix<Scalar, 4, 1>
   */ 
  Matrix<Scalar, 4, 1> getWorldToCam(
    const unsigned& camIndex, const Matrix<Scalar, 4, 1>& posInFoot);

  /**
   * Returns the current best estimate of com state relative to the given
   * base frame.
   *
   * @param baseFrame: Target reference frame 
   * @param eeIndex: Target frame end-effector index
   * 
   * @return ComState<Scalar>
   */
  ComState<Scalar> getComStateWrtFrame(
    const unsigned& baseFrame = -1, 
    const unsigned& eeIndex = 0);
  
  /**
   * Returns the process model used for estimating center of mass
   * state in the given direction
   * @param index: Index that specifies the directions -> 0=x, 1=y
   * @return boost::shared_ptr<ProcessModel<Scalar> >
   */
  boost::shared_ptr<ProcessModel<Scalar> >
  getComModel(const unsigned& index);

  /**
   * Returns the current state of the torso
   *
   * @return boost::shared_ptr<TorsoState<Scalar> >
   */
  boost::shared_ptr<TorsoState<Scalar> > getTorsoState();
  
  /**
   * Returns the foot that is on ground. -1 if none.
   *
   * @return int
   */
  int getFootOnGround();

  /**
   * Returns the motion thread cycle time.
   *
   * @return Scalar
   */
  Scalar getCycleTime();

  unsigned getNJoints()
  {
    return NUM_JOINTS;
  }

  /**
   * Function to print all the kinematic data for debugging.
   */
  void printKinematicData();

  /**
   * Function to setup the plot if needed.
   */
  void setupJointsPlot();

  /**
   * Function to plot the state of a joint.
   *
   * @param index: Index of the joint.
   * @param type: Type of joint
   *   (ACTUAL, SIMULATED, VALIDATION)
   */
  void
  plotJointState(
    const unsigned& index, 
    const JointStateType& type = JointStateType::ACTUAL);
private:
  /**
   * Sets up the joints, links, and link chains
   */
  void init();
  
  /**
   * Sets up the robot links and their inertias
   */
  void setupLinksAndInertias();

  /**
   * Sets up the robot joints and their states
   */ 
  void setupJoints();

  /**
   * Sets up the robot chains
   */ 
  void setupChains();

  /**
   * Sets up the robot torso and center of mass states
   */ 
  void setupWBStates();

  /**
   * Sets up the robot head chain
   */ 
  void setupHeadChain();
  
  /**
   * Sets up the robot left and right arm chains
   */ 
  void setupArmChains();

  /**
   * Sets up the robot left and right leg chains
   */ 
  void setupLegChains();
  
  /**
   * Sets up the robot chain end effectors
   */
  void setupEndEffectors();

  /**
   * Updates the joint states for the given cycle from memory
   */
  void updateJointStates();

  /**
   * Updates the robot torso state based on sensor readings
   */
  void updateTorsoState();
  
  /**
   * Updates the robot center of mass state based on sensor readings
   */
  void updateComState();

  /**
   * Updates the transformation of the camera's with respect to the
   * foot.
   */
  void updateFootToCamT();

  /**
   * Updates the transformation of the left and right feet from torso.
   */
  void updateTorsoToFeet();

  /**
   * Updates which foot is on the ground.
   */
  void updateFootOnGround();
  
  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Prepares the kinematic model by finding the dh matrix with
   * respect to the joint positions.
   *
   * @param ch: the chains to be prepared.
   * @param type: Type of joints
   * 
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  void prepareDHTransforms(
    const unsigned& ch = CHAINS_SIZE, 
    const JointStateType& type = JointStateType::ACTUAL);

  /**
   * The function calculates the center of mass of the robot with
   * respect to the torso frame of the robot.
   *
   * @param type = Type of joints
   *
   * @return Matrix<Scalar, 3, 1>
   */
  Matrix<Scalar, 3, 1> calculateCenterOfMass(
    const JointStateType& type = JointStateType::ACTUAL);

  //! Base class object.
  MotionModule* motionModule;

  //! Motion cycle time
  Scalar cycleTime;
  
  //! Torso acceleration of the robot in X-Y-Z
  boost::shared_ptr<TorsoState<Scalar> > torsoState;
  
  //! Com state of the robot wrt the torso
  boost::shared_ptr<ComState<Scalar> > comState;

  //! KF-based com state estimator
  vector<boost::shared_ptr<ComEstimator<Scalar> > > comEstimator;
  
  //! Total mass of body chains.
  Scalar totalChainsMass;
  
  //! Joints vector
  vector<boost::shared_ptr<Joint<Scalar> > > joints;
  
  //! Links info vector
  vector<boost::shared_ptr<LinkInfo<Scalar> > > links;
  
  //! Link chains vector
  vector<boost::shared_ptr<LinkChain<Scalar> > > linkChains;

  //! Left foot transformation in torso frame.
  Matrix<Scalar, 4, 4> lFootOnGround;

  //! Right foot transformation in torso frame.
  Matrix<Scalar, 4, 4> rFootOnGround;

  //! Upper cam transformation in feet frame.
  Matrix<Scalar, 4, 4> upperCamInFeet;

  //! Lower cam transformation in feet frame.
  Matrix<Scalar, 4, 4> lowerCamInFeet;

  //! The variable that defines whether right or left or both feet are 
  //! on the ground
  int footOnGround;

  //! Torso pitch offset
  Scalar torsoPitchOffset;

  //! Feet forces s buffer
  boost::circular_buffer<Matrix<Scalar, 2, 1>> feetForcesBuffer;

  //! Feet forces buffer size
  size_t ffBufferSize;

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code.
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  Matrix<Scalar, 4, 4> tBaseLLegInv, tEndLLegInv, rotFixLLeg, rotRLeg;

  //! File streams for data-logging.
  fstream jointStateLog;

  //! Log file path;
  string jointStateLogPath;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KinematicsModule<MType> > KinematicsModulePtr;
