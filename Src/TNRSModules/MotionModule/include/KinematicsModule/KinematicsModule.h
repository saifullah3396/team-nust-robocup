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

#include <boost/circular_buffer.hpp>
#include "ConfigManager/include/ConfigMacros.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "MotionModule/include/KinematicsModule/LinkInertiaConsts.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/MotionModule.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/EnvConsts.h"
#include "Utils/include/HardwareIds.h"
//#include "Utils/include/GnuPlotter.h"

using namespace Utils;

static const string jointNames[24] =
  { "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw",
    "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
    "RElbowRoll", "RWristYaw", "LHipYawPitch", "LHipRoll", "LHipPitch",
    "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll",
    "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll" };

/**
 * @class KinematicsModule
 * @brief The class for kinematics and dynamics of the robot.
 */
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
  ~KinematicsModule()
  {
  }

  /**
   * Enum that determines which jointvalues are to be used to prepare
   *   the dhtransformation matrices.
   */
  enum JointUsageType
  {
    ACTUAL,
    SIM,
    VALID,
    NUM_JOINTUSAGE_TYPES
  };

  /**
   * Updates the kinematics model of the robot based
   * on position sensor data.
   */
  void
  updateModel();

  /**
   * Sets the simulated joint states equal to actual joint state
   */
  void
  setSimToActual()
  {
    jointPositions[SIM] = jointPositions[ACTUAL];
    prepareDHTransforms(CHAINS_SIZE, SIM);
  }

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void
  setJointPositions(
    const unsigned& startIndex, 
    const VectorXf& simPosition, 
    const JointUsageType& type = ACTUAL)
  {
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(startIndex + simPosition.size() <= NUM_JOINTS);
    jointPositions[type].segment(startIndex, simPosition.size()) = simPosition;
    prepareDHTransforms(CHAINS_SIZE, type);
  }

  /**
   * Sets the simulated joint positions for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param type: Type of joints.
   */
  void
  setChainPositions(
    const unsigned& chainIndex,
    const VectorXf& simPosition,
    const JointUsageType& type = ACTUAL)
  {
    unsigned size = chainSizes[chainIndex];
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(simPosition.size() == size);
    jointPositions[type].segment(chainStarts[chainIndex], size) = simPosition;
    prepareDHTransforms(chainIndex, type);
  }

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void
  setJointVelocities(
    const unsigned& startIndex,
    const VectorXf& simVelocities,
    const JointUsageType& type = ACTUAL)
  {
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(startIndex + simVelocities.size() <= NUM_JOINTS);
    jointVelocities[type].segment(startIndex, simVelocities.size()) =
      simVelocities;
    prepareDHTransforms(CHAINS_SIZE, type);
  }

  /**
   * Sets the simulated joint velocities for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simVelocities: Simulated joint velocities.
   * @param type: Type of joints
   */
  void
  setChainVelocities(
    const unsigned& chainIndex,
    const VectorXf& simVelocities,
    const JointUsageType& type = ACTUAL)
  {
    unsigned size = chainSizes[chainIndex];
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(simVelocities.size() == size);
    jointVelocities[type].segment(chainStarts[chainIndex], size) =
      simVelocities;
    prepareDHTransforms(chainIndex, type);
  }

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void
  setJointAccelerations(
    const unsigned& startIndex,
    const VectorXf& simAccelerations,
    const JointUsageType& type = ACTUAL)
  {
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(startIndex + simAccelerations.size() <= NUM_JOINTS);
    jointAccelerations[type].segment(startIndex, simAccelerations.size()) =
      simAccelerations;
    prepareDHTransforms(CHAINS_SIZE, type);
  }

  /**
   * Sets the simulated joint accelerations for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simAccelerations: Simulated joint Accelerations.
   * @param type: Type of joints
   */
  void
  setChainAccelerations(
    const unsigned& chainIndex,
    const VectorXf& simAccelerations,
    const JointUsageType& type = ACTUAL)
  {
    unsigned size = chainSizes[chainIndex];
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(simAccelerations.size() == size);
    jointAccelerations[type].segment(chainStarts[chainIndex], size) =
      simAccelerations;
    prepareDHTransforms(chainIndex, type);
  }

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param startIndex: First joint index.
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints 
   */
  void
  setJointState(
    const unsigned& startIndex,
    const VectorXf& simPosition, 
    const VectorXf& simVelocity,
    const VectorXf& simAcceleration,
    const JointUsageType& type = ACTUAL)
  {
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    ASSERT(startIndex + simPosition.size() <= NUM_JOINTS);
    ASSERT(
      simPosition.size() == simVelocity.size() && simPosition.size() == simAcceleration.size());
    jointPositions[type].segment(startIndex, simPosition.size()) = simPosition;
    jointVelocities[type].segment(startIndex, simVelocity.size()) = simVelocity;
    jointAccelerations[type].segment(startIndex, simAcceleration.size()) =
      simAcceleration;
    prepareDHTransforms(CHAINS_SIZE, type);
  }

  /**
   * Sets the simulated joint states for simulated calculations.
   *
   * @param chainIndex: Chain to update the joint values of
   * @param simPosition: Simulated joint positions.
   * @param simVelocity: Simulated joint positions.
   * @param simAcceleration: Simulated joint positions.
   * @param type: Type of joints
   */
  void
  setChainState(
    const unsigned& chainIndex,
    const VectorXf& simPosition, 
    const VectorXf& simVelocity,
    const VectorXf& simAcceleration,
    const JointUsageType& type = ACTUAL)
  {
    if(type == ACTUAL) {
      cout << "Cannot set actual values from user." << endl;
      return;
    }
    unsigned size = chainSizes[chainIndex];
    ASSERT(simPosition.size() == size);
    ASSERT(
      simPosition.size() == simVelocity.size() && simPosition.size() == simAcceleration.size());
    jointPositions[type].segment(chainStarts[chainIndex], size) = simPosition;
    jointVelocities[type].segment(chainStarts[chainIndex], size) = simVelocity;
    jointAccelerations[type].segment(chainStarts[chainIndex], size) =
      simAcceleration;
    prepareDHTransforms(chainIndex, type);
  }

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
   * @return Matrix<float, Dynamic, 1>
   */
  VectorXf
  cartToJointVels(const unsigned& chainIndex,
    const VectorXf cVels, const Matrix4f endEffector,
    const JointUsageType& type = ACTUAL);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Matrix<float, Dynamic, Dynamic>
   */
  MatrixXf
  computeLimbJ(const unsigned& chainIndex,
    const unsigned& eeIndex,
    const JointUsageType& type = ACTUAL);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation
   *   matrix from the final frame of the chain
   * @param type: Type of joints
   * 
   * @return Matrix<float, Dynamic, Dynamic>
   */
  MatrixXf
  computeLimbJ(const unsigned& chainIndex,
    const Matrix4f& endEffector,
    const JointUsageType& type = ACTUAL);

  /**
   * Finds the specified limb jacobian with respect to a given
   * end-effector vector.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector translation
   *   from the final frame of the chain
   * @param type: Type of joints
   * 
   * @return Matrix<float, Dynamic, Dynamic>
   */
  MatrixXf
  computeLimbJ(const unsigned& chainIndex,
    const Vector4f& endEffector,
    const JointUsageType& type = ACTUAL);

  /**
   * Finds the specified limb Center of mass jacobian with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   * 
   * @return Matrix<float, Dynamic, Dynamic>
   */
  MatrixXf
  computeLimbComJ(const unsigned& chainIndex, const JointUsageType& type = ACTUAL);

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
   * @return VectorXf: Ik values of whole body joints.
   */
  VectorXf
  solveComIK(const unsigned& baseLimb,
    const Matrix<float, 6, 1>& comVelocityD,
    const vector<unsigned>& limbMotionSpace,
    const vector<VectorXf>& limbVelocitiesD, 
    const vector<int>& eeIndices,
    const JointUsageType& type = ACTUAL);

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
   * @return VectorXf: Ik values of the chain joints.
   */
  VectorXf
  solveJacobianIK(const unsigned& chainIndex,
    const unsigned& eeIndex, const Matrix4f& targetT,
    const unsigned& maxIterations = 100,
    const unsigned& startType = ACTUAL);

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
   * @return VectorXf: Ik values of the chain joints.
   */
  VectorXf
  solveJacobianIK(const unsigned& chainIndex,
    const Matrix4f& endEffector, const Matrix4f& targetT,
    const unsigned& maxIterations = 100,
    const unsigned& startType = ACTUAL);

  /**
   * Finds the specified limb mass matrix with respect to
   * the base of the limb.
   *
   * @param chainIndex: Index of the limb.
   * @param type: Type of joints
   * 
   * @return Matrix<float, Dynamic, Dynamic>
   */
  MatrixXf
  computeMassMatrix(const unsigned& chainIndex, const JointUsageType& type = ACTUAL);

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
   * @return Matrix<float, Dynamic, 1> returns the joint torques
   */
  VectorXf
  newtonEulerForces(const unsigned& chainIndex,
    const Vector3f& extForces, const Vector3f& extMoments,
    Vector3f& totalForces, Vector3f& totalMoments, 
    const unsigned& supportLeg,
    const JointUsageType& type = ACTUAL);

  /**
   * Computes the Zmp of the robot by solving the system dynamics using
   * Newton Euler approach.
   *
   * @param supportLeg: Index of the support leg.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Vector2f
   */
   
  Vector2f computeZmp(
    const unsigned& supportLeg,
    const JointUsageType& type = ACTUAL
  );

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param eeIndex: The end effector index.
   * @param type: Type of joints
   * 
   * @return Matrix<float, 4, 4>
   */
  Matrix4f
  getForwardEffector(const unsigned& chainIndex,
    const unsigned& eeIndex,
    const JointUsageType& type = ACTUAL);

  /**
   * Forward Kinematics Solver.
   *
   * @param chainIndex: Index of the limb.
   * @param endEffector: The end effector transformation matrix from
   *   the final frame of the chain.
   * @param type: Type of joints
   * @return Matrix<float, 4, 4>
   */
  Matrix4f
  getForwardEffector(const unsigned& chainIndex,
    const Matrix4f& endEffector,
    const JointUsageType& type = ACTUAL);

  /**
   * This function gets the transformation matrix for the required
   * link (or joint).
   *
   * @param index: Index of the link.
   * @param type: Type of joints
   * 
   * @return Matrix<float, 4, 4>
   */
  Matrix4f
  getLinkT(const unsigned index, const unsigned type = ACTUAL)
  {
    return linkTs[type][index];
  }

  /**
   * This function gets the center of mass vector for the required
   * link (or joint).
   *
   * @param index: Index of the link.
   *
   * @return Matrix<float, 4, 1>
   */
  Vector4f
  getLinkCom(const unsigned& index)
  {
    return linkComs[index];
  }

  /**
   * This function gets the mass of the required link (or joint).
   *
   * @param index: Index of the link.
   *
   * @return float
   */
  float
  getLinkMass(const unsigned& index)
  {
    return linkMasses[index];
  }

  /**
   * This function gets the inertia tensor of the required
   * link (or joint).
   *
   * @param index: Index of the link.
   *
   * @return Matrix<float, 3, 3>
   */
  Matrix3f
  getLinkInertia(const unsigned& index)
  {
    return linkInertias[index];
  }

  /**
   * This function gets the position of the required joint.
   *
   * @param index: Index of the link.
   * @param type: Type of joints
   * 
   * @return float
   */
  float
  getJointPosition(const unsigned& index, const JointUsageType& type = ACTUAL)
  {
    return jointPositions[type][index];
  }

  /**
   * This function gets the positions of all the requried joints.
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   * 
   * @return Vector<float, Dynamic, Dynamic>
   */

  VectorXf
  getJointPositions(const unsigned& startIndex,
    const unsigned& nElements,
    const JointUsageType& type = ACTUAL)
  {
    ASSERT(startIndex + nElements <= NUM_JOINTS);
    return jointPositions[type].segment(startIndex, nElements);
  }

  /**
   * Gets the joint positions for the required chain
   *
   * @param chainIndex: Chain of which the values are required
   * @param type: Type of joints.
   * 
   * @return a vector containing the joint values.
   */
  VectorXf
  getChainPositions(const unsigned& chainIndex, const JointUsageType& type = ACTUAL)
  {
    unsigned size = chainSizes[chainIndex];
    return jointPositions[type].segment(chainStarts[chainIndex], size);
  }

  /**
   * Gets the joint velocity limits for the required chain
   * 
   * @param chainIndex: Chain of which the values are required
   *
   * @return a vector containing the joint velocity limits
   */
  VectorXf
  getChainVelLimits(const unsigned& chainIndex)
  {
    unsigned size = chainSizes[chainIndex];
    return jointVLimits.segment(chainStarts[chainIndex], size);
  }

  /**
   * This function gets the velocity of the required joint.
   *
   * @param index: Index of the link.
   * @param type: Type of joints
   * 
   * @return float
   */
  float
  getJointVelocity(const unsigned& index, const JointUsageType& type = ACTUAL)
  {
    return jointVelocities[type][index];
  }

  /**
   * This function gets the velocities of all the required joints.
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   * 
   * @return Vector<float, Dynamic, Dynamic>
   */

  VectorXf
  getJointVelocities(const unsigned& startIndex,
    const unsigned& nElements,
    const JointUsageType& type = ACTUAL)
  {
    ASSERT(startIndex + nElements <= NUM_JOINTS);
    return jointVelocities[type].segment(startIndex, nElements);
  }

  /**
   * This function gets the acceleration of the required joint.
   *
   * @param index: Index of the link.
   * @param type: Type of joints
   * 
   * @return float
   */
  float
  getJointAcceleration(const unsigned& index, const JointUsageType& type = ACTUAL)
  {
    return jointAccelerations[type][index];
  }

  /**
   * This function gets the accelerations of all the requried joint.
   *
   * @param startIndex: First joint index.
   * @param nElements: Number of elements including the
   *   first one.
   * @param type: Type of joints
   * 
   * @return Vector<float, Dynamic, Dynamic>
   */

  VectorXf
  getJointAccelerations(const unsigned& startIndex,
    const unsigned& nElements,
    const JointUsageType& type = ACTUAL)
  {
    ASSERT(startIndex + nElements <= NUM_JOINTS);
    return jointAccelerations[type].segment(startIndex, nElements);
  }

  /**
   * Returns the end effector of the required chain.
   *
   * @param chain: The limb.
   * @param index: Index of te end effector.
   *
   * @return Matrix<float, 4, 4>
   */
  Matrix4f
  getEndEffector(const unsigned& chain, const unsigned& index)
  {
    return endEffectors[chain][index];
  }

  /**
   * Returns the size of the given chain.
   *
   * @param index: Index of the chain.
   *
   * @return unsigned
   */
  unsigned
  getChainSize(const unsigned& index)
  {
    return chainSizes[index];
  }

  /**
   * Returns the start of the given chain.
   *
   * @param index: Index of the chain.
   *
   * @return unsigned
   */
  unsigned
  getChainStart(const unsigned& index)
  {
    return chainStarts[index];
  }

  /**
   * Returns the starting transformation of the given chain.
   *
   * @param index: Index of the chain.
   *
   * @return Matrix4f
   */
  Matrix4f
  getInitT(const unsigned& index)
  {
    return initTs[index];
  }

  /**
   * Returns the ending transformation of the given chain.
   *
   * @param index: Index of the chain.
   *
   * @return Matrix4f
   */
  Matrix4f
  getFinalT(const unsigned& index)
  {
    return finalTs[index];
  }

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code but redefined using Eigen.
   *
   * Analytical inverse kinematics for left leg of the robot.
   *
   * @param endEffector: The end effector transformation.
   * @param targetT: The desired configuration of the end effector
   *
   * @return vector<VectorXf>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<VectorXf>
  inverseLeftLeg(const Matrix4f& endEffector, const Matrix4f& targetT);

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
   * @return vector<VectorXf>
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  vector<VectorXf>
  inverseRightLeg(const Matrix4f& endEffector, const Matrix4f& targetT);

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
  void
  setEndEffector(const unsigned& chain, const unsigned& eeIndex,
    const Vector4f& ee);

  /**
   * This function gets the center of mass with respect to
   * a base frame (floatorso, left ankle or right ankle).
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector.
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void
  getComWrtBase(const unsigned& limbIndex,
    const unsigned& eeIndex, Vector3f& comVector,
    const JointUsageType& type = ACTUAL);

  /**
   * This function gets the center of mass with respect to
   * a base frame (floatorso, left ankle or right ankle) in X-Y plane.
   *
   * @param limbIndex: The base limb for com.
   * @param eeIndex: End effector.
   * @param comVector: Output Vector
   * @param type: Type of joints
   */
  void
  getComWrtBase(const unsigned& limbIndex,
    const unsigned& eeIndex, Vector2f& comVector,
    const JointUsageType& type = ACTUAL);

  bool
  computeVirtualMass(const unsigned& chainIndex,
    const Vector3f& direction, const Matrix4f& endEffector, float& virtualMass,
    const JointUsageType& type = ACTUAL);

  /**
   * Returns the tranformation matrix from torso to center
   *   of the feet.
   *
   * @return Matrix<float, 4, 4>
   */
  Matrix4f
  getFeetCenterT();

  /**
   * Returns the transformed vector from foot frame to camera frame
   * 
   * @param camIndex: top or bottom cam index
   * @param posInFoot: position vector in foot frame
   * 
   * @return Vector4f
   */ 
  Vector4f
  getWorldToCam(
    const unsigned& camIndex, const Vector4f& posInFoot);

  /**
   * Returns the current best estimate of com state relative to the given
   * base frame.
   *
   * @return ComState
   */
  ComState 
  getComState(const unsigned& baseFrame);
  
  /**
   * Returns the current state of the torso
   *
   * @return TorsoState
   */
  TorsoState getTorsoState() { return torsoState; }

  /**
   * Returns the current velocity of the torso
   *
   * @return Vector3f
   */
  Vector3f getTorsoVel() { return torsoState.velocity; }
  
    /**
   * Returns the current acceleration of the torso
   *
   * @return Vector3f
   */
  Vector3f getTorsoAccel() { return torsoState.accel; }

  /**
   * Returns the current rotation of the torso
   *
   * @return Matrix3f
   */
  Matrix3f getTorsoRot() { return torsoState.rot; }

  /**
   * Returns the foot that is on ground. -1 if none.
   *
   * @return int
   */
  int
  getFootOnGround()
  {
    return footOnGround;
  }

  /**
   * Returns the motion thread cycle time.
   *
   * @return float
   */
  float
  getCycleTime()
  {
    return cycleTime;
  }

  /**
   * Function to print all the kinematic data for debugging.
   */
  void
  printKinematicData();

  /**
   * Function to setup the plot if needed.
   */
  void
  setupJointsPlot();

  /**
   * Function to plot the state of a joint.
   *
   * @param index: Index of the joint.
   * @param type: Type of joint
   *   (ACTUAL, SIMULATED, VALIDATION)
   */
  void
  plotJointState(const unsigned& index, const JointUsageType& type = ACTUAL);
private:
  /**
   * The cycle time for update of velocity and accelerations.
   */
  float cycleTime;

  /**
   * Initiates the frame transformations, link coms,
   * masses, and inertias and the DH matrices.
   */
  void
  initKinematicsModel();

  /**
   * Updates the joint positions from the sensor values from
   * shared memory.
   *
   * @param const vector<float> &jointPositionSensor: sensor positions
   */
  void
  updateJointPositions();

  /**
   * Updates the joint velocities by differentiating the joint
   * positions.
   */
  void
  updateJointVelocities();

  /**
   * Updates the joint velocities by differentiating the joint
   * velocities.
   */
  void
  updateJointAccelerations();

  /**
   * Updates the robot torso state based on sensor readings
   */
  void
  updateTorsoState();

  /**
   * Updates the transformation of the camera's with respect to the
   * foot.
   */
  void
  updateFootToCamT();

  /**
   * Updates the transformation of the left and right feet from torso.
   */
  void
  updateTorsoToFeet();

  /**
   * Updates which foot is on the ground.
   */
  void
  updateFootOnGround();
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
  void
  prepareDHTransforms(const unsigned& ch = CHAINS_SIZE, 
    const JointUsageType& type = ACTUAL);

  /**
   * The function calculates the center of mass of the robot with
   * respect to the torso frame of the robot.
   *
   * @param type = Type of joints
   *
   * @return Matrix<float, 3, 1>
   */
  Vector3f
  calculateCenterOfMass(const JointUsageType& type = ACTUAL);

  //! Base class object.
  MotionModule* motionModule;

  //! Left foot transformation in torso frame.
  Matrix4f lFootOnGround;

  //! Right foot transformation in torso frame.
  Matrix4f rFootOnGround;

  //! Upper cam transformation in feet frame.
  Matrix4f upperCamInFeet;

  //! Lower cam transformation in feet frame.
  Matrix4f lowerCamInFeet;

  //! A vector of link transformation matrices (ACTUAL, SIMULATED,
  vector<vector<Matrix4f> > linkTs;

  //! Initial constant transformations for the chains
  vector<Matrix4f> initTs;

  //! Final constant transformations for the chains
  vector<Matrix4f> finalTs;

  //! End effector for each chain.
  vector<vector<Matrix4f> > endEffectors;

  //! A vector of link center of mass vectors.
  vector<Vector4f> linkComs;

  //! A vector of link masses.
  vector<float> linkMasses;

  //! Sizes of body chains.
  vector<unsigned> chainSizes;

  //! Start indices of body chains.
  vector<unsigned> chainStarts;

  //! Total mass of body chains.
  float totalChainsMass;

  //! Masses of body chains.
  vector<float> chainMasses;

  //! A vector of link inertia tensor matrices.
  vector<Matrix3f> linkInertias;

  //! A vector of joint positions in the current iteration.
  vector<VectorXf> jointPositions;

  //! A vector of joint positions in the previous iteration.
  vector<VectorXf> prevJointPositions;

  //! A vector of joint velocities in the current iteratin.
  vector<VectorXf> jointVelocities;

  //! A vector of joint velocities in the previous iteration.
  vector<VectorXf> prevJointVelocities;

  //! A vector of joint accelerations in the current iteration.
  vector<VectorXf> jointAccelerations;

  //! A vector of joint upper limits.
  VectorXf jointULimits;

  //! A vector of joint lower limits.
  VectorXf jointLLimits;

  //! A vector of joint velocity limits.
  VectorXf jointVLimits;

  //! Torso acceleration of the robot in X-Y-Z
  TorsoState torsoState;

  //! The variable that gives whether right or left or both feet are on
  int footOnGround;

  //! Torso pitch offset
  float torsoPitchOffset;

  //! Feet forces value buffer.
  boost::circular_buffer<Vector2f> feetForcesBuffer;

  //! Feet forces buffer size
  size_t ffBufferSize;

  /**
   * This part of the code has been adapted from Kofinas'
   * NAOKinematics code.
   *
   * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
   * @author Vosk
   */
  Matrix4f tBaseLLegInv, tEndLLegInv, rotFixLLeg, rotRLeg;

  //! GnuPlot object
  //Gnuplot gp;

  //! File streams for data-logging.
  fstream jointStateLog;

  //! Log file path;
  string jointStateLogPath;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KinematicsModule> KinematicsModulePtr;
