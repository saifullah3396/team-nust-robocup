/**
 * @file MotionModule/KinematicsModule/KinematicsModule.cpp
 *
 * this file implements the class for solving the kinematics
 * of the robot.cout
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

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/ComEstimator.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "MotionModule/include/KinematicsModule/LinkInertiaConsts.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/LinkInfo.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "ControlModule/include/ActuatorRequests.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/EnvConsts.h"

#define JOINT_STATE(index, type) \
  joints[index]->states[(unsigned)type]
  
#define JOINT_T(index, type) \
  joints[index]->states[(unsigned)type]->trans

#define JOINT_T_IN_BASE(index, type) \
  joints[index]->states[(unsigned)type]->transInBase

template <typename Scalar>
KinematicsModule<Scalar>::KinematicsModule(MotionModule* motionModule) :  
  MemoryBase(motionModule), 
  motionModule(motionModule),
  joints(NUM_JOINTS),
  linkChains(CHAINS_SIZE),
  links(NUM_LINKS), // + 1 for torso link 
  footOnGround(L_FOOT),
  ffBufferSize(15)
{
  GET_CONFIG( "KinCalibration", (Scalar, torsoPitchOffset, torsoPitchOffset), )
  init();
  //setupJointsPlot();
}

template <typename Scalar>
KinematicsModule<Scalar>::~KinematicsModule() {}

template <typename Scalar>
void KinematicsModule<Scalar>::init()
{
  cycleTime = motionModule->getPeriodMinMS() / ((Scalar) 1000);
  setupLinksAndInertias();
  setupJoints();
  setupChains();
  setupWBStates();
  feetForcesBuffer.set_capacity(ffBufferSize);
  updateJointStates();
  prepareDHTransforms();
}

template <typename Scalar>
void KinematicsModule<Scalar>::update()
{
  //auto tStart = high_resolution_clock::now();
  updateJointStates();
  updateTorsoState();
  updateComState();
  updateFootToCamT();
  updateFootOnGround();
  updateTorsoToFeet();
  prepareDHTransforms();
  //duration<double> timeSpan = high_resolution_clock::now() - tStart;
  //LOG_INFO("KM Time: " << timeSpan.count() << "seconds.");
  /*static fstream zmpLog;
  Matrix<Scalar, 2, 1> zmp = computeFsrZmp(CHAIN_L_LEG);
  zmpLog.open(
    (ConfigManager::getLogsDirPath() + string("KM/Zmp.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpLog << zmp[0] << " " << zmp[1] << endl;
  zmpLog.close();*/
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupLinksAndInertias()
{
  Matrix<Scalar, 3, 3> iMat = Matrix<Scalar, 3, 3>::Identity();
  for (size_t i = 0; i < NUM_LINKS; ++i) {
    links[i] = 
      boost::shared_ptr<LinkInfo<Scalar> > (
        new LinkInfo<Scalar>()
      );
    for (size_t j = 0; j < 3; ++j) {
      for (unsigned k = 0; k < 3; ++k) {
        links[i]->inertia(j, k) = InertiaMatrices[j + i * 3][k];
      }
    }
  }
  //! Torso mass and center of mass definitions
  links[TORSO]->com = Matrix<Scalar, 4, 1>(torsoX, torsoY, torsoZ, 1.0f);
  links[TORSO]->mass = torsoMass;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupJoints()
{
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    DHParams<Scalar>* params = 
      new DHParams<Scalar>(
        jointDHConsts[i][0], 
        jointDHConsts[i][1], 
        jointDHConsts[i][2], 
        jointDHConsts[i][3]
      );
    joints[i] = 
      boost::shared_ptr<Joint<Scalar> > (
        new Joint<Scalar>(
          jointNameConsts[i],
          jointMaxPositionConsts[i],
          jointMinPositionConsts[i],
          jointMaxVelocityConsts[i],
          params)
      );
    joints[i]->link = links[i];
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupChains()
{
  for (size_t i = 0; i < CHAINS_SIZE; ++i) {
    linkChains[i] = 
      boost::shared_ptr<LinkChain<Scalar> > (
        new LinkChain<Scalar>()
      );
  }
  
  linkChains[CHAIN_HEAD]->start = HEAD_YAW;
  linkChains[CHAIN_L_ARM]->start = L_SHOULDER_PITCH;
  linkChains[CHAIN_R_ARM]->start = R_SHOULDER_PITCH;
  linkChains[CHAIN_L_LEG]->start = L_HIP_YAW_PITCH;
  linkChains[CHAIN_R_LEG]->start = R_HIP_YAW_PITCH;

  linkChains[CHAIN_HEAD]->size = HEAD_SIZE;
  linkChains[CHAIN_L_ARM]->size = L_ARM_SIZE;
  linkChains[CHAIN_R_ARM]->size = R_ARM_SIZE;
  linkChains[CHAIN_L_LEG]->size = L_LEG_SIZE;
  linkChains[CHAIN_R_LEG]->size = R_LEG_SIZE;
  
  for (size_t i = 0; i < CHAINS_SIZE; ++i) {
    for (size_t j = linkChains[i]->start; 
         j < linkChains[i]->start+linkChains[i]->size; ++j) 
    {
      links[j]->chain = linkChains[i];
    }
  }
  
  setupHeadChain();
  setupArmChains();
  setupLegChains();
  setupEndEffectors();
  
  Matrix<Scalar, 3, 3> iMat = Matrix<Scalar, 3, 3>::Identity();
  //!Moving Inertias of all joints to to center of mass
  for (size_t i = 0; i < NUM_LINKS; ++i) {
    links[i]->inertia =
      links[i]->inertia - links[i]->mass * ((links[i]->com.segment(0, 3).transpose() * links[i]->com.segment(
        0,
        3))(0, 0) * iMat - links[i]->com.segment(0, 3) * links[i]->com.segment(0, 3).transpose());
  }

  Scalar totalChainsMass = 0;
  for (int i = 0; i < CHAINS_SIZE; ++i) {
    linkChains[i]->mass = 0;
    for (int j = 0; j < linkChains[i]->size; ++j) {
      linkChains[i]->mass += links[linkChains[i]->start + j]->mass;
      totalChainsMass += links[linkChains[i]->start + j]->mass;
    }
  }
  LinkChain<Scalar>::totalChainsMass = totalChainsMass;
  
  //! Update links partial masses used in determining com jacobian
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joints[i]->link->partialMass = joints[i]->link->mass / totalChainsMass;
  }
  
  /* Inertia verification
   * 
   * vector<Matrix<Scalar, 3, 1> > comsGlobal(NUM_JOINTS);
  Matrix<Scalar, 4, 4> T;
  unsigned chainStart = 0;
  unsigned n = 0;
  for (int i = 0; i < CHAINS_SIZE; ++i) {
    T = linkChains[i]->startT;
    for (int j = 0; j < linkChains[i]->size; ++j) {
      T = T * joints[linkChains[i]->start + j]->states[ACTUAL]->T;
      Matrix<Scalar, 3, 1> com = (T * links[linkChains[i]->size + j]->com).block(0, 0, 3, 1);
      comsGlobal[chainStart + j] = com;
      ++n;
    }
    chainStart = n;
  }

  for (size_t i = 0; i < NUM_LINKS; ++i) {
    cout << "Joint[" << jointNames[i] << "]" << endl;
    cout << "com" << endl;
    cout << comsGlobal[i] << endl;
    //cout << "inertia" << endl;
    //cout << 
    //  inertiaTrans[i].transpose() * 
    //  linkInertias[i] * 
    //  inertiaTrans[i] << endl;
  }*/
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupWBStates()
{
  torsoState = 
    boost::shared_ptr<TorsoState<Scalar> > (
      new TorsoState<Scalar>()
    );
  comState = 
    boost::shared_ptr<ComState<Scalar> > (
      new ComState<Scalar>()
    );
  comEstimator.push_back(//X
    boost::shared_ptr<ComEstimator<Scalar> > (new ComEstimator<Scalar>())
  );
  comEstimator.push_back(//Y
    boost::shared_ptr<ComEstimator<Scalar> > (new ComEstimator<Scalar>())
  );
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupHeadChain()
{
  Matrix<Scalar, 4, 4> t1;
  //! Chain start transformation
  MathsUtils::makeTranslation(
    linkChains[CHAIN_HEAD]->startT,
    (Scalar) 0.0,
    (Scalar) 0.0,
    (Scalar) neckOffsetZ);
  //! Chain end transformation
  MathsUtils::makeRotationXYZ(
    linkChains[CHAIN_HEAD]->endT,
    (Scalar) M_PI_2,
    (Scalar) M_PI_2,
    (Scalar) 0.0);
  //!Masses
  links[HEAD_YAW]->mass = headYawMass;
  links[HEAD_PITCH]->mass = headPitchMass;
  //!Center of mass vectors.
  links[HEAD_YAW]->com = Matrix<Scalar, 4, 1>(headYawX, headYawY, headYawZ, 1.0f);
  links[HEAD_PITCH]->com = Matrix<Scalar, 4, 1>(headPitchX, headPitchY, headPitchZ, 1.0f);
  
  //!Fixing the coordinate system of center of mass.
  links[HEAD_PITCH]->inertiaTrans = linkChains[CHAIN_HEAD]->endT.block(0, 0, 3, 3);
  links[HEAD_PITCH]->com = linkChains[CHAIN_HEAD]->endT * links[HEAD_PITCH]->com;

  //!Transforming inertia tensor from the given frame to the joint frame.
  links[HEAD_PITCH]->inertia = 
    links[HEAD_PITCH]->inertiaTrans * 
    links[HEAD_PITCH]->inertia * 
    links[HEAD_PITCH]->inertiaTrans.transpose();
  //!----------------------Head End------------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupArmChains()
{
  Matrix<Scalar, 4, 4> t1;
  //! Chain start transformation
  MathsUtils::makeTranslation(
    linkChains[CHAIN_R_ARM]->startT,
    (Scalar) 0.0,
    (Scalar) -(shoulderOffsetY),
    (Scalar) shoulderOffsetZ);
  
  //! Chain end transformation
  MathsUtils::makeRotationXYZ(
    linkChains[CHAIN_R_ARM]->endT,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  linkChains[CHAIN_R_ARM]->endT(0, 3) = 0.f;
  linkChains[CHAIN_R_ARM]->endT(1, 3) = -handOffsetZ;
  linkChains[CHAIN_R_ARM]->endT(2, 3) = handOffsetX;
  
  //! Masses
  links[R_SHOULDER_PITCH]->mass = rShoulderPitchMass;
  links[R_SHOULDER_ROLL]->mass = rShoulderRollMass;
  links[R_ELBOW_YAW]->mass = rElbowYawMass;
  links[R_ELBOW_ROLL]->mass = rElbowRollMass;
  links[R_WRIST_YAW]->mass = rWristYawMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) 0.0, (Scalar) 0.0);
  links[R_SHOULDER_PITCH]->com = Matrix<Scalar, 4, 1>(
    rShoulderPitchX,
    rShoulderPitchY,
    rShoulderPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  links[R_SHOULDER_PITCH]->com = t1 * links[R_SHOULDER_PITCH]->com;

  links[R_SHOULDER_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //links[L_SHOULDER_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);

  //!Fixing the Inertia tensor rotation.
  links[R_SHOULDER_PITCH]->inertia =
    links[R_SHOULDER_PITCH]->inertiaTrans * 
    links[R_SHOULDER_PITCH]->inertia * 
    links[R_SHOULDER_PITCH]->inertiaTrans.transpose();

  /**
   * Both left and right have same transformations
   * hence Rotation Matrices
   */
  links[L_SHOULDER_PITCH]->inertia =
    links[L_SHOULDER_PITCH]->inertiaTrans * 
    links[L_SHOULDER_PITCH]->inertia * 
    links[L_SHOULDER_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[R_SHOULDER_ROLL]->com = Matrix<Scalar, 4, 1>(
    rShoulderRollX,
    rShoulderRollY,
    rShoulderRollZ,
    1.0f);

  links[R_SHOULDER_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[L_SHOULDER_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_SHOULDER_ROLL]->com = t1 * links[R_SHOULDER_ROLL]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_SHOULDER_ROLL]->inertia =
    links[R_SHOULDER_ROLL]->inertiaTrans * 
    links[R_SHOULDER_ROLL]->inertia * 
    links[R_SHOULDER_ROLL]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[L_SHOULDER_ROLL]->inertia =
    links[L_SHOULDER_ROLL]->inertiaTrans * 
    links[L_SHOULDER_ROLL]->inertia * 
    links[L_SHOULDER_ROLL]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[R_ELBOW_YAW]->com = Matrix<Scalar, 4, 1>(rElbowYawX, rElbowYawY, rElbowYawZ, 1.0f);


  links[R_ELBOW_YAW]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[L_ELBOW_YAW]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_ELBOW_YAW]->com = t1 * links[R_ELBOW_YAW]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_ELBOW_YAW]->inertia =
    links[R_ELBOW_YAW]->inertiaTrans * 
    links[R_ELBOW_YAW]->inertia * 
    links[R_ELBOW_YAW]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[L_ELBOW_YAW]->inertia =
    links[L_ELBOW_YAW]->inertiaTrans * 
    links[L_ELBOW_YAW]->inertia * 
    links[L_ELBOW_YAW]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[R_ELBOW_ROLL]->com = Matrix<Scalar, 4, 1>(
    rElbowRollX,
    rElbowRollY,
    rElbowRollZ,
    1.0f);


  links[R_ELBOW_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[L_ELBOW_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_ELBOW_ROLL]->com = t1 * links[R_ELBOW_ROLL]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_ELBOW_ROLL]->inertia =
    links[R_ELBOW_ROLL]->inertiaTrans *
    links[R_ELBOW_ROLL]->inertia *
    links[R_ELBOW_ROLL]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[L_ELBOW_ROLL]->inertia =
    links[L_ELBOW_ROLL]->inertiaTrans * 
    links[L_ELBOW_ROLL]->inertia * 
    links[R_ELBOW_ROLL]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) -M_PI_2, (Scalar) 0.0, (Scalar) -M_PI_2);
  links[R_WRIST_YAW]->com = Matrix<Scalar, 4, 1>(rWristYawX, rWristYawY, rWristYawZ, 1.0f);

  links[R_WRIST_YAW]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[L_WRIST_YAW]->inertiaTrans = t1.block(0, 0, 3, 3);

  //!Fixing the coordinate system of center of mass.
  links[R_WRIST_YAW]->com = t1 * links[R_WRIST_YAW]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_WRIST_YAW]->inertia =
    links[R_WRIST_YAW]->inertiaTrans * 
    links[R_WRIST_YAW]->inertia * 
    links[R_WRIST_YAW]->inertiaTrans.transpose();

  /**
   * Both left and right have same Scalarransformations
   * hence Rotation Matrices
   */
  links[L_WRIST_YAW]->inertia =
    links[L_WRIST_YAW]->inertiaTrans * 
    links[L_WRIST_YAW]->inertia * 
    links[L_WRIST_YAW]->inertiaTrans.transpose();
  //!-------------------Right Arm End----------------------!//

    //!-------------------Left Arm Start--------------------!//

  //!End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[CHAIN_L_ARM]->startT,
    (Scalar) 0.0,
    (Scalar) shoulderOffsetY,
    (Scalar) shoulderOffsetZ);
  linkChains[CHAIN_L_ARM]->endT = linkChains[CHAIN_R_ARM]->endT;

  //!Masses and center of mass vectors
  for (size_t i = 0; i < L_ARM_SIZE; i++) {
    links[linkChains[CHAIN_L_ARM]->start + i]->com =
      links[linkChains[CHAIN_R_ARM]->start + i]->com;
    links[linkChains[CHAIN_L_ARM]->start + i]->mass =
      links[linkChains[CHAIN_R_ARM]->start + i]->mass;
  }

  //!Fixing the center of mass coordinates
  links[L_SHOULDER_PITCH]->com(2) = -links[L_SHOULDER_PITCH]->com(2);
  links[L_SHOULDER_ROLL]->com(0) = -links[L_SHOULDER_ROLL]->com(0);
  links[L_ELBOW_YAW]->com(0) = -links[L_ELBOW_YAW]->com(0);
  links[L_ELBOW_ROLL]->com(0) = -links[L_ELBOW_ROLL]->com(0);
  links[L_WRIST_YAW]->com(0) = -links[L_WRIST_YAW]->com(0);

  //!-------------------Left Arm End--------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupLegChains()
{
  Matrix<Scalar, 4, 4> t1;
  //!------------------Right Leg Start------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[CHAIN_R_LEG]->startT,
    (Scalar) 0.0,
    (Scalar) -hipOffsetY,
    (Scalar) -hipOffsetZ);
  MathsUtils::makeRotationZYX(
    linkChains[CHAIN_R_LEG]->endT,
    (Scalar) M_PI,
    (Scalar) -M_PI_2,
    (Scalar) 0.0);
  rotRLeg = linkChains[CHAIN_R_LEG]->endT;

  //!Masses
  links[R_HIP_YAW_PITCH]->mass = rHipYawPitchMass;
  links[R_HIP_ROLL]->mass = rHipRollMass;
  links[R_HIP_PITCH]->mass = rHipPitchMass;
  links[R_KNEE_PITCH]->mass = rKneePitchMass;
  links[R_ANKLE_PITCH]->mass = rAnklePitchMass;
  links[R_ANKLE_ROLL]->mass = rAnkleRollMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -M_PI_2 / 2,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[R_HIP_YAW_PITCH]->com = Matrix<Scalar, 4, 1>(
    rHipYawPitchX,
    rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);
  
  links[R_HIP_YAW_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[R_HIP_YAW_PITCH]->com = t1 * links[R_HIP_YAW_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_HIP_YAW_PITCH]->inertia =
    links[R_HIP_YAW_PITCH]->inertiaTrans * 
    links[R_HIP_YAW_PITCH]->inertia * 
    links[R_HIP_YAW_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI, (Scalar) M_PI_2, (Scalar) 0.0);
  links[R_HIP_ROLL]->com = Matrix<Scalar, 4, 1>(rHipRollX, rHipRollY, rHipRollZ, 1.0f);

  links[R_HIP_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_HIP_ROLL]->com = t1 * links[R_HIP_ROLL]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_HIP_ROLL]->inertia =
   links[R_HIP_ROLL]->inertiaTrans * 
   links[R_HIP_ROLL]->inertia * 
   links[R_HIP_ROLL]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[R_HIP_PITCH]->com = Matrix<Scalar, 4, 1>(rHipPitchX, rHipPitchY, rHipPitchZ, 1.0f);

  links[R_HIP_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_HIP_PITCH]->com = t1 * links[R_HIP_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_HIP_PITCH]->inertia =
    links[R_HIP_PITCH]->inertiaTrans * 
    links[R_HIP_PITCH]->inertia * 
    links[R_HIP_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[R_KNEE_PITCH]->com = Matrix<Scalar, 4, 1>(
    rKneePitchX,
    rKneePitchY,
    rKneePitchZ,
    1.0f);

  links[R_KNEE_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_KNEE_PITCH]->com = t1 * links[R_KNEE_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_KNEE_PITCH]->inertia =
    links[R_KNEE_PITCH]->inertiaTrans * 
    links[R_KNEE_PITCH]->inertia *  
    links[R_KNEE_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[R_ANKLE_PITCH]->com = Matrix<Scalar, 4, 1>(
    rAnklePitchX,
    rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  links[R_ANKLE_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_ANKLE_PITCH]->com = t1 * links[R_ANKLE_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_ANKLE_PITCH]->inertia =
    links[R_ANKLE_PITCH]->inertiaTrans * 
    links[R_ANKLE_PITCH]->inertia * 
    links[R_ANKLE_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  links[R_ANKLE_ROLL]->com = Matrix<Scalar, 4, 1>(
    rAnkleRollX,
    rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  links[R_ANKLE_ROLL]->inertiaTrans = linkChains[CHAIN_R_LEG]->endT.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[R_ANKLE_ROLL]->com = linkChains[CHAIN_R_LEG]->endT * links[R_ANKLE_ROLL]->com;

  //!Fixing the Inertia tensor rotation.
  links[R_ANKLE_ROLL]->inertia =
    links[R_ANKLE_ROLL]->inertiaTrans * 
    links[R_ANKLE_ROLL]->inertia * 
    links[R_ANKLE_ROLL]->inertiaTrans.transpose();
  //!------------------Right Leg Start------------------!//

  //!------------------Left Leg Start-------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    linkChains[CHAIN_L_LEG]->startT,
    (Scalar) 0.0,
    (Scalar) hipOffsetY,
    (Scalar) -hipOffsetZ);
  tBaseLLegInv = linkChains[CHAIN_L_LEG]->startT;
  tBaseLLegInv = MathsUtils::getTInverse(tBaseLLegInv);
  MathsUtils::makeRotationZYX(
    linkChains[CHAIN_L_LEG]->endT,
    (Scalar) M_PI,
    (Scalar) -M_PI_2,
    (Scalar) 0.0);
  MathsUtils::makeRotationXYZ(
    rotFixLLeg,
    (Scalar) M_PI_4,
    (Scalar) 0.0,
    (Scalar) 0.0);
  tEndLLegInv = t1;
  tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);

  //!Masses
  links[L_HIP_YAW_PITCH]->mass = rHipYawPitchMass;
  links[L_HIP_ROLL]->mass = rHipRollMass;
  links[L_HIP_PITCH]->mass = rHipPitchMass;
  links[L_KNEE_PITCH]->mass = rKneePitchMass;
  links[L_ANKLE_PITCH]->mass = rAnklePitchMass;
  links[L_ANKLE_ROLL]->mass = rAnkleRollMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (Scalar) -(3 * M_PI) / 4,
    (Scalar) 0.0,
    (Scalar) -M_PI_2);
  links[L_HIP_YAW_PITCH]->com = Matrix<Scalar, 4, 1>(
    rHipYawPitchX,
    -rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);
  
  links[L_HIP_YAW_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  links[L_HIP_YAW_PITCH]->com = t1 * links[L_HIP_YAW_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[L_HIP_YAW_PITCH]->inertia =
    links[L_HIP_YAW_PITCH]->inertiaTrans * 
    links[L_HIP_YAW_PITCH]->inertia * 
    links[L_HIP_YAW_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI, (Scalar) M_PI_2, (Scalar) 0.0);
  links[L_HIP_ROLL]->com = Matrix<Scalar, 4, 1>(rHipRollX, -rHipRollY, rHipRollZ, 1.0f);
  
  links[L_HIP_ROLL]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[L_HIP_ROLL]->com = t1 * links[L_HIP_ROLL]->com;

  //!Fixing the Inertia tensor rotation.
  links[L_HIP_ROLL]->inertia =
    links[L_HIP_ROLL]->inertiaTrans * 
    links[L_HIP_ROLL]->inertia * 
    links[L_HIP_ROLL]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[L_HIP_PITCH]->com = Matrix<Scalar, 4, 1>(rHipPitchX, -rHipPitchY, rHipPitchZ, 1.0f);

  links[L_HIP_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[L_HIP_PITCH]->com = t1 * links[L_HIP_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[L_HIP_PITCH]->inertia =
    links[L_HIP_PITCH]->inertiaTrans * 
    links[L_HIP_PITCH]->inertia * 
    links[L_HIP_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[L_KNEE_PITCH]->com = Matrix<Scalar, 4, 1>(
    rKneePitchX,
    -rKneePitchY,
    rKneePitchZ,
    1.0f);

  links[L_KNEE_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[L_KNEE_PITCH]->com = t1 * links[L_KNEE_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[L_KNEE_PITCH]->inertia =
    links[L_KNEE_PITCH]->inertiaTrans * 
    links[L_KNEE_PITCH]->inertia *
    links[L_KNEE_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (Scalar) M_PI_2, (Scalar) M_PI_2, (Scalar) 0.0);
  links[L_ANKLE_PITCH]->com = Matrix<Scalar, 4, 1>(
    rAnklePitchX,
    -rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  links[L_ANKLE_PITCH]->inertiaTrans = t1.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[L_ANKLE_PITCH]->com = t1 * links[L_ANKLE_PITCH]->com;

  //!Fixing the Inertia tensor rotation.
  links[L_ANKLE_PITCH]->inertia =
    links[L_ANKLE_PITCH]->inertiaTrans * 
    links[L_ANKLE_PITCH]->inertia * 
    links[L_ANKLE_PITCH]->inertiaTrans.transpose();

  //!Center of mass vectors.
  links[L_ANKLE_ROLL]->com = Matrix<Scalar, 4, 1>(
    rAnkleRollX,
    -rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  links[L_ANKLE_ROLL]->inertiaTrans = linkChains[CHAIN_L_LEG]->endT.block(0, 0, 3, 3);
  //!Fixing the coordinate system of center of mass.
  links[L_ANKLE_ROLL]->com = linkChains[CHAIN_L_LEG]->endT * links[L_ANKLE_ROLL]->com;


  //!Fixing the Inertia tensor rotation.
  links[L_ANKLE_ROLL]->inertia =
    links[L_ANKLE_ROLL]->inertiaTrans * 
    links[L_ANKLE_ROLL]->inertia * 
    links[L_ANKLE_ROLL]->inertiaTrans.transpose();

  //!------------------Left Leg End-------------------!//
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupEndEffectors()
{
  Matrix<Scalar, 4, 4> t1;
  linkChains[CHAIN_HEAD]->endEffectors.resize(NUM_CAMS);
  linkChains[CHAIN_L_LEG]->endEffectors.resize(NUM_LEG_EES);
  linkChains[CHAIN_R_LEG]->endEffectors.resize(NUM_LEG_EES);
  linkChains[CHAIN_L_ARM]->endEffectors.push_back(Matrix<Scalar, 4, 4>::Identity());
  linkChains[CHAIN_R_ARM]->endEffectors.push_back(Matrix<Scalar, 4, 4>::Identity());
  MathsUtils::makeTranslation(
    linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM],
    (Scalar) cameraTopX,
    (Scalar) 0.0,
    (Scalar) cameraTopZ);
  MathsUtils::makeTranslation(
    linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM],
    (Scalar) cameraBottomX,
    (Scalar) 0.0,
    (Scalar) cameraBottomZ);
  MathsUtils::makeRotationXYZ(t1, (Scalar) 0.0, (Scalar) M_PI_2, (Scalar) -M_PI_2);
  linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM] = linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM] * t1;
  linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM] =
    linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM] * t1;
  MathsUtils::makeRotationXYZ(t1, (Scalar) -cameraTopAngleY, //Y becomes -X after frame transfomration
    (Scalar) 0.0,
    (Scalar) 0.0);
  linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM] = linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM] * t1;
  MathsUtils::makeRotationXYZ(t1, (Scalar) -cameraBotAngleY, //Y becomes -X after frame transfomration
    (Scalar) 0.0,
    (Scalar) 0.0);
  linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM] =
    linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM] * t1;

  //cout << "Camera EEs" << endl;
  //cout << linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM] << endl;
  //cout << linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM] << endl;

  linkChains[CHAIN_L_LEG]->endEffectors[ANKLE] = Matrix<Scalar, 4, 4>::Identity();
  linkChains[CHAIN_R_LEG]->endEffectors[ANKLE] = Matrix<Scalar, 4, 4>::Identity();
  MathsUtils::makeTranslation(
    linkChains[CHAIN_L_LEG]->endEffectors[FEET_BASE],
    (Scalar) 0.0,
    (Scalar) 0.0,
    (Scalar) -footHeight);
  linkChains[CHAIN_R_LEG]->endEffectors[FEET_BASE] = linkChains[CHAIN_L_LEG]->endEffectors[FEET_BASE];
}

template <typename Scalar>
void KinematicsModule<Scalar>::printKinematicData()
{
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  cout << "-------------------________________Printing Kinematic Data________________-------------------" << endl;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cout << "Joint[" << joints[i]->name << "]" << endl;
    cout << "Position:" << JOINT_STATE(i, JointStateType::ACTUAL)->position * 180 / M_PI << ",  ";
    cout << "Velocity:" << JOINT_STATE(i, JointStateType::ACTUAL)->velocity << ",  ";
    cout << "Acceleration:" << JOINT_STATE(i, JointStateType::ACTUAL)->accel << endl;
    cout << "Com: [" << links[i]->com.format(OctaveFmt) << " ,  Mass:" << links[i]->mass << endl;
    cout << "Inertias: " << endl << links[i]->inertia.format(OctaveFmt) << endl << " ------------------------ " << endl;
    cout << "DHMatrix: " << endl << JOINT_STATE(i, JointStateType::ACTUAL)->trans << endl << " ------------------------ " << endl;
  }
  cout << "[Torso]" << endl;
  cout << "Com: [" << links[NUM_JOINTS]->com.format(OctaveFmt) << " ,  Mass:" << links[NUM_JOINTS]->mass << endl;
  cout << "Inertia: " << endl << links[NUM_JOINTS]->inertia.format(OctaveFmt) << endl;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setupJointsPlot()
{
  jointStateLogPath = ConfigManager::getLogsDirPath() + "KinematicsModule/JointState.txt";
  jointStateLog.open(
    jointStateLogPath.c_str(),
    std::ofstream::out | std::ofstream::trunc);
  jointStateLog.close();
  //gp << "set xrange [0:20]\nset yrange [0:20]\n";
  //gp << "plot '" << jointStateLogPath << "' using 1:2 with lines title 'Joint Position'" << endl;
}

template <typename Scalar> 
void KinematicsModule<Scalar>::plotJointState(const unsigned& index, const JointStateType& type)
{
  Scalar time = motionModule->getModuleTime();
  jointStateLog.open(jointStateLogPath.c_str(), fstream::app | fstream::out);
  jointStateLog << time
                << " " 
                << joints[index]->states[(unsigned)type]->position 
                << " " 
                << joints[index]->states[(unsigned)type]->velocity 
                << " " 
                << joints[index]->states[(unsigned)type]->accel 
                << "\n";
  jointStateLog.close();
  //cout << "time: " << time << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:2 with lines title 'Joint Position'" << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:3 with lines title 'Joint Velocity'" << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:4 with lines title 'Joint Acceleration'" << endl;
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateJointStates()
{
  try {
    auto& posSensors = 
      OVAR(vector<float>, MotionModule::jointPositionSensors);
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      JOINT_STATE(i, JointStateType::ACTUAL)->prevPosition = JOINT_STATE(i, JointStateType::ACTUAL)->position;
      JOINT_STATE(i, JointStateType::ACTUAL)->prevVelocity = JOINT_STATE(i, JointStateType::ACTUAL)->velocity;
      JOINT_STATE(i, JointStateType::ACTUAL)->position = posSensors[i];
      JOINT_STATE(i, JointStateType::ACTUAL)->velocity = 
        (JOINT_STATE(i, JointStateType::ACTUAL)->position - JOINT_STATE(i, JointStateType::ACTUAL)->prevPosition) / 
        cycleTime;
      JOINT_STATE(i, JointStateType::ACTUAL)->accel = 
        (JOINT_STATE(i, JointStateType::ACTUAL)->velocity - JOINT_STATE(i, JointStateType::ACTUAL)->prevVelocity) / 
        cycleTime;
    }
  } catch (const exception& e) {
    cout << e.what();
  }
}

/**
 * Inertial sensors enum defined in Utils/include/HardwareIds.h
  GYROSCOPE_X = 0,
  GYROSCOPE_Y,
  GYROSCOPE_Z,
  TORSO_ANGLE_X,
  TORSO_ANGLE_Y,
  TORSO_ANGLE_Z,
  ACCELEROMETER_X,
  ACCELEROMETER_Y,
  ACCELEROMETER_Z,
 */ 
 
template <typename Scalar>
void KinematicsModule<Scalar>::updateTorsoState()
{
  try {
    // Need a kalman filter for this tracker.
    // Nao sensors are updated almost with cycle time of 40ms in memory
    // Whereas motion module runs at 10ms
    auto& inertial = OVAR(vector<float>, MotionModule::inertialSensors);
    torsoState->accel[0] = inertial[ACCELEROMETER_X];
    torsoState->accel[1] = inertial[ACCELEROMETER_Y];
    torsoState->accel[2] = inertial[ACCELEROMETER_Z];
    MathsUtils::makeRotationXYZ(
      torsoState->rot,
      (Scalar) inertial[TORSO_ANGLE_X],
      (Scalar) inertial[TORSO_ANGLE_Y],
      0.f);
    //cout << "inertial[TORSO_ANGLE_X]: " << inertial[TORSO_ANGLE_X] * 180 / M_PI<< endl;
    //cout << "inertial[TORSO_ANGLE_Y]: " << inertial[TORSO_ANGLE_Y] * 180 / M_PI << endl;
    //cout << "torso Accel:\n" << torsoState->accel << endl;
    //cout << "torsoState->rot:\n" << torsoState->rot.block(0, 0, 3, 3) << endl;
    torsoState->accel = torsoState->rot.block(0, 0, 3, 3) * torsoState->accel;
    //cout << "torso Accel rotated:\n" << torsoState->accel << endl;
    torsoState->velocity = torsoState->velocity + torsoState->accel * cycleTime;
    //cout << "torsoState->rot:\n" << torsoState->rot << endl;
    //cout << "torso velocity:\n" << torsoState->velocity << endl;
    //cout << "torso accel:\n" << torsoState->accel << endl;
  } catch (const exception& e) {
    cout << e.what();
  }
}

template <typename Scalar>
void KinematicsModule<Scalar>::updateComState()
{
  unsigned baseFrame = CHAIN_L_LEG;
  Matrix<Scalar, 3, 1> comFoot;
  Matrix<Scalar, 2, 1> fsrZmp = computeFsrZmp(baseFrame);
  computeComWrtBase(baseFrame, FEET_BASE, comFoot);
  comState->position[2] = (Scalar) comFoot[2];
  comState->baseFrame = baseFrame;
  comState->eeIndex = FEET_BASE;
  for (int i = 0; i < comEstimator.size(); ++i) {
    if (!comEstimator[i]->isInitiated()) {
      Matrix<Scalar, Dynamic, 1> state(3, 1);
      state << comFoot[i], 0.0, 0.0;
      comEstimator[i]->init(state, comFoot[2], cycleTime);
    } else {
      comEstimator[i]->updateComHeight(comFoot[2]);
      Matrix<Scalar, Dynamic, 1> meas(3, 1);
      //cout << "fsrZmp[" << i << "]: " << fsrZmp[i] << endl;

      meas << comFoot[i], torsoState->accel[i], fsrZmp[i];
      comEstimator[i]->update(meas);
      Matrix<Scalar, Dynamic, 1> est = comEstimator[i]->getState();
      comState->position[i] = est[0];
      comState->velocity[i] = est[1];
      comState->accel[i] = est[2];
      comState->zmp[i] = comEstimator[i]->getOutput()[0];
      //if (i==1) {
       // cout << "meas:\n" << meas << endl;
      //  cout << "pos: " << comState->position[i] << endl;
      //  cout << "acc: " << comState->accel[i] << endl;
      //  cout << "zmp: " << comState->zmp[i] << endl;
     // }
      /*LOG_INFO("Torso accel:")
      LOG_INFO(DataUtils::varToString(torsoState->accel[0]));
      LOG_INFO(DataUtils::varToString(torsoState->accel[1]));
      LOG_INFO("Zmp:")
      LOG_INFO(DataUtils::varToString(zmp.at<double>(0)))
      LOG_INFO(DataUtils::varToString(zmp.at<double>(1)))
      LOG_INFO(DataUtils::varToString(zmp.at<double>(2)))
      LOG_INFO(DataUtils::varToString(zmp.at<double>(3)))
      LOG_INFO(DataUtils::varToString(zmp.at<double>(4)))
      LOG_INFO(DataUtils::varToString(zmp.at<double>(5)))*/
    }
  }
}

template <typename Scalar>
void
KinematicsModule<Scalar>::prepareDHTransforms(const unsigned& ch, const JointStateType& type)
{
  if (ch == CHAINS_SIZE) {
    for (size_t c = 0; c < ch; ++c) {
      Matrix<Scalar, 4, 4> T = linkChains[c]->startT; // First transform 
      for (size_t i = linkChains[c]->start; i < linkChains[c]->start + linkChains[c]->size; ++i) {
        T = T * joints[i]->computeLinkTrans(type);
        // transformation from base torso
        joints[i]->setTransInBase(T, type);
      }
    }
  } else {
    for (size_t i = linkChains[ch]->start; i < linkChains[ch]->start + linkChains[ch]->size; ++i) {
      joints[i]->computeLinkTrans(type);
    }
  }
}

template <typename Scalar>
void
KinematicsModule<Scalar>::setEndEffector(const unsigned& chain, const unsigned& eeIndex,
  const Matrix<Scalar, 4, 1>& ee)
{
  Matrix<Scalar, 4, 4> t1;
  if (chain == CHAIN_R_LEG) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[chain]->endEffectors[eeIndex] = t1;
    MathsUtils::makeTranslation(t1, ee[0], -ee[1], ee[2]);
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  } else if (chain == CHAIN_L_LEG) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[chain]->endEffectors[eeIndex] = t1;
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  } else {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    linkChains[chain]->endEffectors[eeIndex] = t1;
  }
}

template <typename Scalar>
Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getForwardEffector(
  const unsigned& chainIndex, const Matrix<Scalar, 4, 4> &endEffector, const JointStateType& type)
{
  return JOINT_T_IN_BASE(
    linkChains[chainIndex]->start + linkChains[chainIndex]->size - 1, type) *
    linkChains[chainIndex]->endT * 
    endEffector;
}

template <typename Scalar>
Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getForwardEffector(
  const unsigned& chainIndex, const unsigned& eeIndex, const JointStateType& type)
{
  return getForwardEffector(chainIndex, linkChains[chainIndex]->endEffectors[eeIndex], type);
}

template <typename Scalar>
ComState<Scalar>
KinematicsModule<Scalar>::getComStateWrtFrame(
  const unsigned& baseFrame, 
  const unsigned& eeIndex)
{
  ComState<Scalar> tComState = *comState; // Get current com state
  // These readings will also require a kalman filter for good com state
  if (baseFrame == tComState.baseFrame) {
    return tComState;
  } else if (baseFrame >= 0 && baseFrame < CHAINS_SIZE) {
    Matrix<Scalar, 4, 4> T = 
      MathsUtils::getTInverse(getForwardEffector(baseFrame, eeIndex)) * getForwardEffector(tComState.baseFrame, tComState.eeIndex);
    tComState.position = MathsUtils::transformVector(T, tComState.position);
    tComState.velocity = T.block(0, 0, 3, 3) * tComState.velocity;
    tComState.accel = T.block(0, 0, 3, 3) * tComState.accel;
  }
  return tComState;
}

template <typename Scalar>
boost::shared_ptr<ProcessModel<Scalar> >
KinematicsModule<Scalar>::getComModel(const unsigned& index) {
  return comEstimator[index]->getModel();
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeComWrtBase(const unsigned& limbIndex,
  const unsigned& eeIndex, Matrix<Scalar, 3, 1> &comVector, const JointStateType& type)
{
  Matrix<Scalar, 3, 1> comWrtTorso = calculateCenterOfMass(type);
  if (limbIndex != -1) {
    comVector = MathsUtils::transformVector(
      MathsUtils::getTInverse(getForwardEffector(limbIndex, eeIndex, type)),
      comWrtTorso);
  } else {
    comVector = comWrtTorso;
  }
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeComWrtBase(const unsigned& limbIndex,
  const unsigned& eeIndex, Matrix<Scalar, 2, 1>& comVector, const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  computeComWrtBase(limbIndex, eeIndex, com, type);
  comVector[0] = com(0, 0);
  comVector[1] = com(1, 0);
}

template <typename Scalar>
Matrix<Scalar, 3, 1>
KinematicsModule<Scalar>::computeComWrtBase(const unsigned& limbIndex,
  const unsigned& eeIndex, const JointStateType& type)
{
  if (limbIndex != -1) {
    Matrix<Scalar, 3, 1> com =
      MathsUtils::transformVector(
        MathsUtils::getTInverse(getForwardEffector(limbIndex, eeIndex, type)),
        calculateCenterOfMass(type));
    return com;
  } else {
    return calculateCenterOfMass(type);
  }
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::cartToJointVels(
  const unsigned& chainIndex, const Matrix<Scalar, Dynamic, 1> cVels, const Matrix<Scalar, 4, 4> endEffector, const JointStateType& type)
{
  Matrix<Scalar, Dynamic, Dynamic> jacobian = computeLimbJ(chainIndex, endEffector, type);
  return MathsUtils::pseudoInverseSolve(jacobian, cVels);
}

template <typename Scalar>
Matrix<Scalar, 6, Dynamic>
KinematicsModule<Scalar>::computeLimbJ(const unsigned& chainIndex,
  const unsigned& eeIndex, const JointStateType& type)
{
  return computeLimbJ(
    chainIndex,
    linkChains[chainIndex]->endEffectors[eeIndex],
    type);
}

template <typename Scalar>
Matrix<Scalar, 6, Dynamic>
KinematicsModule<Scalar>::computeLimbJ(const unsigned& chainIndex,
  const Matrix<Scalar, 4, 4>& endEffector, const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  unsigned chainStart = linkChains[chainIndex]->start;
  Matrix<Scalar, 3, Dynamic> jacobianV;
  Matrix<Scalar, 3, Dynamic> jacobianW;
  jacobianV.resize(3, size);
  jacobianW.resize(3, size);
  Matrix<Scalar, 3, 1> eePos = 
    getForwardEffector(chainIndex, endEffector, type).template block<3, 1>(0, 3);
  for (size_t i = 0; i < size; ++i) {
    jacobianV.col(i) = 
      JOINT_STATE(chainStart+i, type)->zInBase.cross(
        eePos - JOINT_STATE(chainStart+i, type)->posInBase
      );
    jacobianW.col(i) = JOINT_STATE(chainStart+i, type)->zInBase;
  }
  Matrix<Scalar, 6, Dynamic> jacobian;
  jacobian.resize(6, size);
  jacobian << jacobianV, jacobianW;
  unsigned baseFrame = CHAIN_L_LEG;
  Matrix<Scalar, 4, 4> supportFe = getForwardEffector(baseFrame, FEET_BASE, type);
  //! Get body center frame rotation in support foot frame
  Matrix<Scalar, 3, 3> rBase = supportFe.block(0, 0, 3, 3).transpose();
  Matrix<Scalar, 6, 6> Xo;
  Xo << rBase, Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Zero(), rBase;
  jacobian = Xo * jacobian;
  return jacobian;
}

template <typename Scalar>
Matrix<Scalar, 3, Dynamic>
KinematicsModule<Scalar>::computeComJacobian(
  const unsigned& baseFrame,
  const JointStateType& type)
{
  Matrix<Scalar, 3, Dynamic> jacobian, jv, jw;
  jacobian.resize(3, NUM_JOINTS);
  jv.resize(3, NUM_JOINTS);
  jw.resize(3, NUM_JOINTS);
  jv.setZero();
  jw.setZero();
  //! Get the center of mass jacobian in body center frame
  for (size_t i = 0; i < CHAINS_SIZE; ++i) {
    jacobian.block(0, linkChains[i]->start, 3, linkChains[i]->size) = 
      computeLimbComJ(i, type);
  }
  //! Convert the center of mass jacobian in support foot frame
  //! Get support foot forward kinematics
  Matrix<Scalar, 4, 4> supportFe = getForwardEffector(baseFrame, FEET_BASE, type);
  //! Get body center frame rotation in support foot frame
  Matrix<Scalar, 3, 3> rBase = supportFe.block(0, 0, 3, 3).transpose();
  Matrix<Scalar, 6, 6> Xo;
  Xo << rBase, Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Zero(), rBase;
  Matrix<Scalar, 6, Dynamic> supportLimbJ = Xo * computeLimbJ(baseFrame, FEET_BASE, type);
  Matrix<Scalar, 3, 1> com = computeComWrtBase(baseFrame, FEET_BASE, type);
  jv.block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size) =
    supportLimbJ.block(0, 0, 3, NUM_JOINTS);
  jw.block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size) =
    supportLimbJ.block(3, 0, 3, NUM_JOINTS);
  //cout << "limbComJs:\n" << (rBase * jacobian).block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size) << endl;
  //cout << "comSkewMat:\n" << MathsUtils::makeSkewMat(com) << endl;
  //cout << "-jv:\n" << -jv.block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size) << endl;
  //cout << "jw:\n" << jw.block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size)  << endl;
  //cout << "baseComJ:\n" << (rBase * jacobian - jv + MathsUtils::makeSkewMat(com) * jw).block(0, linkChains[baseFrame]->start, 3, linkChains[baseFrame]->size) << endl;
  return rBase * jacobian - jv + MathsUtils::makeSkewMat(com) * jw;
}

template <typename Scalar>
Matrix<Scalar, 3, Dynamic>
KinematicsModule<Scalar>::computeLimbComJ(
  const unsigned& chainIndex, 
  const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  unsigned chainStart = linkChains[chainIndex]->start;
  Matrix<Scalar, 3, Dynamic> jacobian(3, size);
  jacobian.setZero();
  /*for (int j = 0; j < size; ++j) {
    Matrix<Scalar, 3, 1> z = JOINT_STATE(chainStart + j, type)->zInBase;
    Matrix<Scalar, 3, 1> pos = JOINT_STATE(chainStart + j, type)->posInBase;
    for (int m = j; m < size; ++m) { // Update the jth row of the jacobian
      jacobian.col(j) +=
        z.cross(JOINT_STATE(chainStart + m, type)->comInBase - pos) * 
        joints[chainStart + m]->link->mass;/// joints[chainStart + m]->link->chain->mass;
    }
  }*/
  Matrix<Scalar, 3, Dynamic> jLink;
  jLink.resize(3, size);
  jLink.setZero();
  Scalar chainMass = joints[chainStart]->link->chain->mass;
  for (int j = 0; j < size; ++j) {
    Matrix<Scalar, 3, 1> comT = JOINT_STATE(chainStart + j, type)->comInBase;
    for (int m = 0; m <= j; ++m) { // Update the jth row of the jacobian
      Matrix<Scalar, 3, 1> z = JOINT_STATE(chainStart + m, type)->zInBase;
      Matrix<Scalar, 3, 1> pos = JOINT_STATE(chainStart + m, type)->posInBase;
      jLink.col(m) += z.cross(comT - pos);
    }
    jacobian += jLink * joints[chainStart + j]->link->mass;
  }
  jacobian /= totalMassH25;
  return jacobian;
}

template <typename Scalar>
void
KinematicsModule<Scalar>::computeLinkComJ(
  const unsigned& index, 
  Matrix<Scalar, 3, Dynamic>& jacobianV,
  Matrix<Scalar, 3, Dynamic>& jacobianW,
  const JointStateType& type)
{
  unsigned chainStart = joints[index]->link->chain->start;
  unsigned size = joints[index]->link->chain->size;
  jacobianV.resize(3, size);
  jacobianW.resize(3, size);
  jacobianV.setZero();
  jacobianW.setZero();
  Matrix<Scalar, 3, 1> comT = JOINT_STATE(index, type)->comInBase;
  for (int m = 0; m <= index - chainStart; ++m) { // Update the jth row of the jacobian
    Matrix<Scalar, 3, 1> z = JOINT_STATE(m, type)->zInBase;
    Matrix<Scalar, 3, 1> pos = JOINT_STATE(m, type)->posInBase;
    jacobianV.col(m) = z.cross(comT - pos);
    jacobianW.col(m) = z;
  }
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveComIkTwoVar(
  const Matrix<Scalar, 3, 1>& desCom,
  const unsigned& baseLimb, 
  const unsigned& eeIndex, 
  const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  computeComWrtBase(baseLimb, eeIndex, com, type);
  Matrix<Scalar, 3, 1> diff = desCom - com;
  Scalar aY = atan2(diff[1], com[2]);
  Matrix<Scalar, Dynamic, 1> joints(NUM_JOINTS);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joints[i] = NAN;
  }
  joints[L_ANKLE_ROLL] = aY;
  joints[R_ANKLE_ROLL] = aY;
  return joints;
  //Scalar aX = atan2(diff[0], diff[2]);
  //Matrix<Scalar, 4, 4> baseTokickFoot = 
  //  MathsUtils::getTInverse(getForwardEffector(baseLimb, eeIndex, type)) * getForwardEffector(baseLimb, eeIndex, type);
  //
  //Scalar lDiff = 
}

/**
 * @Usage:
 * Matrix<Scalar, 3, 1> desComVel = Matrix<Scalar, 3, 1>::Zero();
 * Matrix<Scalar, 3, 1> desTorsoAngularVel = Matrix<Scalar, 3, 1>::Zero();
 * vector<unsigned> limbMotionSpace(CHAINS_SIZE);
 * limbMotionSpace[CHAIN_HEAD] = 0; // Joint space
 * limbMotionSpace[CHAIN_L_ARM] = 0; // Joint space
 * limbMotionSpace[CHAIN_R_ARM] = 0; // Joint space
 * limbMotionSpace[CHAIN_L_LEG] = 1; // Cartesian space
 * limbMotionSpace[CHAIN_R_LEG] = 1; // Cartesian space
 * // Define desired velocities for each joint of limbs other than support limb
 * vector<Matrix<Scalar, Dynamic, 1>> limbVelocitiesD(CHAINS_SIZE); 
 * // Zero velocity in joint space
 * limbVelocitiesD[CHAIN_HEAD] = Matrix<Scalar, 2, 1>::Zero();
 * // Zero velocity in joint space
 * limbVelocitiesD[CHAIN_L_ARM] = Matrix<Scalar, 5, 1>::Zero();
 * // Zero velocity in joint space
 * limbVelocitiesD[CHAIN_R_ARM] = Matrix<Scalar, 5, 1>::Zero();
 * // Zero velocity in cartesian space (meaning for end-effector pose)
 * limbVelocitiesD[CHAIN_L_LEG] = Matrix<Scalar, 6, 1>::Zero();
 * limbVelocitiesD[CHAIN_R_LEG] = Matrix<Scalar, 6, 1>::Zero();
 * vector<int> eeIndices(CHAINS_SIZE);
 * eeIndices[CHAIN_HEAD] = 0; // default
 * eeIndices[CHAIN_L_ARM] = 0; // default
 * eeIndices[CHAIN_R_ARM] = 0; // default
 * eeIndices[CHAIN_L_LEG] = FEET_BASE;
 * eeIndices[CHAIN_R_LEG] = FEET_BASE;
 * Matrix<Scalar, Dynamic, 1> jointsD = 
 *   solveComIK(
 *     CHAIN_L_LEG,
 *     comVelocityD,
 *     limbMotionSpace,
 *     limbVelocitiesD,
 *     eeIndices,
 *     JointStateType::ACTUAL
 *   );
 */
template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveComIK(const unsigned& baseLimb,
  const Matrix<Scalar, 6, 1>& comVelocityD,
  const vector<unsigned>& limbMotionSpace,
  const vector<Matrix<Scalar, Dynamic, 1> >& limbVelocitiesD, const vector<int>& eeIndices, const JointStateType& type)
{
  ASSERT(limbMotionSpace.size() == CHAINS_SIZE);
  ASSERT(eeIndices.size() == CHAINS_SIZE);
  ASSERT(baseLimb == CHAIN_L_LEG || baseLimb == CHAIN_R_LEG);
  vector<Matrix<Scalar, 4, 4> > limbTs(CHAINS_SIZE);
  Matrix<Scalar, 3, 3> rBase;
  Matrix<Scalar, 6, 6> XBase;
  Matrix<Scalar, 6, 6> Xo;
  vector<Matrix<Scalar, 3, Dynamic> > limbComJs(CHAINS_SIZE);
  vector<Matrix<Scalar, Dynamic, Dynamic> > limbJs(CHAINS_SIZE);
  vector<Matrix<Scalar, 6, 6> > XiWithBaseJacobian(CHAINS_SIZE);
  vector<Matrix<Scalar, Dynamic, Dynamic> > limbJsInv(CHAINS_SIZE);
  Matrix<Scalar, 3, 1> comWrtBaseLimb;
  computeComWrtBase(baseLimb, eeIndices[baseLimb], comWrtBaseLimb, type);
  cout << "comwrtBase: " << comWrtBaseLimb << endl;
  limbTs[baseLimb] = getForwardEffector(baseLimb, eeIndices[baseLimb], type);
  rBase = torsoState->rot.block(0, 0, 3, 3);//MathsUtils::getTInverse(temp).block(0, 0, 3, 3);
  Matrix<Scalar, 3, 1> eeBase = limbTs[baseLimb].block(0, 3, 3, 1);
  XBase << Matrix<Scalar, 3, 3>::Identity(), MathsUtils::makeSkewMat(rBase * eeBase),
           Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Identity();
  Xo << rBase, Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Zero(), rBase;
  Matrix<Scalar, Dynamic, Dynamic> baseComJ;
  baseComJ.resize(6, linkChains[baseLimb]->size);
  baseComJ.setZero();
  Matrix<Scalar, 6, 1> baseComVelocityD = comVelocityD;
  limbJs[baseLimb] = Xo * computeLimbJ(baseLimb, eeIndices[baseLimb], type);
  Matrix<Scalar, Dynamic, Dynamic> baseJT = XBase * limbJs[baseLimb];
  Matrix<Scalar, 3, 1> comLimbsDiff = Matrix<Scalar, 3, 1>::Zero();
  for (size_t i = 0; i < CHAINS_SIZE; ++i) {
    //cout << "limbMotionSpace[i];" << limbMotionSpace[i] << endl;
    limbComJs[i] = rBase * computeLimbComJ(i, type);
    if (limbMotionSpace[i]) {
      if (i == baseLimb) {
        baseComJ.block(3, 0, 3, linkChains[i]->size) =
          -limbJs[i].block(3, 0, 3, linkChains[i]->size);
        baseComJ.block(0, 0, 3, linkChains[i]->size) =
          baseComJ.block(0, 0, 3, linkChains[i]->size) +
          -limbJs[i].block(0, 0, 3,linkChains[i]->size) +
          MathsUtils::makeSkewMat(comWrtBaseLimb) *
          limbJs[i].block(3, 0, 3,linkChains[i]->size) +
          limbComJs[i];
        cout << "limbComJs:\n" << limbComJs[i] << endl;
        cout << "comSkewMat:\n" << MathsUtils::makeSkewMat(comWrtBaseLimb) << endl;
        cout << "-jv:\n" << -limbJs[i].block(0, 0, 3,linkChains[i]->size) << endl;
        cout << "jw:\n" << limbJs[i].block(3, 0, 3,linkChains[i]->size)  << endl;
        cout << "baseComJ:\n" << baseComJ.block(0, 0, 3, linkChains[i]->size) << endl;
      } else {
        limbTs[i] = getForwardEffector(i, eeIndices[i], type);
        limbJs[i] = Xo * computeLimbJ(i, eeIndices[i], type);
        Matrix<Scalar, 3, 1> ee = limbTs[i].block(0, 3, 3, 1);
        Matrix<Scalar, 6, 6> Xi;
        Xi << Matrix<Scalar, 3, 3>::Identity(), MathsUtils::makeSkewMat(rBase * ee), Matrix<Scalar, 3, 3>::Zero(), Matrix<Scalar, 3, 3>::Identity();
        XiWithBaseJacobian[i] = Xi.inverse() * baseJT;
        limbJsInv[i] = MathsUtils::pseudoInverse(limbJs[i]);
        baseComJ.block(0, 0, 3, linkChains[baseLimb]->size) =
          baseComJ.block(0, 0, 3, linkChains[baseLimb]->size) +
          limbComJs[i] * limbJsInv[i] * XiWithBaseJacobian[i];
        comLimbsDiff =
          comLimbsDiff + limbComJs[i] * limbJsInv[i] * limbVelocitiesD[i];
      }
    } else {
      comLimbsDiff =
        comLimbsDiff + limbComJs[i] * limbVelocitiesD[i];
    }
  }
  baseComVelocityD.segment(0, 3) =
    baseComVelocityD.segment(0, 3) - comLimbsDiff;
  Matrix<Scalar, Dynamic, 1> jointD(NUM_JOINTS);
  vector<Matrix<Scalar, Dynamic, 1> > jointVD(CHAINS_SIZE);
  cout << "baseComJ: " << baseComJ << endl;
  cout << "baseComVelocityD: " << baseComVelocityD << endl;
  jointVD[baseLimb] = MathsUtils::pseudoInverse(baseComJ, (Scalar)1e-2) * baseComVelocityD;
  for (size_t i = 0; i < CHAINS_SIZE; ++i) {
    if (i != baseLimb) {
      jointVD[i].resize(linkChains[i]->size);
      jointVD[i].setZero(); 
      if (limbMotionSpace[i]) {
        Matrix<Scalar, Dynamic, Dynamic> rhs = limbVelocitiesD[i] + XiWithBaseJacobian[i] * jointVD[baseLimb];
        jointVD[i] = limbJsInv[i] * rhs;
      }
    }
    for (size_t j = linkChains[i]->start; j < linkChains[i]->start + linkChains[i]->size; ++j) {
      jointD[j] = JOINT_STATE(j, type)->position + jointVD[i][j-linkChains[i]->start] * cycleTime;
      //if (i == baseLimb) {
      //  cout << "j: " << j << endl;
      //  cout << "JOINT_STATE(j, type)->position: " << JOINT_STATE(j, type)->position << endl;
      //  cout << "jointD[j]: " << jointD[j] << endl;
      //}
    }
  }
  cout << "jointsVD: " << jointVD[baseLimb] << endl;
  return jointD;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
KinematicsModule<Scalar>::computeMassMatrix(
  const unsigned& chainIndex, const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  unsigned chainStart = linkChains[chainIndex]->start;
  Matrix<Scalar, 3, Dynamic> jacobianCV;
  Matrix<Scalar, 3, Dynamic> jacobianCW;
  Matrix<Scalar, Dynamic, Dynamic> massMatrix;
  massMatrix.resize(size, size);
  massMatrix.setZero();
  for (int j = 0; j < size; ++j) {
    computeLinkComJ(chainStart + j, jacobianCV, jacobianCW, type);
    massMatrix +=
      links[chainStart + j]->mass * jacobianCV.transpose() *jacobianCV + 
      jacobianCW.transpose() * links[chainStart + j]->inertia * jacobianCW;
  }
  return massMatrix;
}

template <typename Scalar>
bool
KinematicsModule<Scalar>::computeVirtualMass(
  const unsigned& chainIndex, const Matrix<Scalar, 3, 1>& direction,
  const Matrix<Scalar, 4, 4>& endEffector, Scalar& virtualMass, const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  unsigned chainStart = linkChains[chainIndex]->start;
  //Matrix<Scalar, Dynamic, 1> joints = getJointPositions(chainStart, size, type);
  ////! center of mass of final link
  //Matrix<Scalar, Dynamic, Dynamic> T = MathsUtils::getTInverse(linkChains[chainIndex]->endT);
  //Matrix<Scalar, 4, 1> lastCom = T * linkComs[chainStart + size - 1];
  //cout << "lastCom: " << lastCom << endl;

  //! Inertia matrix (size x size) at given joint configuration
  Matrix<Scalar, Dynamic, Dynamic> massMatrix = computeMassMatrix(chainIndex, type);
  //cout << "Mass matrix: " << endl << massMatrix << endl;

  //Matrix<Scalar, Dynamic, Dynamic> jacobian = computeLimbJ(chainIndex, lastCom, type);
  //cout << "Jacobian: " << endl << jacobian << endl;
  //! Jacobian matrix (6 x size) at given joint configuration
  Matrix<Scalar, Dynamic, Dynamic> jacobianEE = computeLimbJ(chainIndex, endEffector, type);
  ///cout << "JacobianEE: " << endl << jacobianEE << endl;

  //! Inertia matrix inverse (size x size) at given joint configuration
  Matrix<Scalar, Dynamic, Dynamic> mmInv = massMatrix.inverse();
  //cout << "Mass matrix Inv: " << endl << mmInv << endl;

  //! Inertial projection in cartesian space (6x6)
  //! (G = 6x6 Symmetric) [G11, G12;G21, G22]
  //Matrix<Scalar, Dynamic, Dynamic> gMatrix = jacobian * mmInv * jacobian.transpose();
  //cout << " gMatrix: " << endl <<  gMatrix << endl;
  Matrix<Scalar, Dynamic, Dynamic> gMatrixEE = jacobianEE * mmInv * jacobianEE.transpose();
  //cout << "G-matrixEE: " << endl << gMatrixEE << endl;

  //! Position vector from center of mass to end effector
  //Matrix<Scalar, 3, 1> pos =
  //  endEffector.block(0, 3, 3, 1) - lastCom.block(0, 0, 3, 1);
  //Matrix<Scalar, 3, 3> skewPosT = MathsUtils::makeSkewMat(pos);
  //Matrix<Scalar, 3, 3> skewPos = skewPosT.transpose();
  //! Conversion of cartesian space inertia matrix from center of mass
  //! to the contact point using skewPos
  //Matrix<Scalar, 3, 3> g11 = gMatrix.block(0, 0, 3, 3);
  //Matrix<Scalar, 3, 3> g12 = gMatrix.block(0, 3, 3, 3);
  //Matrix<Scalar, 3, 3> g22 = gMatrix.block(3, 3, 3, 3);
  //Matrix<Scalar, Dynamic, Dynamic> transfMassMatrix =
  // g11 + skewPos * g12.transpose() + g12* skewPosT + skewPos * g22 * skewPosT;
  //cout << "G-matrix: " << endl << transfMassMatrix << endl;
  //cout << "G12: " << endl << g12 + skewPos *g22 << endl;
  //cout << "G22: " << endl << g22 << endl;
  Matrix<Scalar, Dynamic, Dynamic> g11 = gMatrixEE.block(0, 0, 3, 3);
  //! Virtual Mass in the target direction
  //virtualMass = direction.transpose() * transfMassMatrix * direction;
  virtualMass = direction.transpose() * g11 * direction;
  if (virtualMass != 0) {
    virtualMass = 1.f / virtualMass;
    //LOG_INFO("Calculated virtual mass =" + DataUtils::varToString(virtualMass));
    return true;
  } else return false;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::newtonEulerForces(
  const unsigned& chainIndex, const Matrix<Scalar, 3, 1>& extForces,
  const Matrix<Scalar, 3, 1>& extMoments, Matrix<Scalar, 3, 1>& totalForces, Matrix<Scalar, 3, 1>& totalMoments, 
  const unsigned& supportLeg, const JointStateType& type)
{
  unsigned chainSize = linkChains[chainIndex]->size;
  unsigned chainStart = linkChains[chainIndex]->start;
  //!Forward Recursion
  Matrix<Scalar, 3, 1> zAxis(0, 0, 1);
  Matrix<Scalar, 3, 1> linAcc(0, 0, -gConst);
  Matrix<Scalar, 3, 1> angVel(0, 0, 0);
  Matrix<Scalar, 3, 1> angAcc(0, 0, 0);
  Matrix<Scalar, 3, 1> linAccCom(0, 0, 0);
  vector<Matrix<Scalar, 3, 1> > comForces(chainSize);
  vector<Matrix<Scalar, 3, 1> > comMoments(chainSize);

  //! Rotating torso forces and moments to inertial frame situated at 
  //! the base support leg
  Matrix<Scalar, 4, 4> supportT = getForwardEffector(supportLeg, FEET_BASE, type);
  linAcc = MathsUtils::transformVector(supportT, linAcc);

  //cout << "chainSize" << chainSize << endl;
  //cout << "chainStart" << chainStart << endl;
  for (size_t i = 0; i < chainSize; ++i) {
    Matrix<Scalar, 4, 4> tMat =  JOINT_T(chainStart + i, type);
    Matrix<Scalar, 3, 3> rotMat = tMat.block(0, 0, 3, 3).transpose(); // transposed
    Matrix<Scalar, 3, 1> transMat = tMat.block(0, 3, 3, 1);
    angVel = rotMat * angVel + zAxis * JOINT_STATE(chainStart + i, type)->velocity; // good
    angAcc = rotMat * angAcc + (rotMat * angVel).cross(
      zAxis *  JOINT_STATE(chainStart + i, type)->velocity) + 
      zAxis * JOINT_STATE(chainStart + i, type)->accel;  // good
    linAcc = rotMat * (angAcc.cross(transMat) + angVel.cross(
      angVel.cross(transMat)) + linAcc);  // good
    Matrix<Scalar, 3, 1> comP = links[chainStart + i]->com.segment(0, 3);
    linAccCom =
      angAcc.cross(comP) + angVel.cross(angVel.cross(comP)) + linAcc;
    comForces[i] = links[chainStart + i]->mass * linAccCom;
    comMoments[i] = links[chainStart + i]->inertia * angVel + angVel.cross(
      links[chainStart + i]->inertia * angVel);
    
    /*cout << "comForces[" << i << "]" << "      "<< comForces[i](0,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](1,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](2,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](0,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](1,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](2,0) << endl;
    cout<< endl;*/
  }

  //!Backward Recursion
  Matrix<Scalar, Dynamic, 1> jointTorques;
  jointTorques.resize(chainSize);
  Matrix<Scalar, 3, 1> f(0, 0, 0);
  Matrix<Scalar, 3, 1> n(0, 0, 0);
  for (size_t i = chainSize; i > 0; --i) {
    Matrix<Scalar, 4, 4> tMat;
    if (i == chainSize) {
      tMat.setIdentity();
      f = extForces;
      n = extMoments;
    } else {
      tMat =  JOINT_T(chainStart + i, type);
    }
    Matrix<Scalar, 3, 3> rotMat = tMat.block(0, 0, 3, 3);
    Matrix<Scalar, 3, 1> transMat = tMat.block(0, 3, 3, 1);
    Matrix<Scalar, 3, 1> comP = links[chainStart + i - 1]->com.segment(0, 3);
    n = comMoments[i-1] + rotMat * n + comP.cross(comForces[i-1]) + transMat.cross(rotMat * f);
    f = comForces[i-1] + rotMat * f;
    jointTorques[i-1] = n.transpose() * zAxis;
  }
  Matrix<Scalar, 3, 3> initRot = JOINT_T(chainStart, type).block(0, 0, 3, 3);
  f = initRot * f;
  n = initRot * n;
  //jointTorques[0] = n.transpose() * zAxis;
  /*cout << "forces" << f(0,0) << endl;
  cout << "forces" << f(1,0) << endl;
  cout << "forces" << f(2,0) << endl;
  cout << "moments" << n(0,0) << endl;
  cout << "moments" << n(1,0) << endl;
  cout << "moments" << n(2,0) << endl;*/
  
  //! Relocating moments to torso origin frame
  n = n + Matrix<Scalar, 3, 1>(linkChains[chainIndex]->startT.block(0, 3, 3, 1)).cross(f);
  // f remains the same
      
  totalMoments = n;
  totalForces = f;
  return jointTorques;
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeZmp(
  const unsigned& supportLeg, const JointStateType& type)
{
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> torsoForces;
  Matrix<Scalar, 3, 1> torsoMoments;
  extForces.setZero();
  extMoments.setZero();
  torsoForces.setZero();
  torsoMoments.setZero();
  for (int i = 0; i < CHAINS_SIZE; ++i) {
    Matrix<Scalar, 3, 1> chainForces;
    Matrix<Scalar, 3, 1> chainMoments;
    Matrix<Scalar, Dynamic, 1> torque = 
      newtonEulerForces(
        i, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
    torsoForces += chainForces;
    torsoMoments += chainMoments;
  }
  
  //! Rotating torso forces and moments to inertial frame situated at 
  //! the base support leg
  Matrix<Scalar, 4, 4> supportT;
  supportT =
    MathsUtils::getTInverse(
      getForwardEffector(supportLeg, FEET_BASE, type)
    );
    
  Matrix<Scalar, 3, 3> supportR = supportT.block(0, 0, 3, 3);
  torsoForces = supportR * torsoForces;
  torsoMoments = supportR * torsoMoments;  
  
  //! Torso weight and vector
  Matrix<Scalar, 3, 1> torsoCog, batteryCog;
  torsoCog = (supportT * links[TORSO]->com).block(0, 0, 3, 1); // Torso center of mass
  //batteryCog = (supportT * Matrix<Scalar, 4, 1>(batteryX, batteryY, batteryZ, 1.f)).block(0, 0, 3, 1); // Battery center of mass
  Matrix<Scalar, 3, 1> torsoWeight(0.f, 0.f, links[TORSO]->mass * -gConst);
  //auto batteryWeight = Matrix<Scalar, 3, 1>(0.f, 0.f, batteryMass * -gConst);
  
  //! Resulatant moments and forces at the base frame
  torsoMoments = 
    torsoMoments + 
    Matrix<Scalar, 3, 1>(supportT.block(0, 3, 3, 1)).cross(torsoForces) +
    torsoCog.cross(torsoWeight);//
    //batteryCog.cross(batteryWeight);
  
  Matrix<Scalar, 3, 1> rForce = -torsoForces - torsoWeight;// - batteryWeight;
  
  // zmp_y * R_z - zmp_z * R_y + M_x = 0
  // zmp_z * R_x - zmp_x * R_z + M_y = 0
  // zmp_x * R_y - zmp_y * R_x + M_z = 0
  
  Matrix<Scalar, 2, 1> zmp;
  zmp[0] = torsoMoments[1] / rForce[2]; // M_y / R_z
  zmp[1] = - torsoMoments[0] / rForce[2]; // - M_x / R_z  
  
  Matrix<Scalar, 2, 1> com;
  computeComWrtBase(supportLeg, FEET_BASE, com, type);
  //cout << "Center of mass: " << endl;
  //cout << com << endl;
  
  //cout << "Zmp: " << endl;
  //cout << zmp << endl;
  return zmp;
}

template <typename Scalar>
Matrix<Scalar, 3, 1> KinematicsModule<Scalar>::calculateCenterOfMass(
  const JointStateType& type)
{
  Matrix<Scalar, 3, 1> com;
  com.setZero();
//  cout << "Calculating center of mass ..." << endl;
  for (size_t i = 0; i < NUM_JOINTS; ++i) 
  {
    //! Get center of mass position in base frame
    com += joints[i]->states[(unsigned)type]->comInBase * joints[i]->link->mass;
    //cout << "JointCominBase: " << joints[i]->states[(unsigned)type]->comInBase << endl;
    //cout << "JointMass: << joints[i]->link->mass: " << endl;
  }
  com += links[TORSO]->com.block(0, 0, 3, 1) * links[TORSO]->mass;
  //cout << "torso mass: " << links[TORSO]->mass << endl;
  //cout << "torso com: " << links[TORSO]->com.block(0, 0, 3, 1) << endl;
  //com = com + Matrix<Scalar, 3, 1>(batteryX, batteryY, batteryZ) * batteryMass;
  com = com / totalMassH25;
  //cout << "com: " << com << endl;
  return com;
}

template <typename Scalar>
void
KinematicsModule<Scalar>::updateFootOnGround()
{
  auto& fsrSensors = OVAR(vector<float>, MotionModule::fsrSensors);
  feetForcesBuffer.push_back(
    Matrix<Scalar, 2, 1>(fsrSensors[L_FOOT_TOTAL_WEIGHT], fsrSensors[R_FOOT_TOTAL_WEIGHT]));
  if (feetForcesBuffer.size() >= ffBufferSize) {
    Matrix<Scalar, 2, 1> bufferAvg = Matrix<Scalar, 2, 1>::Zero();
    for (int i = 0; i < feetForcesBuffer.size(); ++i)
      bufferAvg = bufferAvg + feetForcesBuffer[i];
    bufferAvg = bufferAvg / ffBufferSize;

    if (bufferAvg[L_FOOT] < 0.1 && bufferAvg[R_FOOT] < 0.1) {
      footOnGround = -1;
    } else {
      if (bufferAvg[L_FOOT] > 1.3) footOnGround = L_FOOT;
      else if (bufferAvg[R_FOOT] > 1.3) footOnGround = R_FOOT;
      else footOnGround = L_FOOT;
    }
  }
}

template <typename Scalar>
Matrix<Scalar, 2, 1> KinematicsModule<Scalar>::computeFsrZmp(const unsigned& refFrame)
{
  auto& fsrSensors = OVAR(vector<float>, MotionModule::fsrSensors);
  fsrSensors[L_FOOT_TOTAL_WEIGHT] =
      fsrSensors[L_FOOT_FSR_FL] +
      fsrSensors[L_FOOT_FSR_FR] +
      fsrSensors[L_FOOT_FSR_RL] +
      fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[R_FOOT_TOTAL_WEIGHT] =
      fsrSensors[R_FOOT_FSR_FL] +
      fsrSensors[R_FOOT_FSR_FR] +
      fsrSensors[R_FOOT_FSR_RL] +
      fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_X] =
    LFSRFL_X * fsrSensors[L_FOOT_FSR_FL] +
    LFSRFR_X * fsrSensors[L_FOOT_FSR_FR] +
    LFSRRL_X * fsrSensors[L_FOOT_FSR_RL] +
    LFSRRR_X * fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_X] /= fsrSensors[L_FOOT_TOTAL_WEIGHT];
  fsrSensors[L_FOOT_COP_Y] =
    LFSRFL_Y * fsrSensors[L_FOOT_FSR_FL] +
    LFSRFR_Y * fsrSensors[L_FOOT_FSR_FR] +
    LFSRRL_Y * fsrSensors[L_FOOT_FSR_RL] +
    LFSRRR_Y * fsrSensors[L_FOOT_FSR_RR];
  fsrSensors[L_FOOT_COP_Y] /= fsrSensors[L_FOOT_TOTAL_WEIGHT];
  fsrSensors[R_FOOT_COP_X] =
    RFSRFL_X * fsrSensors[R_FOOT_FSR_FL] +
    RFSRFR_X * fsrSensors[R_FOOT_FSR_FR] +
    RFSRRL_X * fsrSensors[R_FOOT_FSR_RL] +
    RFSRRR_X * fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[R_FOOT_COP_X] /= fsrSensors[R_FOOT_TOTAL_WEIGHT];
  fsrSensors[R_FOOT_COP_Y] =
    RFSRFL_Y * fsrSensors[R_FOOT_FSR_FL] +
    RFSRFR_Y * fsrSensors[R_FOOT_FSR_FR] +
    RFSRRL_Y * fsrSensors[R_FOOT_FSR_RL] +
    RFSRRR_Y * fsrSensors[R_FOOT_FSR_RR];
  fsrSensors[R_FOOT_COP_Y] /= fsrSensors[R_FOOT_TOTAL_WEIGHT];

  Matrix<Scalar, 4, 4> tl = getForwardEffector(CHAIN_L_LEG, ANKLE);
  Matrix<Scalar, 4, 4> tr = getForwardEffector(CHAIN_R_LEG, ANKLE);
  Matrix<Scalar, 2, 1> cop;
  cop[0] = 
    ((tl(0, 3) + fsrSensors[L_FOOT_COP_X]) * fsrSensors[L_FOOT_TOTAL_WEIGHT] +
    (tr(0, 3) + fsrSensors[R_FOOT_COP_X]) * fsrSensors[R_FOOT_TOTAL_WEIGHT]) /
    (fsrSensors[L_FOOT_TOTAL_WEIGHT] + fsrSensors[R_FOOT_TOTAL_WEIGHT]);
  cop[1] = 
    ((tl(1, 3) + fsrSensors[L_FOOT_COP_Y]) * fsrSensors[L_FOOT_TOTAL_WEIGHT] +
    (tr(1, 3) + fsrSensors[R_FOOT_COP_Y]) * fsrSensors[R_FOOT_TOTAL_WEIGHT]) /
    (fsrSensors[L_FOOT_TOTAL_WEIGHT] + fsrSensors[R_FOOT_TOTAL_WEIGHT]);
  if (refFrame == CHAIN_L_LEG) {
    cop[0] -= tl(0, 3);
    cop[1] -= tl(1, 3);
  } else if (refFrame == CHAIN_R_LEG) {
    cop[0] -= tr(0, 3);
    cop[1] -= tr(1, 3);
  }
  return cop;
}


template <typename Scalar> Matrix<Scalar, 4, 4>
KinematicsModule<Scalar>::getFeetCenterT()
{
  Matrix<Scalar, 4, 4> T;
  Matrix<Scalar, 4, 4> ee;
  if (footOnGround == L_FOOT) {
    ee = linkChains[CHAIN_L_LEG]->endEffectors[FEET_BASE];
    MathsUtils::makeTranslation(T, (Scalar) 0.0, (Scalar) -0.05, (Scalar) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_L_LEG, ee);
  } else if (footOnGround == R_FOOT) {
    ee = linkChains[CHAIN_R_LEG]->endEffectors[FEET_BASE];
    MathsUtils::makeTranslation(T, (Scalar) 0.0, (Scalar) 0.05, (Scalar) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_R_LEG, ee);
  } else {
    ee = linkChains[CHAIN_L_LEG]->endEffectors[FEET_BASE];
    MathsUtils::makeTranslation(T, (Scalar) 0.0, (Scalar) -0.05, (Scalar) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_L_LEG, ee);
  }
  return T;
}

template <typename Scalar> void
KinematicsModule<Scalar>::updateTorsoToFeet()
{
  OVAR(Matrix4f, MotionModule::lFootOnGround) = 
    getForwardEffector(CHAIN_L_LEG, FEET_BASE).template cast <float> ();
  OVAR(Matrix4f, MotionModule::rFootOnGround) = 
    getForwardEffector(CHAIN_R_LEG, FEET_BASE).template cast <float> ();
}

template <typename Scalar> Matrix<Scalar, 4, 1>
KinematicsModule<Scalar>::getWorldToCam(
  const unsigned& camIndex, const Matrix<Scalar, 4, 1>& posInFoot)
{
  if (camIndex == TOP_CAM) {
    return OVAR(Matrix4f, MotionModule::upperCamInFeet).cast <Scalar> () * posInFoot;
  } else {
    return OVAR(Matrix4f, MotionModule::lowerCamInFeet).cast <Scalar> () * posInFoot;
  }
}

template <typename Scalar> void
KinematicsModule<Scalar>::updateFootToCamT()
{
  Matrix<Scalar, 4, 4> torsoToFeet = getFeetCenterT();
  //cout << "Feet center: \n" << torsoToFeet << endl;
  Matrix<Scalar, 4, 4> torsoPitchRot;
  MathsUtils::makeRotationXYZ(
    torsoPitchRot,
    (Scalar) 0.0,
    (Scalar) -torsoPitchOffset * M_PI / 180,
    (Scalar) 0.0);
  torsoToFeet = torsoPitchRot * torsoToFeet;
  //cout << "Feet center: \n" << torsoToFeet << endl;
  OVAR(Matrix4f, MotionModule::upperCamInFeet) =
    (MathsUtils::getTInverse(
      getForwardEffector(
        CHAIN_HEAD, linkChains[CHAIN_HEAD]->endEffectors[TOP_CAM])
     ) * torsoToFeet
    ).template cast <float> ();
  OVAR(Matrix4f, MotionModule::lowerCamInFeet) = 
    (MathsUtils::getTInverse(
      getForwardEffector(
        CHAIN_HEAD, linkChains[CHAIN_HEAD]->endEffectors[BOTTOM_CAM])
     ) * torsoToFeet
    ).template cast <float> ();
}

template <typename Scalar> boost::shared_ptr<PostureTask<Scalar> >
KinematicsModule<Scalar>::makePostureTask(
  const Matrix<Scalar, Dynamic, 1>& targetJoints,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain)
{
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, false);
    for (size_t i = 0; i < NUM_JOINTS; ++i)
      activeJoints[i] = true;
  }
  return boost::make_shared<PostureTask<Scalar> >(
    targetJoints,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule()
  );
}

template <typename Scalar> boost::shared_ptr<ContactTask<Scalar> >
KinematicsModule<Scalar>::makeContactTask(
  const unsigned& chainIndex,
  const unsigned& eeIndex,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain)
{
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, false);
    for (size_t i = 0; i < linkChains[chainIndex]->size; ++i)
      activeJoints[linkChains[chainIndex]->start + i] = true;
  }
  return boost::make_shared<ContactTask<Scalar> >(
    chainIndex,
    linkChains[chainIndex]->endEffectors[eeIndex],
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule()
  );
}

template <typename Scalar> boost::shared_ptr<CartesianTask<Scalar> >
KinematicsModule<Scalar>::makeCartesianTask(
  const unsigned& chainIndex,
  const unsigned& eeIndex,
  const Matrix<Scalar, 4, 4>& targetT,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain)
{
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, false);
    for (size_t i = 0; i < linkChains[chainIndex]->size; ++i)
      activeJoints[linkChains[chainIndex]->start + i] = true;
  }
  return boost::make_shared<CartesianTask<Scalar> >(
    chainIndex,
    linkChains[chainIndex]->endEffectors[eeIndex],
    targetT,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule()
  );
}

template <typename Scalar> boost::shared_ptr<ComTask<Scalar> >
KinematicsModule<Scalar>::makeComTask(
  // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
  const unsigned& refFrame,
  const Matrix<Scalar, 3, 1>& comTarget,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain)
{
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, true);
  }
  return boost::make_shared<ComTask<Scalar> >(
    refFrame,
    comTarget,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule()
  );
}

template <typename Scalar> boost::shared_ptr<TorsoTask<Scalar> >
KinematicsModule<Scalar>::makeTorsoTask(
  // com reference frame (left or right foot defined by CHAIN_L_LEG or CHAIN_R_LEG)
  const unsigned& refFrame,
  const unsigned& eeIndex,
  const Matrix<Scalar, 4, 4>& target,
  vector<bool> activeJoints,
  const Scalar& weight,
  const Scalar& gain)
{
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, true);
  }
  return boost::make_shared<TorsoTask<Scalar> >(
    refFrame,
    linkChains[refFrame]->endEffectors[eeIndex],
    target,
    weight,
    gain,
    activeJoints,
    motionModule->getKinematicsModule()
  );
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveTasksIK(
  const vector<boost::shared_ptr<MotionTask<Scalar> > >& tasks,
  const unsigned& maxIterations)
{
  vector<bool> activeJoints(NUM_JOINTS, false);
  TaskIkSolver<Scalar> tis =
    TaskIkSolver<Scalar>(
      motionModule->getKinematicsModule(), maxIterations, vector<bool>(), 1e-2);
  for (size_t i = 0; i < activeJoints.size(); ++i) {
    for (size_t j = 0; j < tasks.size(); ++j) {
      if (tasks[j]) {
        if (tasks[j]->getActiveJoints()[i])
          activeJoints[i] = true;
      }
    }
  }
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i])
      tis.addTask(tasks[i]);
  }
  tis.setActiveJoints(activeJoints);
  return tis.solve();
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveCartesianIK(
  const unsigned& chainIndex,
  const unsigned& eeIndex,
  const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations)
{
  vector<bool> activeJoints;
  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, false);
    for (size_t i = 0; i < linkChains[chainIndex]->size; ++i)
      activeJoints[linkChains[chainIndex]->start + i] = true;
  }
  TaskIkSolver<Scalar> tis =
    TaskIkSolver<Scalar>(
      motionModule->getKinematicsModule(), maxIterations, activeJoints, 1e-2);
  CartesianTaskPtr ctp =
    boost::make_shared<CartesianTask<Scalar> >(
      chainIndex,
      linkChains[chainIndex]->endEffectors[eeIndex],
      targetT,
      1,
      0.9,
      activeJoints,
      motionModule->getKinematicsModule()
    );  if (activeJoints.empty()) {
    activeJoints.resize(NUM_JOINTS, false);
    for (size_t i = 0; i < linkChains[chainIndex]->size; ++i)
      activeJoints[linkChains[chainIndex]->start + i] = true;
  }
  tis.addTask(ctp);
  return tis.solve();
}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveJacobianIK(
  const unsigned& chainIndex, const unsigned& eeIndex, const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations, const JointStateType& startType, const bool& solveForOrientation,
  const Scalar& pTol,
  const Scalar& oTol)
{
  return solveJacobianIK(
    chainIndex,
    linkChains[chainIndex]->endEffectors[eeIndex],
    targetT,
    maxIterations,
    startType,
    solveForOrientation,
    pTol,
    oTol);
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1>
KinematicsModule<Scalar>::solveJacobianIK(
  const unsigned& chainIndex, const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT,
  const unsigned& maxIterations,
  const JointStateType& startType,
  const bool& solveForOrientation,
  const Scalar& pTol,
  const Scalar& oTol)
{
  JointStateType type = JointStateType::SIM;
  unsigned chainStart = linkChains[chainIndex]->start;
  unsigned chainSize = linkChains[chainIndex]->size;
  Scalar kP = 1.0, kO = 1.0;
  setStateFromTo(startType, type);
  Matrix<Scalar, 4, 4> initT = 
    getForwardEffector(chainIndex, endEffector, type);
  Matrix<Scalar, 3, 1> initPos = initT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> targetPos = targetT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> diffPos = kP * (targetPos - initPos);
  Matrix<Scalar, 3, 1> diffOrient;
  if (solveForOrientation)
    diffOrient = kO * MathsUtils::getOrientationDiff(initT, targetT);
  bool success = false;
  if (solveForOrientation) {
    if (diffPos.norm() < pTol)
      success = true;
  } else {
    if (diffPos.norm() < pTol && diffOrient.norm() < oTol)
      success = true;
  }
  for (size_t i = 0; i < maxIterations; ++i) {
    Matrix<Scalar, Dynamic, Dynamic> J, jInv;
    Matrix<Scalar, Dynamic, 1> result;
    if (solveForOrientation) {
      J = computeLimbJ(chainIndex, endEffector, type);
      jInv = MathsUtils::pseudoInverse(J);
      Matrix<Scalar, 6, 1> diff;
      diff.block(0, 0, 3, 1) = diffPos;
      diff.block(3, 0, 3, 1) = diffOrient;
      result = jInv * diff;
    } else {
      J = computeLimbJ(chainIndex, endEffector, type).block(0, 0, 3, 6);
      jInv = MathsUtils::pseudoInverse(J);
      result = jInv * diffPos;
    }
    for (size_t i = 0; i < chainSize; ++i) {
      auto& position = JOINT_STATE(chainStart + i, type)->position;
      position += result[i];
      if (abs(position) > M_PI)
        position = atan2(sin(position), cos(position));
      if (position > joints[chainStart + i]->maxPosition) 
        position = joints[chainStart + i]->maxPosition;
      if (position < joints[chainStart + i]->minPosition) 
        position = joints[chainStart + i]->minPosition;
      JOINT_STATE(chainStart + i, type)->position = position;
    }
    prepareDHTransforms(chainIndex, type);
    initT = getForwardEffector(chainIndex, endEffector, type);
    initPos = initT.block(0, 3, 3, 1);
    if (solveForOrientation) {
      diffOrient = kO * MathsUtils::getOrientationDiff(initT, targetT);
    }
    diffPos = kP * (targetPos - initPos);
    if (solveForOrientation) {
      if (diffPos.norm() < pTol)
        success = true;
      else
        success = false;
    } else {
      if (diffPos.norm() < pTol && diffOrient.norm() < oTol)
        success = true;
      else
        success = false;
    }
  }
  cout << "Success: " << success << endl;
  cout << "diffO: " << diffOrient << endl;
  cout << "diffP: " << diffPos << endl;
  Matrix<Scalar, Dynamic, 1> angles;
  angles.resize(chainSize);
  if (!success) {
      for (size_t i = 0; i < chainSize; ++i)
        angles[i] = NAN;
  } else {
      for (size_t i = 0; i < chainSize; ++i)
        angles[i] = JOINT_STATE(chainStart + i, type)->position;
  }
  return angles;
}

template <typename Scalar>
vector<Matrix<Scalar, Dynamic, 1> >
KinematicsModule<Scalar>::inverseLeftLeg(
  const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT)
{
  auto type = JointStateType::SIM;
  vector<Matrix<Scalar, Dynamic, 1> > returnResult;
  Matrix<Scalar, 4, 4> tTempTheta5, t4I, t5I, t6I, tTemp, tTemp2;
  Matrix<Scalar, 4, 4> t = targetT;
  Matrix<Scalar, 4, 4> tInit = t;

  //!Move the start point to the hipyawpitch point
  Matrix<Scalar, 4, 4> base = tBaseLLegInv;
  base *= t;

  //!Move the end point to the anklePitch joint
  base *= tEndLLegInv;

  //!Rotate hipyawpitch joint
  Matrix<Scalar, 4, 4> rot = rotFixLLeg;
  rot *= base;

  //!Invert the table, because we need the
  //!chain from the ankle to the hip
  Matrix<Scalar, 4, 4> tStart = rot;
  rot = MathsUtils::getTInverse(rot);
  t = rot;

  //!Build the rotation table
  Scalar side1 = thighLength;
  Scalar side2 = tibiaLength;
  Scalar distanceSqrd = pow(t.block(0, 3, 3, 1).norm(), 2);

  //!Calculate Theta 4
  Scalar theta4 = M_PI - MathsUtils::safeAcos(
    (pow(side1, 2) + pow(side2, 2) - distanceSqrd) / (2 * side1 * side2));
  if (theta4 != theta4) {
    return returnResult;
  }
  Scalar theta6 = atan(t(1, 3) / t(2, 3));
  //if(theta6 < lAnkleRollLow || theta6 > lAnkleRollHigh)
  //  return returnResult;
  if (theta6 < lAnkleRollLow) theta6 = lAnkleRollLow;
  else if (theta6 > lAnkleRollHigh) theta6 = lAnkleRollHigh;

  MathsUtils::makeDHTransformation(
    t6I,
    (Scalar) 0.0,
    (Scalar) -M_PI_2,
    (Scalar) 0.0,
    theta6);
  t6I *= rotRLeg;
  //try
  //{
  t6I = MathsUtils::getTInverse(t6I);
  tStart *= t6I;
  tTempTheta5 = tStart;
  tTempTheta5 = MathsUtils::getTInverse(tTempTheta5);
  //  }
  //catch(KMath::KMat::SingularMatrixInvertionException d)
  //{
  //return returnResult;
  //}
  for (int itter = 0; itter < 2; itter++) {
    theta4 = (itter == 0) ? theta4 : -theta4;
    if (theta4 < rKneePitchLow || theta4 > rKneePitchHigh) continue;
    MathsUtils::makeDHTransformation(
      t4I,
      (Scalar) -thighLength,
      (Scalar) 0.0,
      (Scalar) 0.0,
      theta4);
    Scalar up =
      tTempTheta5(1, 3) * (tibiaLength + thighLength * cos(theta4)) + thighLength * tTempTheta5(
        0,
        3) * sin(theta4);
    Scalar down = pow(thighLength, 2) * pow(sin(theta4), 2) + pow(
      tibiaLength + thighLength * cos(theta4),
      2);
    Scalar theta5 = asin(-up / down);
    Scalar posOrNegPIt5 = (theta5 >= 0) ? M_PI : -M_PI;
    if (theta5 != theta5 && up / down < 0) theta5 = -M_PI_2;
    else if (theta5 != theta5) theta5 = M_PI_2;
    for (int i = 0; i < 2; i++) {
      if (i == 0 && (theta5 > lAnklePitchHigh || theta5 < lAnklePitchLow)) continue;
      else if (i == 1 && (posOrNegPIt5 - theta5 > lAnklePitchHigh || posOrNegPIt5 - theta5 < lAnklePitchLow)) continue;
      else if (i == 1) theta5 = posOrNegPIt5 - theta5;
      MathsUtils::makeDHTransformation(
        t5I,
        (Scalar) -tibiaLength,
        (Scalar) 0.0,
        (Scalar) 0.0,
        theta5);
      tTemp = t4I;
      tTemp *= t5I;
      //try
      //  {
      tTemp = MathsUtils::getTInverse(tTemp);
      //}
      //catch(KMath::KMat::SingularMatrixInvertionException d)
      //{
      //  continue;
      //}
      tTemp2 = tStart;
      tTemp2 *= tTemp;
      Scalar temptheta2 = MathsUtils::safeAcos(tTemp2(1, 2));
      Scalar theta2;
      for (int l = 0; l < 2; l++) {
        if (l == 0 && (temptheta2 - M_PI_4 > lHipRollHigh || temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 1 && (-temptheta2 - M_PI_4 > lHipRollHigh || -temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 0) theta2 = temptheta2 - M_PI_4;
        else if (l == 1) theta2 = -temptheta2 - M_PI_4;
        Scalar theta3 = asin(tTemp2(1, 1) / sin(theta2 + M_PI_4));
        Scalar posOrNegPIt3 = (theta3 >= 0) ? M_PI : -M_PI;
        if (theta3 != theta3 && tTemp2(1, 1) / sin(theta2 + M_PI_4) < 0) theta3 =
          -M_PI_2;
        else if (theta3 != theta3) theta3 = M_PI_2;
        for (int k = 0; k < 2; k++) {
          if (k == 0 && (theta3 > lHipPitchHigh || theta3 < lHipPitchLow)) continue;
          else if (k == 1 && (posOrNegPIt3 - theta3 > lHipPitchHigh || posOrNegPIt3 - theta3 < lHipPitchLow)) continue;
          else if (k == 1) theta3 = posOrNegPIt3 - theta3;
          Scalar temptheta1 = MathsUtils::safeAcos(
            tTemp2(0, 2) / sin(theta2 + M_PI_4));
          if (temptheta1 != temptheta1) temptheta1 = 0;
          for (int p = 0; p < 2; p++) {
            Scalar theta1;

            if (p == 0 && (temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 1 && (-temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 0) theta1 = temptheta1 + M_PI_2;
            else if (p == 1) theta1 = -temptheta1 + M_PI_2;

            //!Forward VALID step
            JOINT_STATE(L_HIP_YAW_PITCH, type)->position = theta1;
            JOINT_STATE(L_HIP_ROLL, type)->position = theta2;
            JOINT_STATE(L_HIP_PITCH, type)->position = theta3;
            JOINT_STATE(L_KNEE_PITCH, type)->position = theta4;
            JOINT_STATE(L_ANKLE_PITCH, type)->position = theta5;
            JOINT_STATE(L_ANKLE_ROLL, type)->position = theta6;
            prepareDHTransforms(CHAIN_L_LEG, type);
            Matrix<Scalar, 4, 4> test = getForwardEffector(CHAIN_L_LEG, endEffector, type);
            if (MathsUtils::almostEqual(test, tInit)) {
              Matrix<Scalar, Dynamic, 1> r(R_LEG_SIZE);
              r[0] = theta1;
              r[1] = theta2;
              r[2] = theta3;
              r[3] = theta4;
              r[4] = theta5;
              r[5] = theta6;
              returnResult.push_back(r);
            }
          }
        }
      }
    }
  }
  return returnResult;
}

template <typename Scalar>
vector<Matrix<Scalar, Dynamic, 1> >
KinematicsModule<Scalar>::inverseRightLeg(const Matrix<Scalar, 4, 4>& endEffector,
  const Matrix<Scalar, 4, 4>& targetT)
{
  Matrix<Scalar, 4, 4> mirrored = MathsUtils::mirrorTransformation(targetT);
  vector<Matrix<Scalar, Dynamic, 1> > res = inverseLeftLeg(endEffector, mirrored);
  for (size_t i = 0; i < res.size(); i++) {
    res[i][1] = -res[i][1]; //HIP_ROLL
    res[i][5] = -res[i][5]; //ANKLE_ROLL
  }
  return res;
}

template <typename Scalar>
void KinematicsModule<Scalar>::setStateFromTo(
  const JointStateType& from, 
  const JointStateType& to)
{
  for (size_t i = 0; i < joints.size(); ++i) {
    *joints[i]->states[(unsigned)to] =
      *joints[i]->states[(unsigned)from];
  }
  //prepareDHTransforms(CHAINS_SIZE, to);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointPositions(
  const unsigned& startIndex, 
  const Matrix<Scalar, Dynamic, 1>& simPosition, 
  const JointStateType& type)
{
  ASSERT(startIndex + simPosition.size() <= NUM_JOINTS);
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[startIndex + i]->states[(unsigned)type]->position = simPosition[i];
  }
  prepareDHTransforms(CHAINS_SIZE, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainPositions(
  const unsigned& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition,
  const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  ASSERT(simPosition.size() == size);
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->position = 
      simPosition[i];
  }
  prepareDHTransforms(chainIndex, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointVelocities(
  const unsigned& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simVelocities,
  const JointStateType& type)
{
  ASSERT(startIndex + simVelocities.size() <= NUM_JOINTS);
  for (size_t i = 0; i < simVelocities.size(); ++i) {
    joints[startIndex + i]->states[(unsigned)type]->velocity = simVelocities[i];
  }
  prepareDHTransforms(CHAINS_SIZE, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainVelocities(
  const unsigned& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simVelocities,
  const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  ASSERT(simVelocities.size() == size);
  for (size_t i = 0; i < simVelocities.size(); ++i) {
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->velocity = 
      simVelocities[i];
  }
  prepareDHTransforms(chainIndex, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointAccelerations(
  const unsigned& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simAccelerations,
  const JointStateType& type)
{
  ASSERT(startIndex + simAccelerations.size() <= NUM_JOINTS);
  for (size_t i = 0; i < simAccelerations.size(); ++i) {
    joints[startIndex + i]->states[(unsigned)type]->accel = simAccelerations[i];
  }
  prepareDHTransforms(CHAINS_SIZE, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainAccelerations(
  const unsigned& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simAccelerations,
  const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  ASSERT(simAccelerations.size() == size);
  for (size_t i = 0; i < simAccelerations.size(); ++i) {
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->accel = 
      simAccelerations[i];
  }
  prepareDHTransforms(chainIndex, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setJointState(
  const unsigned& startIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition, 
  const Matrix<Scalar, Dynamic, 1>& simVelocity,
  const Matrix<Scalar, Dynamic, 1>& simAcceleration,
  const JointStateType& type)
{
  ASSERT(startIndex + simPosition.size() <= NUM_JOINTS);
  ASSERT(
    simPosition.size() == simVelocity.size() && 
    simPosition.size() == simAcceleration.size());
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[startIndex + i]->states[(unsigned)type]->position = simPosition[i];
    joints[startIndex + i]->states[(unsigned)type]->velocity = simVelocity[i];
    joints[startIndex + i]->states[(unsigned)type]->accel = simAcceleration[i];
  }
  prepareDHTransforms(CHAINS_SIZE, type);
}

template <typename Scalar>
void KinematicsModule<Scalar>::setChainState(
  const unsigned& chainIndex,
  const Matrix<Scalar, Dynamic, 1>& simPosition, 
  const Matrix<Scalar, Dynamic, 1>& simVelocity,
  const Matrix<Scalar, Dynamic, 1>& simAcceleration,
  const JointStateType& type)
{
  unsigned size = linkChains[chainIndex]->size;
  ASSERT(simPosition.size() == size);
  ASSERT(
    simPosition.size() == simVelocity.size() && 
    simPosition.size() == simAcceleration.size());
  for (size_t i = 0; i < simPosition.size(); ++i) {
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->position = 
      simPosition[i];
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->velocity = 
      simVelocity[i];
    joints[linkChains[chainIndex]->start + i]->states[(unsigned)type]->accel = 
      simAcceleration[i];
  }
  prepareDHTransforms(CHAINS_SIZE, type);  
}

template <typename Scalar>
boost::shared_ptr<Joint<Scalar> > 
KinematicsModule<Scalar>::getJoint(const unsigned& index)
{
  return joints[index];
}
  
template <typename Scalar>
boost::shared_ptr<JointState<Scalar> > 
KinematicsModule<Scalar>::getJointState(
  const unsigned& index, 
  const JointStateType& type)
{
  return joints[index]->states[(unsigned)type];
}

template <typename Scalar>
vector<boost::shared_ptr<Joint<Scalar> > > 
KinematicsModule<Scalar>::getJoints(
  const unsigned& startIndex,
  const unsigned& nElements)
{
  ASSERT(startIndex + nElements <= NUM_JOINTS);
  return 
    vector<boost::shared_ptr<Joint<Scalar> > >(
      joints.begin() + startIndex, 
      joints.begin() + startIndex + nElements
    );
}

template <typename Scalar>
vector<boost::shared_ptr<JointState<Scalar> > > 
KinematicsModule<Scalar>::getJointStates(
  const unsigned& startIndex,
  const unsigned& nElements,
  const JointStateType& type)
{
  ASSERT(startIndex + nElements <= NUM_JOINTS);
  vector<boost::shared_ptr<JointState<Scalar> > > states;
  for (size_t i = startIndex; i < startIndex + nElements; ++i)
    states.push_back(joints[i]->states[(unsigned)type]);
  return states;
}

template <typename Scalar>
vector<boost::shared_ptr<JointState<Scalar> > > 
KinematicsModule<Scalar>::getChainStates(
  const unsigned& chainIndex, 
  const JointStateType& type)
{
  unsigned start = linkChains[chainIndex]->start;
  unsigned size = linkChains[chainIndex]->size;
  vector<boost::shared_ptr<JointState<Scalar> > > states;
  for (size_t i = start; i < start + size; ++i)
    states.push_back(joints[i]->states[(unsigned)type]);
  return states;
}
template <typename Scalar>
Scalar KinematicsModule<Scalar>::getJointPosition(
  const unsigned& index,
  const JointStateType& type)
{
  return joints[index]->states[(unsigned)type]->position;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> KinematicsModule<Scalar>::getJointPositions(
  const unsigned& startIndex,
  const unsigned& nElements,
  const JointStateType& type)
{
  Matrix<Scalar, Dynamic, 1> positions;
  positions.resize(nElements);
  for (size_t i = startIndex; i < startIndex + nElements; ++i) {
    positions[i] = joints[i]->states[(unsigned)type]->position;
  }
  return positions;
}

template <typename Scalar>
boost::shared_ptr<LinkInfo<Scalar> > 
KinematicsModule<Scalar>::getLink(const unsigned& index)
{
  return links[index];
}

template <typename Scalar>
boost::shared_ptr<LinkChain<Scalar> > 
KinematicsModule<Scalar>::getLinkChain(const unsigned& index)
{
  return linkChains[index];
}

template <typename Scalar>
Matrix<Scalar, 4, 4>  KinematicsModule<Scalar>::getEndEffector(
  const unsigned& chain, const unsigned& index)
{
  return linkChains[chain]->endEffectors[index];
}

template <typename Scalar>
boost::shared_ptr<TorsoState<Scalar> > 
KinematicsModule<Scalar>::getTorsoState() 
{ 
  return torsoState; 
}

template <typename Scalar>
int KinematicsModule<Scalar>::getFootOnGround()
{
  return footOnGround;
}

template <typename Scalar>
Scalar KinematicsModule<Scalar>::getCycleTime()
{
  return cycleTime;
}

template class KinematicsModule<MType>;
