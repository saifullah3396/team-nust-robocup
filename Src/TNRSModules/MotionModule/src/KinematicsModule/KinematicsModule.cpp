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

#include "MotionModule/include/KinematicsModule/KinematicsModule.h"

KinematicsModule::KinematicsModule(MotionModule* motionModule) :  MemoryBase(motionModule), motionModule(motionModule),
    linkTs(NUM_JOINTUSAGE_TYPES), initTs(CHAINS_SIZE), finalTs(CHAINS_SIZE),
    endEffectors(CHAINS_SIZE), chainMasses(CHAINS_SIZE),
    prevJointPositions(NUM_JOINTUSAGE_TYPES),
    prevJointVelocities(NUM_JOINTUSAGE_TYPES), jointPositions(NUM_JOINTUSAGE_TYPES),
    jointVelocities(NUM_JOINTUSAGE_TYPES), jointAccelerations(NUM_JOINTUSAGE_TYPES),
    linkComs(NUM_JOINTS + TORSO_SIZE), linkMasses(NUM_JOINTS + TORSO_SIZE),
    linkInertias(NUM_JOINTS + TORSO_SIZE), jointULimits(NUM_JOINTS),
    jointLLimits(NUM_JOINTS), jointVLimits(NUM_JOINTS), footOnGround(L_FOOT),
    ffBufferSize(15)
{
  GET_CONFIG( "KinCalibration", (float, torsoPitchOffset, torsoPitchOffset), )
  initKinematicsModel();
  /*updateModel();
  Vector3f extForces;
  Vector3f extMoments;
  Vector3f torsoForces;
  Vector3f torsoMoments;
  extForces.setZero();
  extMoments.setZero();
  torsoForces.setZero();
  torsoMoments.setZero();
  jointPositions[ACTUAL].setZero();
  jointVelocities[ACTUAL].setZero();
  jointAccelerations[ACTUAL].setZero();
  for (int i = 0; i < CHAINS_SIZE; ++i) {
    Vector3f chainForces;
    Vector3f chainMoments;
    VectorXf torque = 
      newtonEulerForces(
        ACTUAL, i, extForces, extMoments, chainForces, chainMoments);
    torsoForces += chainForces;
    torsoMoments += chainMoments;
  }
  computeZmp();*/
  //setupJointsPlot();
}

void
KinematicsModule::updateModel()
{
  updateJointPositions();
  updateJointVelocities();
  updateJointAccelerations();
  updateTorsoState();
  prepareDHTransforms();
  updateFootToCamT();
  updateFootOnGround();
  updateTorsoToFeet();
  //computeZmp(CHAIN_L_LEG, KinematicsModule::ACTUAL);
  //cout << "footToCam: " << OVAR(Matrix4f, MotionModule::upperCamInFeet) << endl;
  //Matrix4f forwardTransform =
  //    MathsUtils::getTInverse(getForwardEffector(KinematicsModule::ACTUAL, CHAIN_L_LEG, ANKLE)) *
  //    getForwardEffector(KinematicsModule::ACTUAL, CHAIN_R_LEG, ANKLE);
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  //Matrix4f forwardTransform =OVAR(Matrix4f, MotionModule::upperCamInFeet);
  //Matrix4f forwardTransform =OVAR(Matrix4f, MotionModule::lowerCamInFeet);
  //cout << "ft: " << endl << forwardTransform.format(OctaveFmt) << endl;
  //cout << "limbJ: " << endl << computeLimbComJ(ACTUAL, CHAIN_L_LEG) << endl;
  //computeMassMatrix(ACTUAL, CHAIN_L_LEG).format(OctaveFmt);
  //cout << "R"<< endl << computeMassMatrix(ACTUAL, CHAIN_R_LEG).format(OctaveFmt) << endl;
  /*float vm;
  Matrix4f endEffector;
  endEffector << 1, 0, 0, 0.10272,
                 0, 1, 0, 0.0,
                 0, 0, 1, -0.03519,
                 0, 0, 0, 1;
  computeVirtualMass(
    CHAIN_R_LEG,
    Vector3f(1, 0, 0),
    endEffector,
    vm);
  cout << "virtual mass 1, 0, 0: " << vm << endl;
  computeVirtualMass(
    CHAIN_R_LEG,
    Vector3f(cos(M_PI/4), sin(M_PI/4), 0),
    endEffector,
    vm);
  cout << "virtual mass 0.707, 0.707, 0: " << vm << endl;
  computeVirtualMass(
    CHAIN_R_LEG,
    Vector3f(0, 0, 1),
    endEffector,
    vm);
  cout << "virtual mass 0, 0, 1: " << vm << endl;*/
  //printKinematicData();    
  //cout << "HERE" << endl;
  //cout << "forwardT: " << endl << forwardTransform << endl;
  //plotJointState(ACTUAL, HEAD_YAW);
}

void
KinematicsModule::printKinematicData()
{
  IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  cout << "-------------------________________Printing Kinematic Data________________-------------------" << endl;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cout << "Joint[" << jointNames[i] << "]" << endl;
    cout << "Position:" << jointPositions[ACTUAL][i] * 180 << ",  ";
    cout << "Velocity:" << jointVelocities[ACTUAL][i] << ",  ";
    cout << "Acceleration:" << jointAccelerations[ACTUAL][i] << endl;
    cout << "Com: [" << linkComs[i].format(OctaveFmt) << " ,  Mass:" << linkMasses[i] << endl;
    cout << "Inertias: " << endl << linkInertias[i].format(OctaveFmt) << endl << " ------------------------ " << endl;
  }
  cout << "[Torso]" << endl;
  cout << "Com: [" << linkComs[NUM_JOINTS][0] << ", " << linkComs[NUM_JOINTS][1] << ", " << linkComs[NUM_JOINTS][2] << "]" << ",  Mass:" << linkMasses[NUM_JOINTS] << endl;
  cout << "Inertia: " << endl << linkInertias[NUM_JOINTS].format(OctaveFmt) << endl;
}

void
KinematicsModule::setupJointsPlot()
{
  jointStateLogPath = ConfigManager::getLogsDirPath() + "KinematicsModule/JointState.txt";
  jointStateLog.open(
    jointStateLogPath.c_str(),
    std::ofstream::out | std::ofstream::trunc);
  jointStateLog.close();
  //gp << "set xrange [0:20]\nset yrange [0:20]\n";
  //gp << "plot '" << jointStateLogPath << "' using 1:2 with lines title 'Joint Position'" << endl;
}

void
KinematicsModule::plotJointState(const unsigned& index, const JointUsageType& type)
{
  float time = motionModule->getModuleTime();
  jointStateLog.open(jointStateLogPath.c_str(), fstream::app | fstream::out);
  jointStateLog << time << " " << jointPositions[type][index] << " " << jointVelocities[type][index] << " " << jointAccelerations[type][index] << "\n";
  jointStateLog.close();
  //cout << "time: " << time << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:2 with lines title 'Joint Position'" << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:3 with lines title 'Joint Velocity'" << endl;
  //gp << "replot '" << jointStateLogPath << "' using 1:4 with lines title 'Joint Acceleration'" << endl;
}

void
KinematicsModule::updateJointPositions()
{
  try {
    prevJointPositions[ACTUAL] = jointPositions[ACTUAL];
    vector<float> posSensors = IVAR(
      vector<float>,
      MotionModule::jointPositionSensors
    );
    jointPositions[ACTUAL] = VectorXf::Map(
      posSensors.data(),
      posSensors.size());
    //cout << "jointPositions" << endl;
    //cout << jointPositions[ACTUAL] * 180/ M_PI << endl;
  } catch (const exception& e) {
    cout << e.what();
  }
}

void
KinematicsModule::updateJointVelocities()
{
  try {
    prevJointVelocities[ACTUAL] = jointVelocities[ACTUAL];
    jointVelocities[ACTUAL] =
      (jointPositions[ACTUAL] - prevJointPositions[ACTUAL]) / cycleTime;
  } catch (const exception& e) {
    cout << e.what();
  }
}

void
KinematicsModule::updateJointAccelerations()
{
  try {
    jointAccelerations[ACTUAL] =
      (jointVelocities[ACTUAL] - prevJointVelocities[ACTUAL]) / cycleTime;
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

void
KinematicsModule::updateTorsoState()
{
  try {
    // Need a kalman filter for this tracker.
    // Nao sensors are updated almost with cycle time of 40ms in memory
    // Whereas motion module runs at 10ms
    auto& inertial = IVAR(vector<float>, MotionModule::inertialSensors);
    torsoState.accel[0] = inertial[ACCELEROMETER_X];
    torsoState.accel[1] = inertial[ACCELEROMETER_Y];
    torsoState.accel[2] = inertial[ACCELEROMETER_Z];    
    MathsUtils::makeRotationXYZ(
      torsoState.rot,
      (float) inertial[TORSO_ANGLE_X],
      (float) inertial[TORSO_ANGLE_Y],
      0.f);
    torsoState.accel = torsoState.rot * torsoState.accel;
    torsoState.velocity = torsoState.velocity + torsoState.accel * cycleTime;
    //cout << "torsoState.rot:\n" << torsoState.rot << endl;
    //cout << "torso velocity:\n" << torsoState.velocity << endl;
    //cout << "torso accel:\n" << torsoState.accel << endl;
  } catch (const exception& e) {
    cout << e.what();
  }
}

void
KinematicsModule::initKinematicsModel()
{
  cycleTime = motionModule->getPeriodMinMS() / ((float) 1000);
  //!cout << "cycleTimeKinematics: " << cycleTime << endl;
  Matrix3f iMat = Matrix3f::Identity();
  Matrix4f t1, t2;
  for (unsigned i = 0; i < NUM_JOINTS + 1; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      for (unsigned k = 0; k < 3; ++k) {
        linkInertias[i](j, k) = InertiaMatrices[j + i * 3][k];
      }
    }
    //cout << linkInertias[i] << endl;
  }

  for (int i = 0; i < NUM_JOINTUSAGE_TYPES; ++i) {
    jointPositions[i].resize(NUM_JOINTS);
    jointVelocities[i].resize(NUM_JOINTS);
    jointAccelerations[i].resize(NUM_JOINTS);
    prevJointPositions[i].resize(NUM_JOINTS);
    prevJointVelocities[i].resize(NUM_JOINTS);
    jointPositions[i].setZero();
    jointVelocities[i].setZero();
    jointAccelerations[i].setZero();
    prevJointPositions[i].setZero();
    prevJointVelocities[i].setZero();
    linkTs[i].resize(NUM_JOINTS);
    for (unsigned j = 0; j < NUM_JOINTS; j++) {
      linkTs[i][j] = Matrix4f::Identity();
    }
  }

  jointULimits[0] = headYawHigh;
  jointULimits[1] = headPitchHigh;
  jointULimits[2] = lShoulderPitchHigh;
  jointULimits[3] = lShoulderRollHigh;
  jointULimits[4] = lElbowYawHigh;
  jointULimits[5] = lElbowRollHigh;
  jointULimits[6] = lWristYawHigh;
  jointULimits[7] = rShoulderPitchHigh;
  jointULimits[8] = rShoulderRollHigh;
  jointULimits[9] = rElbowYawHigh;
  jointULimits[10] = rElbowRollHigh;
  jointULimits[11] = rWristYawHigh;
  jointULimits[12] = lHipYawPitchHigh;
  jointULimits[13] = lHipRollHigh;
  jointULimits[14] = lHipPitchHigh;
  jointULimits[15] = lKneePitchHigh;
  jointULimits[16] = lAnklePitchHigh;
  jointULimits[17] = lAnkleRollHigh;
  jointULimits[18] = rHipYawPitchHigh;
  jointULimits[19] = rHipRollHigh;
  jointULimits[20] = rHipPitchHigh;
  jointULimits[21] = rKneePitchHigh;
  jointULimits[22] = rAnklePitchHigh;
  jointULimits[23] = rAnkleRollHigh;

  jointLLimits[0] = headYawLow;
  jointLLimits[1] = headPitchLow;
  jointLLimits[2] = lShoulderPitchLow;
  jointLLimits[3] = lShoulderRollLow;
  jointLLimits[4] = lElbowYawLow;
  jointLLimits[5] = lElbowRollLow;
  jointLLimits[6] = lWristYawLow;
  jointLLimits[7] = rShoulderPitchLow;
  jointLLimits[8] = rShoulderRollLow;
  jointLLimits[9] = rElbowYawLow;
  jointLLimits[10] = rElbowRollLow;
  jointLLimits[11] = rWristYawLow;
  jointLLimits[12] = lHipYawPitchLow;
  jointLLimits[13] = lHipRollLow;
  jointLLimits[14] = lHipPitchLow;
  jointLLimits[15] = lKneePitchLow;
  jointLLimits[16] = lAnklePitchLow;
  jointLLimits[17] = lAnkleRollLow;
  jointLLimits[18] = rHipYawPitchLow;
  jointLLimits[19] = rHipRollLow;
  jointLLimits[20] = rHipPitchLow;
  jointLLimits[21] = rKneePitchLow;
  jointLLimits[22] = rAnklePitchLow;
  jointLLimits[23] = rAnkleRollLow;

  jointVLimits[0] = headYawVelLimit;
  jointVLimits[1] = headPitchVelLimit;
  jointVLimits[2] = lShoulderPitchVelLimit;
  jointVLimits[3] = lShoulderRollVelLimit;
  jointVLimits[4] = lElbowYawVelLimit;
  jointVLimits[5] = lElbowRollVelLimit;
  jointVLimits[6] = lWristYawVelLimit;
  jointVLimits[7] = rShoulderPitchVelLimit;
  jointVLimits[8] = rShoulderRollVelLimit;
  jointVLimits[9] = rElbowYawVelLimit;
  jointVLimits[10] = rElbowRollVelLimit;
  jointVLimits[11] = rWristYawVelLimit;
  jointVLimits[12] = lHipYawPitchVelLimit;
  jointVLimits[13] = lHipRollVelLimit;
  jointVLimits[14] = lHipPitchVelLimit;
  jointVLimits[15] = lKneePitchVelLimit;
  jointVLimits[16] = lAnklePitchVelLimit;
  jointVLimits[17] = lAnkleRollVelLimit;
  jointVLimits[18] = rHipYawPitchVelLimit;
  jointVLimits[19] = rHipRollVelLimit;
  jointVLimits[20] = rHipPitchVelLimit;
  jointVLimits[21] = rKneePitchVelLimit;
  jointVLimits[22] = rAnklePitchVelLimit;
  jointVLimits[23] = rAnkleRollVelLimit;

  chainSizes.push_back(HEAD_SIZE);
  chainSizes.push_back(L_ARM_SIZE);
  chainSizes.push_back(R_ARM_SIZE);
  chainSizes.push_back(L_LEG_SIZE);
  chainSizes.push_back(R_LEG_SIZE);
  chainStarts.push_back(HEAD_YAW);
  chainStarts.push_back(L_SHOULDER_PITCH);
  chainStarts.push_back(R_SHOULDER_PITCH);
  chainStarts.push_back(L_HIP_YAW_PITCH);
  chainStarts.push_back(R_HIP_YAW_PITCH);

  //!torso mass and center of mass definitions.
  linkComs[NUM_JOINTS + TORSO] = Vector4f(torsoX, torsoY, torsoZ, 1.0f);
  linkMasses[NUM_JOINTS + TORSO] = torsoMass;
  //!----------------------Head Start------------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    initTs[CHAIN_HEAD],
    (float) 0.0,
    (float) 0.0,
    (float) neckOffsetZ);
  MathsUtils::makeRotationXYZ(
    finalTs[CHAIN_HEAD],
    (float) M_PI_2,
    (float) M_PI_2,
    (float) 0.0);

  //!Masses
  linkMasses[HEAD_YAW] = headYawMass;
  linkMasses[HEAD_PITCH] = headPitchMass;

  //!Center of mass vectors.
  linkComs[HEAD_YAW] = Vector4f(headYawX, headYawY, headYawZ, 1.0f);
  linkComs[HEAD_PITCH] = Vector4f(headPitchX, headPitchY, headPitchZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[HEAD_PITCH] = finalTs[CHAIN_HEAD] * linkComs[HEAD_PITCH];

  //!Transforming inertia tensor from the given frame to the joint frame.
  linkInertias[HEAD_PITCH] =
    finalTs[CHAIN_HEAD].block(0, 0, 3, 3) * linkInertias[HEAD_PITCH] * (finalTs[CHAIN_HEAD].block(
      0,
      0,
      3,
      3).transpose());
  //!----------------------Head End------------------------!//

  //!-------------------Right Arm Start--------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    initTs[CHAIN_R_ARM],
    (float) 0.0,
    (float) -(shoulderOffsetY),
    (float) shoulderOffsetZ);
  MathsUtils::makeRotationXYZ(
    finalTs[CHAIN_R_ARM],
    (float) -M_PI_2,
    (float) 0.0,
    (float) -M_PI_2);
  MathsUtils::makeTranslation(
    t1,
    (float) (lowerArmLength + handOffsetX),
    (float) 0.0,
    (float) -handOffsetZ);
  finalTs[CHAIN_R_ARM] *= t1;

  //!Masses
  linkMasses[R_SHOULDER_PITCH] = rShoulderPitchMass;
  linkMasses[R_SHOULDER_ROLL] = rShoulderRollMass;
  linkMasses[R_ELBOW_YAW] = rElbowYawMass;
  linkMasses[R_ELBOW_ROLL] = rElbowRollMass;
  linkMasses[R_WRIST_YAW] = rWristYawMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) 0.0, (float) 0.0);
  linkComs[R_SHOULDER_PITCH] = Vector4f(
    rShoulderPitchX,
    rShoulderPitchY,
    rShoulderPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_SHOULDER_PITCH] = t1 * linkComs[R_SHOULDER_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_SHOULDER_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[R_SHOULDER_PITCH] * (t1.block(
      0,
      0,
      3,
      3).transpose());

  /**
   * Both left and right have same transformations
   * hence Rotation Matrices
   */
  linkInertias[L_SHOULDER_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[L_SHOULDER_PITCH] * (t1.block(
      0,
      0,
      3,
      3).transpose());

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) 0.0, (float) 0.0, (float) -M_PI_2);
  linkComs[R_SHOULDER_ROLL] = Vector4f(
    rShoulderRollX,
    rShoulderRollY,
    rShoulderRollZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_SHOULDER_ROLL] = t1 * linkComs[R_SHOULDER_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_SHOULDER_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[R_SHOULDER_ROLL] * (t1.block(0, 0, 3, 3).transpose());

  /**
   * Both left and right have same floatransformations
   * hence Rotation Matrices
   */
  linkInertias[L_SHOULDER_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[L_SHOULDER_ROLL] * (t1.block(0, 0, 3, 3).transpose());

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (float) -M_PI_2,
    (float) 0.0,
    (float) -M_PI_2);
  linkComs[R_ELBOW_YAW] = Vector4f(rElbowYawX, rElbowYawY, rElbowYawZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_ELBOW_YAW] = t1 * linkComs[R_ELBOW_YAW];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_ELBOW_YAW] =
    t1.block(0, 0, 3, 3) * linkInertias[R_ELBOW_YAW] * (t1.block(0, 0, 3, 3).transpose());

  /**
   * Both left and right have same floatransformations
   * hence Rotation Matrices
   */
  linkInertias[L_ELBOW_YAW] =
    t1.block(0, 0, 3, 3) * linkInertias[L_ELBOW_YAW] * (t1.block(0, 0, 3, 3).transpose());

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) 0.0, (float) 0.0, (float) -M_PI_2);
  linkComs[R_ELBOW_ROLL] = Vector4f(
    rElbowRollX,
    rElbowRollY,
    rElbowRollZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_ELBOW_ROLL] = t1 * linkComs[R_ELBOW_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_ELBOW_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[R_ELBOW_ROLL] * (t1.block(0, 0, 3, 3).transpose());

  /**
   * Both left and right have same floatransformations
   * hence Rotation Matrices
   */
  linkInertias[L_ELBOW_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[L_ELBOW_ROLL] * (t1.block(0, 0, 3, 3).transpose());

  //!Center of mass vectors.
  MathsUtils::makeRotationZYX(t1, (float) M_PI_2, (float) 0.0, (float) M_PI_2);
  linkComs[R_WRIST_YAW] = Vector4f(rWristYawX, rWristYawY, rWristYawZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_WRIST_YAW] = t1 * linkComs[R_WRIST_YAW];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_WRIST_YAW] =
    t1.block(0, 0, 3, 3) * linkInertias[R_WRIST_YAW] * t1.block(0, 0, 3, 3).transpose();

  /**
   * Both left and right have same floatransformations
   * hence Rotation Matrices
   */
  linkInertias[L_WRIST_YAW] =
    t1.block(0, 0, 3, 3) * linkInertias[L_WRIST_YAW] * t1.block(0, 0, 3, 3).transpose();
  //!-------------------Right Arm End----------------------!//

  //!-------------------Left Arm Start--------------------!//

  //!End and base transformations.
  MathsUtils::makeTranslation(
    initTs[CHAIN_L_ARM],
    (float) 0.0,
    (float) shoulderOffsetY,
    (float) shoulderOffsetZ);
  finalTs[CHAIN_L_ARM] = finalTs[CHAIN_R_ARM];

  //!Masses and center of mass vectors
  for (unsigned i = 0; i < L_ARM_SIZE; i++) {
    linkComs[chainStarts[CHAIN_L_ARM] + i] =
      linkComs[chainStarts[CHAIN_R_ARM] + i];
    linkMasses[chainStarts[CHAIN_L_ARM] + i] =
      linkMasses[chainStarts[CHAIN_R_ARM] + i];
  }

  //!Fixing the center of mass coordinates
  linkComs[L_SHOULDER_PITCH](2) = -linkComs[L_SHOULDER_PITCH](2);
  linkComs[L_SHOULDER_ROLL](0) = -linkComs[L_SHOULDER_ROLL](0);
  linkComs[L_ELBOW_YAW](0) = -linkComs[L_ELBOW_YAW](0);
  linkComs[L_ELBOW_ROLL](0) = -linkComs[L_ELBOW_ROLL](0);
  linkComs[L_WRIST_YAW](0) = -linkComs[L_ELBOW_ROLL](0);

  //!-------------------Left Arm End--------------------!//

  //!------------------Right Leg Start------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    initTs[CHAIN_R_LEG],
    (float) 0.0,
    (float) -hipOffsetY,
    (float) -hipOffsetZ);
  MathsUtils::makeRotationZYX(
    finalTs[CHAIN_R_LEG],
    (float) M_PI,
    (float) -M_PI_2,
    (float) 0.0);
  rotRLeg = finalTs[CHAIN_R_LEG];
  MathsUtils::makeTranslation(t1, (float) 0.0, (float) 0.0, (float) 0.0);
  finalTs[CHAIN_R_LEG] *= t1;

  //!Masses
  linkMasses[R_HIP_YAW_PITCH] = rHipYawPitchMass;
  linkMasses[R_HIP_ROLL] = rHipRollMass;
  linkMasses[R_HIP_PITCH] = rHipPitchMass;
  linkMasses[R_KNEE_PITCH] = rKneePitchMass;
  linkMasses[R_ANKLE_PITCH] = rAnklePitchMass;
  linkMasses[R_ANKLE_ROLL] = rAnkleRollMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (float) -M_PI_2 / 2,
    (float) 0.0,
    (float) -M_PI_2);
  linkComs[R_HIP_YAW_PITCH] = Vector4f(
    rHipYawPitchX,
    rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);
  linkComs[R_HIP_YAW_PITCH] = t1 * linkComs[R_HIP_YAW_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_HIP_YAW_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[R_HIP_YAW_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI, (float) M_PI_2, (float) 0.0);
  linkComs[R_HIP_ROLL] = Vector4f(rHipRollX, rHipRollY, rHipRollZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_HIP_ROLL] = t1 * linkComs[R_HIP_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_HIP_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[R_HIP_ROLL] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[R_HIP_PITCH] = Vector4f(rHipPitchX, rHipPitchY, rHipPitchZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_HIP_PITCH] = t1 * linkComs[R_HIP_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_HIP_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[R_HIP_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[R_KNEE_PITCH] = Vector4f(
    rKneePitchX,
    rKneePitchY,
    rKneePitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_KNEE_PITCH] = t1 * linkComs[R_KNEE_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_KNEE_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[R_KNEE_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[R_ANKLE_PITCH] = Vector4f(
    rAnklePitchX,
    rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_ANKLE_PITCH] = t1 * linkComs[R_ANKLE_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_ANKLE_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[R_ANKLE_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI, (float) M_PI_2, (float) 0.0);
  linkComs[R_ANKLE_ROLL] = Vector4f(
    rAnkleRollX,
    rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[R_ANKLE_ROLL] = t1 * linkComs[R_ANKLE_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[R_ANKLE_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[R_ANKLE_ROLL] * t1.block(0, 0, 3, 3).transpose();
  //!------------------Right Leg Start------------------!//

  //!------------------Left Leg Start-------------------!//
  //!End and base transformations.
  MathsUtils::makeTranslation(
    initTs[CHAIN_L_LEG],
    (float) 0.0,
    (float) hipOffsetY,
    (float) -hipOffsetZ);
  tBaseLLegInv = initTs[CHAIN_L_LEG];
  tBaseLLegInv = MathsUtils::getTInverse(tBaseLLegInv);
  MathsUtils::makeRotationZYX(
    finalTs[CHAIN_L_LEG],
    (float) M_PI,
    (float) -M_PI_2,
    (float) 0.0);
  MathsUtils::makeRotationXYZ(
    rotFixLLeg,
    (float) M_PI_4,
    (float) 0.0,
    (float) 0.0);
  MathsUtils::makeTranslation(t1, (float) 0.0, (float) 0.0, (float) 0.0);
  finalTs[CHAIN_L_LEG] *= t1;
  tEndLLegInv = t1;
  tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);

  //!Masses
  linkMasses[L_HIP_YAW_PITCH] = rHipYawPitchMass;
  linkMasses[L_HIP_ROLL] = rHipRollMass;
  linkMasses[L_HIP_PITCH] = rHipPitchMass;
  linkMasses[L_KNEE_PITCH] = rKneePitchMass;
  linkMasses[L_ANKLE_PITCH] = rAnklePitchMass;
  linkMasses[L_ANKLE_ROLL] = rAnkleRollMass;

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(
    t1,
    (float) -(3 * M_PI) / 4,
    (float) 0.0,
    (float) -M_PI_2);
  linkComs[L_HIP_YAW_PITCH] = Vector4f(
    rHipYawPitchX,
    -rHipYawPitchY,
    rHipYawPitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  t1 = MathsUtils::getTInverse(t1);
  linkComs[L_HIP_YAW_PITCH] = t1 * linkComs[L_HIP_YAW_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_HIP_YAW_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[L_HIP_YAW_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI, (float) M_PI_2, (float) 0.0);
  linkComs[L_HIP_ROLL] = Vector4f(rHipRollX, -rHipRollY, rHipRollZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[L_HIP_ROLL] = t1 * linkComs[L_HIP_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_HIP_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[L_HIP_ROLL] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[L_HIP_PITCH] = Vector4f(rHipPitchX, -rHipPitchY, rHipPitchZ, 1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[L_HIP_PITCH] = t1 * linkComs[L_HIP_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_HIP_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[L_HIP_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[L_KNEE_PITCH] = Vector4f(
    rKneePitchX,
    -rKneePitchY,
    rKneePitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[L_KNEE_PITCH] = t1 * linkComs[L_KNEE_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_KNEE_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[L_KNEE_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI_2, (float) M_PI_2, (float) 0.0);
  linkComs[L_ANKLE_PITCH] = Vector4f(
    rAnklePitchX,
    -rAnklePitchY,
    rAnklePitchZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[L_ANKLE_PITCH] = t1 * linkComs[L_ANKLE_PITCH];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_ANKLE_PITCH] =
    t1.block(0, 0, 3, 3) * linkInertias[L_ANKLE_PITCH] * t1.block(0, 0, 3, 3).transpose();

  //!Center of mass vectors.
  MathsUtils::makeRotationXYZ(t1, (float) M_PI, (float) M_PI_2, (float) 0.0);
  linkComs[L_ANKLE_ROLL] = Vector4f(
    rAnkleRollX,
    -rAnkleRollY,
    rAnkleRollZ,
    1.0f);

  //!Fixing the coordinate system of center of mass.
  linkComs[L_ANKLE_ROLL] = t1 * linkComs[L_ANKLE_ROLL];

  //!Fixing the Inertia tensor rotation.
  linkInertias[L_ANKLE_ROLL] =
    t1.block(0, 0, 3, 3) * linkInertias[L_ANKLE_ROLL] * t1.block(0, 0, 3, 3).transpose();

  //!------------------Left Leg End-------------------!//

  //!Moving Inertias of all joints to to center of mass
  for (unsigned i = 0; i < NUM_JOINTS + 1; ++i) {
    linkInertias[i] =
      linkInertias[i] - linkMasses[i] * ((linkComs[i].segment(0, 3).transpose() * linkComs[i].segment(
        0,
        3))(0, 0) * iMat - linkComs[i].segment(0, 3) * linkComs[i].segment(0, 3).transpose());
  }

  //!Definition of effectors.
  endEffectors[CHAIN_HEAD].resize(NUM_CAMS);
  endEffectors[CHAIN_L_LEG].resize(NUM_LEG_EES);
  endEffectors[CHAIN_R_LEG].resize(NUM_LEG_EES);
  endEffectors[CHAIN_L_ARM].push_back(Matrix4f::Identity());
  endEffectors[CHAIN_R_ARM].push_back(Matrix4f::Identity());
  MathsUtils::makeTranslation(
    endEffectors[CHAIN_HEAD][TOP_CAM],
    (float) cameraTopX,
    (float) 0.0,
    (float) cameraTopZ);
  MathsUtils::makeTranslation(
    endEffectors[CHAIN_HEAD][BOTTOM_CAM],
    (float) cameraBottomX,
    (float) 0.0,
    (float) cameraBottomZ);
  MathsUtils::makeRotationXYZ(t1, (float) 0.0, (float) M_PI_2, (float) -M_PI_2);
  endEffectors[CHAIN_HEAD][TOP_CAM] = endEffectors[CHAIN_HEAD][TOP_CAM] * t1;
  endEffectors[CHAIN_HEAD][BOTTOM_CAM] =
    endEffectors[CHAIN_HEAD][BOTTOM_CAM] * t1;
  MathsUtils::makeRotationXYZ(t1, (float) -cameraTopAngleY, //Y becomes -X after frame transfomration
    (float) 0.0,
    (float) 0.0);
  endEffectors[CHAIN_HEAD][TOP_CAM] = endEffectors[CHAIN_HEAD][TOP_CAM] * t1;
  MathsUtils::makeRotationXYZ(t1, (float) -cameraBotAngleY, //Y becomes -X after frame transfomration
    (float) 0.0,
    (float) 0.0);
  endEffectors[CHAIN_HEAD][BOTTOM_CAM] =
    endEffectors[CHAIN_HEAD][BOTTOM_CAM] * t1;

  //cout << "Camera EEs" << endl;
  //cout << endEffectors[CHAIN_HEAD][TOP_CAM] << endl;
  //cout << endEffectors[CHAIN_HEAD][BOTTOM_CAM] << endl;

  endEffectors[CHAIN_L_LEG][ANKLE] = Matrix4f::Identity();
  endEffectors[CHAIN_R_LEG][ANKLE] = Matrix4f::Identity();
  MathsUtils::makeTranslation(
    endEffectors[CHAIN_L_LEG][FEET_BASE],
    (float) 0.0,
    (float) 0.0,
    (float) -footHeight);
  endEffectors[CHAIN_R_LEG][FEET_BASE] = endEffectors[CHAIN_L_LEG][FEET_BASE];
  float totalChainsMass = 0;
  for (int i = 0; i < chainSizes.size(); ++i) {
    chainMasses[i] = 0;
    for (int j = 0; j < chainSizes[i]; ++j) {
      chainMasses[i] += linkMasses[chainStarts[i] + j];
      totalChainsMass += linkMasses[chainStarts[i] + j];
    }
    //cout << "chainMasses[i] " << i << "  "<< chainMasses[i] << endl; // 4.25579 kg
  }
  //cout << "totalChainsMass:  " << totalChainsMass << endl;
  updateModel();

  feetForcesBuffer.set_capacity(ffBufferSize);
}

void
KinematicsModule::prepareDHTransforms(const unsigned& ch, const JointUsageType& type)
{
  Matrix4f t1;
  if (ch == CHAIN_HEAD || ch == CHAINS_SIZE) {
    MathsUtils::makeDHTransformation(
      linkTs[type][HEAD_YAW],
      (float) 0.0,
      (float) 0.0,
      (float) 0.0,
      (float) jointPositions[type][HEAD_YAW]);
    MathsUtils::makeDHTransformation(
      linkTs[type][HEAD_PITCH],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) (jointPositions[type][HEAD_PITCH] - M_PI_2));
  }
  if (ch == CHAIN_L_ARM || ch == CHAINS_SIZE) {
    MathsUtils::makeDHTransformation(
      linkTs[type][L_SHOULDER_PITCH],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][L_SHOULDER_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_SHOULDER_ROLL],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) (jointPositions[type][L_SHOULDER_ROLL] + M_PI_2));
    MathsUtils::makeDHTransformation(
      linkTs[type][L_ELBOW_YAW],
      (float) elbowOffsetY,
      (float) M_PI_2,
      (float) upperArmLength,
      (float) jointPositions[type][L_ELBOW_YAW]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_ELBOW_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][L_ELBOW_ROLL]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_WRIST_YAW],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][L_WRIST_YAW]);
  }
  if (ch == CHAIN_R_ARM || ch == CHAINS_SIZE) {
    MathsUtils::makeDHTransformation(
      linkTs[type][R_SHOULDER_PITCH],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][R_SHOULDER_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_SHOULDER_ROLL],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) (jointPositions[type][R_SHOULDER_ROLL] + M_PI_2));
    MathsUtils::makeDHTransformation(
      linkTs[type][R_ELBOW_YAW],
      (float) -elbowOffsetY,
      (float) M_PI_2,
      (float) upperArmLength,
      (float) jointPositions[type][R_ELBOW_YAW]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_ELBOW_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][R_ELBOW_ROLL]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_WRIST_YAW],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][R_WRIST_YAW]);
  }
  if (ch == CHAIN_L_LEG || ch == CHAINS_SIZE) {
    MathsUtils::makeDHTransformation(
      linkTs[type][L_HIP_YAW_PITCH],
      (float) 0.0,
      (float) (-3 * M_PI_4),
      (float) 0.0,
      (float) (jointPositions[type][L_HIP_YAW_PITCH] - M_PI_2));
    MathsUtils::makeDHTransformation(
      linkTs[type][L_HIP_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) (jointPositions[type][L_HIP_ROLL] + M_PI_4));
    MathsUtils::makeDHTransformation(
      linkTs[type][L_HIP_PITCH],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][L_HIP_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_KNEE_PITCH],
      (float) -thighLength,
      (float) 0.0,
      (float) 0.0,
      (float) jointPositions[type][L_KNEE_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_ANKLE_PITCH],
      (float) -tibiaLength,
      (float) 0.0,
      (float) 0.0,
      (float) jointPositions[type][L_ANKLE_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][L_ANKLE_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][L_ANKLE_ROLL]);
  }

  if (ch == CHAIN_R_LEG || ch == CHAINS_SIZE) {
    MathsUtils::makeDHTransformation(
      linkTs[type][R_HIP_YAW_PITCH],
      (float) 0.0,
      (float) -M_PI_4,
      (float) 0.0,
      (float) (jointPositions[type][R_HIP_YAW_PITCH] - M_PI_2));
    MathsUtils::makeDHTransformation(
      linkTs[type][R_HIP_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) (jointPositions[type][R_HIP_ROLL] - M_PI_4));
    MathsUtils::makeDHTransformation(
      linkTs[type][R_HIP_PITCH],
      (float) 0.0,
      (float) M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][R_HIP_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_KNEE_PITCH],
      (float) -thighLength,
      (float) 0.0,
      (float) 0.0,
      (float) jointPositions[type][R_KNEE_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_ANKLE_PITCH],
      (float) -tibiaLength,
      (float) 0.0,
      (float) 0.0,
      (float) jointPositions[type][R_ANKLE_PITCH]);
    MathsUtils::makeDHTransformation(
      linkTs[type][R_ANKLE_ROLL],
      (float) 0.0,
      (float) -M_PI_2,
      (float) 0.0,
      (float) jointPositions[type][R_ANKLE_ROLL]);
  }
}

void
KinematicsModule::setEndEffector(const unsigned& chain, const unsigned& eeIndex,
  const Vector4f& ee)
{
  Matrix4f t1;
  if (chain == CHAIN_R_LEG) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    endEffectors[chain][eeIndex] = t1;
    MathsUtils::makeTranslation(t1, ee[0], -ee[1], ee[2]);
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  }
  if (chain == CHAIN_L_LEG) {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    endEffectors[chain][eeIndex] = t1;
    tEndLLegInv = t1;
    tEndLLegInv = MathsUtils::getTInverse(tEndLLegInv);
  } else {
    MathsUtils::makeTranslation(t1, ee[0], ee[1], ee[2]);
    endEffectors[chain][eeIndex] = t1;
  }
}

Matrix4f
KinematicsModule::getForwardEffector(
  const unsigned& chainIndex, const Matrix4f &endEffector, const JointUsageType& type)
{
  Matrix4f t;
  t = initTs[chainIndex];
  for (unsigned i = 0; i < chainSizes[chainIndex]; i++) {
    t *= linkTs[type][chainStarts[chainIndex] + i];
  }
  t *= finalTs[chainIndex] * endEffector;
  return t;
}

Matrix4f
KinematicsModule::getForwardEffector(
  const unsigned& chainIndex, const unsigned& eeIndex, const JointUsageType& type)
{
  return getForwardEffector(chainIndex, endEffectors[chainIndex][eeIndex], type);
}

ComState 
KinematicsModule::getComState(const unsigned& baseFrame)
{
  // This process assumes torso velocity/acceleration is equal to 
  // center of mass acceleration
  // Another assumption is that the baseFrame rotation is always aligned 
  // with the gravity vector since torso accelerations are rotated 
  // according to the gravity vector
  // Since the robot's feet frames are always aligned with the ground,
  // this assumption is valid
  // These readings will also require a kalman filter for good com state
  if (baseFrame != CHAIN_L_LEG && baseFrame != CHAIN_R_LEG)
    return ComState();
  ComState com;
  com.baseFrame = baseFrame;
  getComWrtBase(baseFrame, FEET_BASE, com.position, ACTUAL);
  com.velocity = torsoState.velocity;
  com.accel = torsoState.accel;
  return com;
}

void
KinematicsModule::getComWrtBase(const unsigned& limbIndex,
  const unsigned& eeIndex, Vector3f &comVector, const JointUsageType& type)
{
  Vector3f comWrtTorso = calculateCenterOfMass(type);
  if (limbIndex != -1) {
    comVector = MathsUtils::transformVector(
      MathsUtils::getTInverse(getForwardEffector(limbIndex, eeIndex, type)),
      comWrtTorso);
  } else {
    comVector = comWrtTorso;
  }
}

void
KinematicsModule::getComWrtBase(const unsigned& limbIndex,
  const unsigned& eeIndex, Vector2f& comVector, const JointUsageType& type)
{
  Vector3f com;
  getComWrtBase(limbIndex, eeIndex, com, type);
  comVector[0] = com(0, 0);
  comVector[1] = com(1, 0);
}

VectorXf
KinematicsModule::cartToJointVels(
  const unsigned& chainIndex, const VectorXf cVels, const Matrix4f endEffector, const JointUsageType& type)
{
  MatrixXf jacobian = computeLimbJ(chainIndex, endEffector, type);
  return MathsUtils::pseudoInverseSolve(jacobian, cVels);
}

MatrixXf
KinematicsModule::computeLimbJ(const unsigned& chainIndex,
  const unsigned& eeIndex, const JointUsageType& type)
{
  return computeLimbJ(
    chainIndex,
    Vector4f(endEffectors[chainIndex][eeIndex].block<4, 1>(0, 3)),
    type);
}

MatrixXf
KinematicsModule::computeLimbJ(const unsigned& chainIndex,
  const Matrix4f& endEffector, const JointUsageType& type)
{
  return computeLimbJ(chainIndex, Vector4f(endEffector.block<4, 1>(0, 3)), type);
}

MatrixXf
KinematicsModule::computeLimbJ(const unsigned& chainIndex,
  const Vector4f& endEffector, const JointUsageType& type)
{
  unsigned size = chainSizes[chainIndex];
  unsigned chainStart = chainStarts[chainIndex];
  MatrixXf jacobian;
  vector<Vector3f> z;
  vector<Vector3f> pos;
  Matrix4f T;
  Vector3f fPos;
  jacobian.resize(6, size);
  z.resize(size);
  pos.resize(size);
  //cout << "chainStarts" << chainStarts << endl;
  T = initTs[chainIndex];
  for (int j = 0; j < size; ++j) {
    T = T * linkTs[type][chainStart + j];
    //cout << "T   " << T << endl;
    z[j] = T.block<3, 1>(0, 2);
    //cout << "z   " << z[j] << endl;
    pos[j] = T.block<3, 1>(0, 3);
    //cout << "pos   " << pos[j] << endl;
    jacobian.block<3, 1>(3, j) = z[j];
  }
  //cout << jacobian << endl;
  T = T * finalTs[chainIndex];
  fPos = (T * endEffector).block<3, 1>(0, 0);
  for (int j = 0; j < size; ++j) {
    jacobian.block<3, 1>(0, j) =
      MathsUtils::makeSkewMat(z[j]) * (fPos - pos[j]);
  }
  return jacobian;
}

MatrixXf
KinematicsModule::computeLimbComJ(
  const unsigned& chainIndex, const JointUsageType& type)
{
  unsigned size = chainSizes[chainIndex];
  unsigned chainStart = chainStarts[chainIndex];
  MatrixXf jacobianC;
  MatrixXf jacobianCV;
  MatrixXf jacobianCW;
  Vector3f comT;
  vector<Vector3f> z;
  vector<Vector3f> pos;
  Matrix4f T;
  Vector4f ee;
  Vector4f zero;
  float partialMass = 0;
  jacobianC.resize(6, size);
  jacobianCV.resize(3, size);
  jacobianCW.resize(3, size);
  z.resize(size);
  pos.resize(size);
  T.setIdentity();
  jacobianC.setZero();
  comT.setZero();
  zero.setZero();
  zero(3, 0) = 1;
  float chainMass = chainMasses[chainIndex];
  //cout << "chainStart" << chainStart << endl;
  T = initTs[chainIndex];
  for (int j = 0; j < size; ++j) {
    jacobianCV.setZero();
    jacobianCW.setZero();
    partialMass = linkMasses[chainStart + j];
    T = T * linkTs[type][chainStart + j];
    comT = (T * linkComs[chainStart + j]).block<3, 1>(0, 0);
    //cout << "T   " << T << endl;
    z[j] = T.block<3, 1>(0, 2);
    //cout << "z   " << z[j] << endl;
    pos[j] = T.block<3, 1>(0, 3);
    //cout << "pos   " << pos[j] << endl;
    for (int m = 0; m <= j; ++m) {
      jacobianCW.block<3, 1>(0, m) = z[m];
      jacobianCV.block<3, 1>(0, m) = MathsUtils::makeSkewMat(z[m]) *
      //(comT + pos[j] - pos[m]);
      (comT - pos[m]);
    }
    jacobianC.block(3, 0, 3, size) =
      jacobianC.block(3, 0, 3, size) + jacobianCW * (partialMass / chainMass);
    jacobianC.block(3, 0, 3, size) =
      jacobianC.block(3, 0, 3, size) + jacobianCV * (partialMass / chainMass);
  }
  return jacobianC;
}

VectorXf
KinematicsModule::solveComIK(const unsigned& baseLimb,
  const Matrix<float, 6, 1>& comVelocityD,
  const vector<unsigned>& limbMotionSpace,
  const vector<VectorXf>& limbVelocitiesD, const vector<int>& eeIndices, const JointUsageType& type)
{
  ASSERT(limbMotionSpace.size() == CHAINS_SIZE);
  ASSERT(eeIndices.size() == CHAINS_SIZE);
  ASSERT(baseLimb == CHAIN_L_LEG || baseLimb == CHAIN_R_LEG);
  vector<Matrix4f> limbTs(CHAINS_SIZE);
  Matrix3f rBase;
  Matrix<float, 6, 6> XBase;
  Matrix<float, 6, 6> Xo;
  vector<MatrixXf> limbComJs(CHAINS_SIZE);
  vector<MatrixXf> limbJs(CHAINS_SIZE);
  vector<Matrix<float, 6, 6> > XiWithBase(CHAINS_SIZE);
  vector<MatrixXf> Xinv(CHAINS_SIZE);
  Vector3f comWrtBaseLimb;
  limbTs[baseLimb] = getForwardEffector(baseLimb, eeIndices[baseLimb], type);
  rBase = MathsUtils::getTInverse(limbTs[baseLimb]).block(0, 0, 3, 3);
  Vector3f eeBase = limbTs[baseLimb].block(0, 3, 3, 1);
  XBase << Matrix3f::Identity(), rBase * MathsUtils::makeSkewMat(eeBase), Matrix3f::Zero(), Matrix3f::Identity();
  Xo << rBase, Matrix3f::Zero(), Matrix3f::Zero(), rBase;
  getComWrtBase(baseLimb, eeIndices[baseLimb], comWrtBaseLimb, type);
  MatrixXf baseComJ;
  baseComJ.resize(6, chainSizes[baseLimb]);
  baseComJ.setZero();
  Matrix<float, 6, 1> baseComVelocityD = comVelocityD;
  limbJs[baseLimb] = computeLimbJ(baseLimb, eeIndices[baseLimb], type);
  MatrixXf baseJT = XBase * limbJs[baseLimb];
  Vector3f comLimbsDiff = Vector3f::Zero();
  for (unsigned i = 0; i < CHAINS_SIZE; ++i) {
    limbComJs[i] = computeLimbComJ(i, type);
    limbComJs[i].block(0, 0, 3, chainSizes[i]) = rBase * limbComJs[i].block(
      0,
      0,
      3,
      chainSizes[i]);
    if (limbMotionSpace[i]) {
      if (i == baseLimb) {
        limbJs[i] = Xo * limbJs[i];
        baseComJ.block(3, 0, 3, chainSizes[i]) = -limbJs[i].block(
          3,
          0,
          3,
          chainSizes[i]);
        baseComJ.block(0, 0, 3, chainSizes[i]) =
          baseComJ.block(0, 0, 3, chainSizes[i]) + -limbJs[i].block(
            0,
            0,
            3,
            chainSizes[i]) + MathsUtils::makeSkewMat(comWrtBaseLimb) * limbJs[i].block(
            3,
            0,
            3,
            chainSizes[i]) + limbComJs[i].block(0, 0, 3, chainSizes[i]);
      } else {
        limbTs[i] = getForwardEffector(i, eeIndices[i], type);
        limbJs[i] = computeLimbJ(i, eeIndices[i], type);
        Vector3f ee = limbTs[i].block(0, 3, 3, 1);
        Matrix<float, 6, 6> Xi;
        Xi << Matrix3f::Identity(), rBase * MathsUtils::makeSkewMat(ee), Matrix3f::Zero(), Matrix3f::Identity();
        XiWithBase[i] = Xi.inverse() * baseJT;
        MatrixXf X = Xo * limbJs[i];
        Xinv[i] = MathsUtils::pseudoInverse(X);
        baseComJ.block(0, 0, 3, chainSizes[baseLimb]) =
          baseComJ.block(0, 0, 3, chainSizes[baseLimb]) + limbComJs[i].block(
            0,
            0,
            3,
            chainSizes[i]) * Xinv[i] * XiWithBase[i];
        comLimbsDiff =
          comLimbsDiff + limbComJs[i].block(0, 0, 3, chainSizes[i]) * Xinv[i] * limbVelocitiesD[i];
      }
    } else {
      comLimbsDiff =
        comLimbsDiff + limbComJs[i].block(0, 0, 3, chainSizes[i]) * limbVelocitiesD[i];
    }
  }
  baseComVelocityD.segment(0, 3) =
    baseComVelocityD.segment(0, 3) - comLimbsDiff;
  VectorXf jointD(NUM_JOINTS);
  vector<VectorXf> jointVD(CHAINS_SIZE);

  jointVD[baseLimb] = MathsUtils::pseudoInverseSolve(
    baseComJ,
    baseComVelocityD);
  for (unsigned i = 0; i < CHAINS_SIZE; ++i) {
    if (i != baseLimb) {
      jointVD[i].resize(chainSizes[i]);
      jointVD[i].setZero();
      if (limbMotionSpace[i]) {
        MatrixXf rhs = limbVelocitiesD[i] + XiWithBase[i] * jointVD[baseLimb];
        jointVD[i] = Xinv[i] * rhs;
      }
    }
    jointD.segment(chainStarts[i], chainSizes[i]) =
      jointPositions[type].segment(chainStarts[i], chainSizes[i]) + jointVD[i] * cycleTime;
  }
  return jointD;
}

MatrixXf
KinematicsModule::computeMassMatrix(
  const unsigned& chainIndex, const JointUsageType& type)
{
  unsigned size = chainSizes[chainIndex];
  unsigned chainStart = chainStarts[chainIndex];
  MatrixXf jacobianCV;
  MatrixXf jacobianCW;
  MatrixXf massMatrix;
  Vector3f comT;
  vector<Vector3f> z;
  vector<Vector3f> pos;
  Matrix4f T;
  Vector4f ee;
  Vector4f zero;
  jacobianCV.resize(3, size);
  jacobianCW.resize(3, size);
  massMatrix.resize(size, size);
  z.resize(size);
  pos.resize(size);
  T.setIdentity();
  jacobianCV.setZero();
  jacobianCW.setZero();
  massMatrix.setZero();
  comT.setZero();
  zero.setZero();
  zero(3, 0) = 1;
  //MatrixXf combined;
  //combined.resize(3, size);
  //combined.setZero();
  //float chainMass = chainMasses[chainIndex];
  T = initTs[chainIndex];
  for (int j = 0; j < size; ++j) {
    jacobianCV.setZero();
    jacobianCW.setZero();
    T = T * linkTs[type][chainStart + j];
    comT = (T * linkComs[chainStart + j]).block<3, 1>(0, 0);
    z[j] = T.block<3, 1>(0, 2);
    pos[j] = T.block<3, 1>(0, 3);
    for (int m = 0; m <= j; ++m) {
      jacobianCW.block<3, 1>(0, m) = z[m];
      jacobianCV.block<3, 1>(0, m) = MathsUtils::makeSkewMat(z[m]) *
      //(comT + pos[j] - pos[m]);
      (comT - pos[m]);
    }
    massMatrix =
      massMatrix + linkMasses[chainStart + j] * jacobianCV.transpose() * jacobianCV + jacobianCW.transpose() * linkInertias[chainStart + j] * jacobianCW;
    //cout << "combined: " << endl << combined << endl;
    //combined = combined + jacobianCV * (linkMasses[chainStart + j] / chainMass);
  }
  //cout << "combined: " << endl << combined << endl;
  //cout << massMatrix << endl;
  return massMatrix;
}

bool
KinematicsModule::computeVirtualMass(
  const unsigned& chainIndex, const Vector3f& direction,
  const Matrix4f& endEffector, float& virtualMass, const JointUsageType& type)
{
  unsigned size = chainSizes[chainIndex];
  unsigned chainStart = chainStarts[chainIndex];
  //VectorXf joints = getJointPositions(chainStart, size, type);
  ////! center of mass of final link
  //MatrixXf T = MathsUtils::getTInverse(finalTs[chainIndex]);
  //Vector4f lastCom = T * linkComs[chainStart + size - 1];
  //cout << "lastCom: " << lastCom << endl;

  //! Inertia matrix (size x size) at given joint configuration
  MatrixXf massMatrix = computeMassMatrix(chainIndex, type);
  //cout << "Mass matrix: " << endl << massMatrix << endl;

  //MatrixXf jacobian = computeLimbJ(chainIndex, lastCom, type);
  //cout << "Jacobian: " << endl << jacobian << endl;
  //! Jacobian matrix (6 x size) at given joint configuration
  MatrixXf jacobianEE = computeLimbJ(chainIndex, endEffector, type);
  ///cout << "JacobianEE: " << endl << jacobianEE << endl;

  //! Inertia matrix inverse (size x size) at given joint configuration
  MatrixXf mmInv = massMatrix.inverse();
  //cout << "Mass matrix Inv: " << endl << mmInv << endl;

  //! Inertial projection in cartesian space (6x6)
  //! (G = 6x6 Symmetric) [G11, G12;G21, G22]
  //MatrixXf gMatrix = jacobian * mmInv * jacobian.transpose();
  //cout << " gMatrix: " << endl <<  gMatrix << endl;
  MatrixXf gMatrixEE = jacobianEE * mmInv * jacobianEE.transpose();
  //cout << "G-matrixEE: " << endl << gMatrixEE << endl;

  //! Position vector from center of mass to end effector
  //Vector3f pos =
  //  endEffector.block(0, 3, 3, 1) - lastCom.block(0, 0, 3, 1);
  //Matrix3f skewPosT = MathsUtils::makeSkewMat(pos);
  //Matrix3f skewPos = skewPosT.transpose();
  //! Conversion of cartesian space inertia matrix from center of mass
  //! to the contact point using skewPos
  //Matrix3f g11 = gMatrix.block(0, 0, 3, 3);
  //Matrix3f g12 = gMatrix.block(0, 3, 3, 3);
  //Matrix3f g22 = gMatrix.block(3, 3, 3, 3);
  //MatrixXf transfMassMatrix =
  // g11 + skewPos * g12.transpose() + g12* skewPosT + skewPos * g22 * skewPosT;
  //cout << "G-matrix: " << endl << transfMassMatrix << endl;
  //cout << "G12: " << endl << g12 + skewPos *g22 << endl;
  //cout << "G22: " << endl << g22 << endl;
  MatrixXf g11 = gMatrixEE.block(0, 0, 3, 3);
  //! Virtual Mass in the target direction
  //virtualMass = direction.transpose() * transfMassMatrix * direction;
  virtualMass = direction.transpose() * g11 * direction;
  if (virtualMass != 0) {
    virtualMass = 1.f / virtualMass;
    //PRINT("Calculated virtual mass =" + DataUtils::varToString(virtualMass));
    return true;
  } else return false;
}

VectorXf
KinematicsModule::newtonEulerForces(
  const unsigned& chainIndex, const Vector3f& extForces,
  const Vector3f& extMoments, Vector3f& totalForces, Vector3f& totalMoments, 
  const unsigned& supportLeg, const JointUsageType& type)
{
  unsigned chainSize = chainSizes[chainIndex];
  unsigned chainStart = chainStarts[chainIndex];
  //!Forward Recursion
  Vector3f zAxis(0, 0, 1);
  Vector3f linAcc(0, 0, -gConst);
  Vector3f angVel(0, 0, 0);
  Vector3f angAcc(0, 0, 0);
  Vector3f linAccCom(0, 0, 0);
  vector<Vector3f> comForces(chainSize);
  vector<Vector3f> comMoments(chainSize);

  //! Rotating torso forces and moments to inertial frame situated at 
  //! the base support leg
  Matrix4f supportT = getForwardEffector(supportLeg, FEET_BASE, type);
  linAcc = MathsUtils::transformVector(supportT, linAcc);

  //cout << "chainSize" << chainSize << endl;
  //cout << "chainStart" << chainStart << endl;
  for (unsigned i = 0; i < chainSize; ++i) {
    Matrix4f tMat = linkTs[type][chainStart + i];
    Matrix3f rotMat = tMat.block(0, 0, 3, 3).transpose(); // transposed
    Vector3f transMat = tMat.block(0, 3, 3, 1);
    angVel = rotMat * angVel + zAxis * jointVelocities[type][i]; // good
    angAcc = rotMat * angAcc + (rotMat * angVel).cross(
      zAxis * jointVelocities[type][i]) + zAxis * jointAccelerations[type][i];  // good
    linAcc = rotMat * (angAcc.cross(transMat) + angVel.cross(
      angVel.cross(transMat)) + linAcc);  // good
    Vector3f comP = linkComs[chainStart + i].segment(0, 3);
    linAccCom =
      angAcc.cross(comP) + angVel.cross(angVel.cross(comP)) + linAcc;
    comForces[i] = linkMasses[chainStart + i] * linAccCom;
    comMoments[i] = linkInertias[chainStart + i] * angVel + angVel.cross(
      linkInertias[chainStart + i] * angVel);
    
    /*cout << "comForces[" << i << "]" << "      "<< comForces[i](0,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](1,0) << endl;
    cout << "comForces[" << i << "]" << "      "<< comForces[i](2,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](0,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](1,0) << endl;
    cout << "comMoments[" << i << "]" << "      "<< comMoments[i](2,0) << endl;
    cout<< endl;*/
  }

  //!Backward Recursion
  VectorXf jointTorques;
  jointTorques.resize(chainSize);
  Vector3f f(0, 0, 0);
  Vector3f n(0, 0, 0);
  for (unsigned i = chainSize; i > 0; --i) {
    Matrix4f tMat;
    if (i == chainSize) {
      tMat.setIdentity();
      f = extForces;
      n = extMoments;
    } else {
      tMat = linkTs[type][chainStart + i];
    }
    Matrix3f rotMat = tMat.block(0, 0, 3, 3);
    Vector3f transMat = tMat.block(0, 3, 3, 1);
    Vector3f comP = linkComs[chainStart + i - 1].segment(0, 3);
    n = comMoments[i-1] + rotMat * n + comP.cross(comForces[i-1]) + transMat.cross(rotMat * f);
    f = comForces[i-1] + rotMat * f;
    jointTorques[i-1] = n.transpose() * zAxis;
  }
  f = linkTs[type][chainStart].block(0, 0, 3, 3) * f;
  n = linkTs[type][chainStart].block(0, 0, 3, 3) * n;
  //jointTorques[0] = n.transpose() * zAxis;
  /*cout << "forces" << f(0,0) << endl;
  cout << "forces" << f(1,0) << endl;
  cout << "forces" << f(2,0) << endl;
  cout << "moments" << n(0,0) << endl;
  cout << "moments" << n(1,0) << endl;
  cout << "moments" << n(2,0) << endl;*/
  
  //! Relocating moments to torso origin frame
  n = n + Vector3f(initTs[chainIndex].block(0, 3, 3, 1)).cross(f);
  // f remains the same
      
  totalMoments = n;
  totalForces = f;
  return jointTorques;
}

Vector2f KinematicsModule::computeZmp(
  const unsigned& supportLeg, const JointUsageType& type)
{
  Vector3f extForces;
  Vector3f extMoments;
  Vector3f torsoForces;
  Vector3f torsoMoments;
  extForces.setZero();
  extMoments.setZero();
  torsoForces.setZero();
  torsoMoments.setZero();
  for (int i = 0; i < CHAINS_SIZE; ++i) {
    Vector3f chainForces;
    Vector3f chainMoments;
    VectorXf torque = 
      newtonEulerForces(
        i, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
    torsoForces += chainForces;
    torsoMoments += chainMoments;
  }
  
  //! Rotating torso forces and moments to inertial frame situated at 
  //! the base support leg
  Matrix4f supportT;
  supportT =
    MathsUtils::getTInverse(
      getForwardEffector(supportLeg, FEET_BASE, type)
    );
    
  Matrix3f supportR = supportT.block(0, 0, 3, 3);
  torsoForces = supportR * torsoForces;
  torsoMoments = supportR * torsoMoments;  
  
  //! Torso weight and vector
  Vector3f torsoCog, batteryCog;
  torsoCog = (supportT * linkComs[NUM_JOINTS + TORSO]).block<3, 1>(0, 0); // Torso center of mass
  //batteryCog = (supportT * Vector4f(batteryX, batteryY, batteryZ, 1.f)).block<3, 1>(0, 0); // Battery center of mass
  auto torsoWeight = Vector3f(0.f, 0.f, linkMasses[NUM_JOINTS + TORSO] * -gConst);
  //auto batteryWeight = Vector3f(0.f, 0.f, batteryMass * -gConst);
  
  //! Resulatant moments and forces at the base frame
  torsoMoments = 
    torsoMoments + 
    Vector3f(supportT.block(0, 3, 3, 1)).cross(torsoForces) +
    torsoCog.cross(torsoWeight);//
    //batteryCog.cross(batteryWeight);
  
  Vector3f rForce = -torsoForces - torsoWeight;// - batteryWeight;
  
  // zmp_y * R_z - zmp_z * R_y + M_x = 0
  // zmp_z * R_x - zmp_x * R_z + M_y = 0
  // zmp_x * R_y - zmp_y * R_x + M_z = 0
  
  Vector2f zmp;
  zmp[0] = torsoMoments[1] / rForce[2]; // M_y / R_z
  zmp[1] = - torsoMoments[0] / rForce[2]; // - M_x / R_z  
  
  Vector2f com;
  getComWrtBase(supportLeg, FEET_BASE, com, type);
  //cout << "Center of mass: " << endl;
  //cout << com << endl;
  
  //cout << "Zmp: " << endl;
  //cout << zmp << endl;
  return zmp;
}

Vector3f
KinematicsModule::calculateCenterOfMass(const JointUsageType& type)
{
  Vector3f com;
  Matrix4f T;
  com.setZero();
  unsigned chainStarts = 0;
  unsigned n = 0;
  for (int i = 0; i < chainSizes.size(); ++i) {
    T = initTs[i];
    for (int j = 0; j < chainSizes[i]; ++j) {
      T = T * linkTs[type][chainStarts + j];
      Vector3f temp = (T * linkComs[chainStarts + j]).block<3, 1>(0, 0);
      com = com + temp * linkMasses[chainStarts + j];
      ++n;
    }
    chainStarts = n;
  }
  com =
    com + linkComs[NUM_JOINTS + TORSO].block(0, 0, 3, 1) * linkMasses[NUM_JOINTS + TORSO];
  //com = com + Vector3f(batteryX, batteryY, batteryZ) * batteryMass;
  com = com / totalMassH25;
  return com;
}

void
KinematicsModule::updateFootOnGround()
{
  vector<float> fsrSensors = IVAR(vector<float>, MotionModule::fsrSensors);
  feetForcesBuffer.push_back(
    Vector2f(fsrSensors[L_FOOT_TOTAL_WEIGHT], fsrSensors[R_FOOT_TOTAL_WEIGHT]));
  if (feetForcesBuffer.size() >= ffBufferSize) {
    Vector2f bufferAvg = Vector2f::Zero();
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

Matrix4f
KinematicsModule::getFeetCenterT()
{
  Matrix4f T;
  Matrix4f ee;
  if (footOnGround == L_FOOT) {
    ee = endEffectors[CHAIN_L_LEG][FEET_BASE];
    MathsUtils::makeTranslation(T, (float) 0.0, (float) -0.05, (float) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_L_LEG, ee);
  } else if (footOnGround == R_FOOT) {
    ee = endEffectors[CHAIN_R_LEG][FEET_BASE];
    MathsUtils::makeTranslation(T, (float) 0.0, (float) 0.05, (float) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_R_LEG, ee);
  } else {
    ee = endEffectors[CHAIN_L_LEG][FEET_BASE];
    MathsUtils::makeTranslation(T, (float) 0.0, (float) -0.05, (float) 0.0);
    ee = ee * T;
    T = getForwardEffector(CHAIN_L_LEG, ee);
  }
  return T;
}

void
KinematicsModule::updateTorsoToFeet()
{
  OVAR(Matrix4f, MotionModule::lFootOnGround) = 
    getForwardEffector(CHAIN_L_LEG, FEET_BASE);
  OVAR(Matrix4f, MotionModule::rFootOnGround) = 
    getForwardEffector(CHAIN_R_LEG, FEET_BASE);
}

Vector4f
KinematicsModule::getWorldToCam(
  const unsigned& camIndex, const Vector4f& posInFoot)
{
  if (camIndex == TOP_CAM) {
    return OVAR(Matrix4f, MotionModule::upperCamInFeet) * posInFoot;
  } else {
    return OVAR(Matrix4f, MotionModule::lowerCamInFeet) * posInFoot;
  }
}

void
KinematicsModule::updateFootToCamT()
{
  Matrix4f torsoToFeet = getFeetCenterT();
  //cout << "Feet center: \n" << torsoToFeet << endl;
  Matrix4f torsoPitchRot;
  MathsUtils::makeRotationXYZ(
    torsoPitchRot,
    (float) 0.0,
    (float) -torsoPitchOffset * M_PI / 180,
    (float) 0.0);
  torsoToFeet = torsoPitchRot * torsoToFeet;
  //cout << "Feet center: \n" << torsoToFeet << endl;
  OVAR(Matrix4f, MotionModule::upperCamInFeet) =
    MathsUtils::getTInverse(
      getForwardEffector(CHAIN_HEAD, endEffectors[CHAIN_HEAD][TOP_CAM])) * torsoToFeet;
  OVAR(Matrix4f, MotionModule::lowerCamInFeet) = MathsUtils::getTInverse(
    getForwardEffector(
      CHAIN_HEAD,
      endEffectors[CHAIN_HEAD][BOTTOM_CAM])) * torsoToFeet;
  //cout << "Tmatrix: " << endl << OVAR(Matrix4f, MotionModule::upperCamInFeet) << endl << endl;
  //cout << "Tmatrix: " << endl << OVAR(Matrix4f, MotionModule::lowerCamInFeet) << endl << endl;
  //cout << "Tmatrix Inv: " << endl << MathsUtils::getTInverse(OVAR(Matrix4f, MotionModule::upperCamInFeet)) << endl << endl;
}

VectorXf
KinematicsModule::solveJacobianIK(
  const unsigned& chainIndex, const unsigned& eeIndex, const Matrix4f& targetT,
  const unsigned& maxIterations, const unsigned& startType)
{
  return solveJacobianIK(
    chainIndex,
    endEffectors[chainIndex][eeIndex],
    targetT,
    maxIterations,
    startType);
}

VectorXf
KinematicsModule::solveJacobianIK(
  const unsigned& chainIndex, const Matrix4f& endEffector,
  const Matrix4f& targetT, const unsigned& maxIterations, const unsigned& startType)
{
  float kO = 1.f;
  float kP = 1.f;
  int it = 0;
  JointUsageType type = VALID;
  jointPositions[type].segment(chainStarts[chainIndex], chainSizes[chainIndex]) =
    jointPositions[startType].segment(
      chainStarts[chainIndex],
      chainSizes[chainIndex]);
  prepareDHTransforms(chainIndex, type);
  Matrix4f initT = getForwardEffector(chainIndex, endEffector, type);
  Vector3f initTrans, targetTrans;
  Matrix<float, 6, 1> diff;
  initTrans = initT.block(0, 3, 3, 1);
  //init.block(3, 0, 3, 1) = MathsUtils::getEulerAngles(initT);
  targetTrans = targetT.block(0, 3, 3, 1);
  //target.block(3, 0, 3, 1) = MathsUtils::getEulerAngles(targetT);
  Matrix3f skew1t, skew2t, skew3t;
  skew1t = MathsUtils::makeSkewMat((Vector3f) targetT.block(0, 0, 3, 1));
  skew2t = MathsUtils::makeSkewMat((Vector3f) targetT.block(0, 1, 3, 1));
  skew3t = MathsUtils::makeSkewMat((Vector3f) targetT.block(0, 2, 3, 1));
  Matrix3f skew1i, skew2i, skew3i;
  skew1i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 0, 3, 1));
  skew2i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 1, 3, 1));
  skew3i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 2, 3, 1));
  Matrix3f L = -0.5 * (skew1t * skew1i + skew2t * skew2i + skew3t * skew3i);
  Vector3f orientError;
  orientError =
    0.5 * (skew1i * targetT.block(0, 0, 3, 1) + skew2i * targetT.block(
      0,
      1,
      3,
      1) + skew3i * targetT.block(0, 2, 3, 1));
  diff.block(0, 0, 3, 1) = kP * (targetTrans - initTrans);
  diff.block(3, 0, 3, 1) = kO * (L.inverse() * orientError);
  bool diffCheck = false;
  for (int i = 0; i < diff.rows(); ++i) {
    if (abs(diff[i]) > 1e-3) {
      diffCheck = true;
      break;
    }
    diffCheck = false;
  }
  while (diffCheck) {
    MatrixXf J = computeLimbJ(chainIndex, endEffector, type);
    MatrixXf jInv = MathsUtils::pseudoInverse(J);
    VectorXf result = jInv * diff;
    jointPositions[type].segment(
      chainStarts[chainIndex],
      chainSizes[chainIndex]) += result;
    VectorXf newJoints = jointPositions[type].segment(
      chainStarts[chainIndex],
      chainSizes[chainIndex]);
    for (int i = 0; i < newJoints.rows(); ++i) {
      if (abs(newJoints[i]) > M_PI) newJoints[i] = atan2(
        sin(newJoints[i]),
        cos(newJoints[i]));
      if (newJoints[i] > jointULimits[chainStarts[chainIndex] + i]) newJoints[i] =
        jointULimits[chainStarts[chainIndex] + i];
      if (newJoints[i] < jointLLimits[chainStarts[chainIndex] + i]) newJoints[i] =
        jointLLimits[chainStarts[chainIndex] + i];
    }
    jointPositions[type].segment(
      chainStarts[chainIndex],
      chainSizes[chainIndex]) = newJoints;
    prepareDHTransforms(chainIndex, type);
    initT = getForwardEffector(chainIndex, endEffector, type);
    initTrans = initT.block(0, 3, 3, 1);
    skew1i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 0, 3, 1));
    skew2i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 1, 3, 1));
    skew3i = MathsUtils::makeSkewMat((Vector3f) initT.block(0, 2, 3, 1));
    L = -0.5 * (skew1t * skew1i + skew2t * skew2i + skew3t * skew3i);
    orientError =
      0.5 * (skew1i * targetT.block(0, 0, 3, 1) + skew2i * targetT.block(
        0,
        1,
        3,
        1) + skew3i * targetT.block(0, 2, 3, 1));
    diff.block(0, 0, 3, 1) = kP * (targetTrans - initTrans);
    diff.block(3, 0, 3, 1) = kO * (L.inverse() * orientError);
    for (int i = 0; i < diff.rows(); ++i) {
      if (abs(diff[i]) > 1e-3) {
        diffCheck = true;
        break;
      }
      diffCheck = false;
    }
    it++;
    if (it > maxIterations) break;
  }
  VectorXf angles = jointPositions[type].segment(
    chainStarts[chainIndex],
    chainSizes[chainIndex]);
  return angles;
}

vector<VectorXf>
KinematicsModule::inverseLeftLeg(const Matrix4f& endEffector,
  const Matrix4f& targetT)
{
  vector<VectorXf> returnResult;
  Matrix4f tTempTheta5, t4I, t5I, t6I, tTemp, tTemp2;
  Matrix4f t = targetT;
  Matrix4f tInit = t;

  //!Move the start point to the hipyawpitch point
  Matrix4f base = tBaseLLegInv;
  base *= t;

  //!Move the end point to the anklePitch joint
  base *= tEndLLegInv;

  //!Rotate hipyawpitch joint
  Matrix4f rot = rotFixLLeg;
  rot *= base;

  //!Invert the table, because we need the
  //!chain from the ankle to the hip
  Matrix4f tStart = rot;
  rot = MathsUtils::getTInverse(rot);
  t = rot;

  //!Build the rotation table
  float side1 = thighLength;
  float side2 = tibiaLength;
  float distanceSqrd = pow(t.block(0, 3, 3, 1).norm(), 2);

  //!Calculate Theta 4
  float theta4 = M_PI - MathsUtils::safeAcos(
    (pow(side1, 2) + pow(side2, 2) - distanceSqrd) / (2 * side1 * side2));
  if (theta4 != theta4) {
    return returnResult;
  }
  float theta6 = atan(t(1, 3) / t(2, 3));
  //if(theta6 < lAnkleRollLow || theta6 > lAnkleRollHigh)
  //  return returnResult;
  if (theta6 < lAnkleRollLow) theta6 = lAnkleRollLow;
  else if (theta6 > lAnkleRollHigh) theta6 = lAnkleRollHigh;

  MathsUtils::makeDHTransformation(
    t6I,
    (float) 0.0,
    (float) -M_PI_2,
    (float) 0.0,
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
      (float) -thighLength,
      (float) 0.0,
      (float) 0.0,
      theta4);
    float up =
      tTempTheta5(1, 3) * (tibiaLength + thighLength * cos(theta4)) + thighLength * tTempTheta5(
        0,
        3) * sin(theta4);
    float down = pow(thighLength, 2) * pow(sin(theta4), 2) + pow(
      tibiaLength + thighLength * cos(theta4),
      2);
    float theta5 = asin(-up / down);
    float posOrNegPIt5 = (theta5 >= 0) ? M_PI : -M_PI;
    if (theta5 != theta5 && up / down < 0) theta5 = -M_PI_2;
    else if (theta5 != theta5) theta5 = M_PI_2;
    for (int i = 0; i < 2; i++) {
      if (i == 0 && (theta5 > lAnklePitchHigh || theta5 < lAnklePitchLow)) continue;
      else if (i == 1 && (posOrNegPIt5 - theta5 > lAnklePitchHigh || posOrNegPIt5 - theta5 < lAnklePitchLow)) continue;
      else if (i == 1) theta5 = posOrNegPIt5 - theta5;
      MathsUtils::makeDHTransformation(
        t5I,
        (float) -tibiaLength,
        (float) 0.0,
        (float) 0.0,
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
      float temptheta2 = MathsUtils::safeAcos(tTemp2(1, 2));
      float theta2;
      for (int l = 0; l < 2; l++) {
        if (l == 0 && (temptheta2 - M_PI_4 > lHipRollHigh || temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 1 && (-temptheta2 - M_PI_4 > lHipRollHigh || -temptheta2 - M_PI_4 < lHipRollLow)) continue;
        else if (l == 0) theta2 = temptheta2 - M_PI_4;
        else if (l == 1) theta2 = -temptheta2 - M_PI_4;
        float theta3 = asin(tTemp2(1, 1) / sin(theta2 + M_PI_4));
        float posOrNegPIt3 = (theta3 >= 0) ? M_PI : -M_PI;
        if (theta3 != theta3 && tTemp2(1, 1) / sin(theta2 + M_PI_4) < 0) theta3 =
          -M_PI_2;
        else if (theta3 != theta3) theta3 = M_PI_2;
        for (int k = 0; k < 2; k++) {
          if (k == 0 && (theta3 > lHipPitchHigh || theta3 < lHipPitchLow)) continue;
          else if (k == 1 && (posOrNegPIt3 - theta3 > lHipPitchHigh || posOrNegPIt3 - theta3 < lHipPitchLow)) continue;
          else if (k == 1) theta3 = posOrNegPIt3 - theta3;
          float temptheta1 = MathsUtils::safeAcos(
            tTemp2(0, 2) / sin(theta2 + M_PI_4));
          if (temptheta1 != temptheta1) temptheta1 = 0;
          for (int p = 0; p < 2; p++) {
            float theta1;

            if (p == 0 && (temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 1 && (-temptheta1 + M_PI_2 > lHipYawPitchHigh || -temptheta1 + M_PI_2 < lHipYawPitchLow)) continue;
            else if (p == 0) theta1 = temptheta1 + M_PI_2;
            else if (p == 1) theta1 = -temptheta1 + M_PI_2;

            //!Forward VALID step
            jointPositions[VALID][L_HIP_YAW_PITCH] = theta1;
            jointPositions[VALID][L_HIP_ROLL] = theta2;
            jointPositions[VALID][L_HIP_PITCH] = theta3;
            jointPositions[VALID][L_KNEE_PITCH] = theta4;
            jointPositions[VALID][L_ANKLE_PITCH] = theta5;
            jointPositions[VALID][L_ANKLE_ROLL] = theta6;
            prepareDHTransforms(CHAIN_L_LEG, VALID);
            Matrix4f test = getForwardEffector(CHAIN_L_LEG, endEffector, VALID);
            if (MathsUtils::almostEqual(test, tInit)) {
              VectorXf r(R_LEG_SIZE);
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

vector<VectorXf>
KinematicsModule::inverseRightLeg(const Matrix4f& endEffector,
  const Matrix4f& targetT)
{
  Matrix4f mirrored = MathsUtils::mirrorTransformation(targetT);
  vector<VectorXf> res = inverseLeftLeg(endEffector, mirrored);
  for (unsigned i = 0; i < res.size(); i++) {
    res[i][1] = -res[i][1]; //HIP_ROLL
    res[i][5] = -res[i][5]; //ANKLE_ROLL
  }
  return res;
}
