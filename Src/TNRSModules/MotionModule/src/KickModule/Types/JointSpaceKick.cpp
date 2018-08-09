/**
 * @file MotionModule/include/KickModule/JointSpaceKick.cpp
 *
 * This file implements the class JointSpaceKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"

JSKickConfigPtr JointSpaceKick::getBehaviorCast()
{
  return boost::static_pointer_cast <JSKickConfig> (config);
}

void
JointSpaceKick::setupBalance()
{
  PRINT("JointSpaceKick.setupBalance()")
  //auto bConfig = // Set from kick config
  //  boost::make_shared<MPComControlConfig>(supportLeg, 1.f);
  if (getBehaviorCast()->balanceConfig) {
    getBehaviorCast()->balanceConfig->supportLeg = supportLeg;
    setupChildRequest(getBehaviorCast()->balanceConfig);
  } else {
    PRINT("JointSpaceKick.setupBalance(): No balance config found.")
  }
}

void
JointSpaceKick::setDesBallVel()
{
	desBallVel = sqrt(targetDistance / (0.026 / rf + 0.02496 / sf));
}
  
void
JointSpaceKick::computeDesImpactVel(const VectorXf& impJoints)
{
  float coeffRest = 1.f; // Full elastic collision
  setDesBallVel();
	Vector3f direction = 
		torsoToSupport.block(0, 0, 3, 3) * ballToTargetUnit;
	// impact pose joints
  kM->setChainPositions(kickLeg, impJoints, KinematicsModule::SIM);
  float vm;
  kM->computeVirtualMass(
    kickLeg,
    direction,
    endEffector,
    vm,
    KinematicsModule::SIM);
   cout << "Virtual mass in given direction and found pose: " << vm << endl;
   float desEndEffVel = desBallVel * (vm + ballMass) / ((1 + coeffRest) * vm);
   cout << "desEndEffVel: " << desEndEffVel << endl;
   desImpactVel[0] = desEndEffVel * ballToTargetUnit[0];
   desImpactVel[1] = desEndEffVel * ballToTargetUnit[1];
   desImpactVel[2] = 0.f;
}

void
JointSpaceKick::defineTrajectory()
{
  //cout << "supportToKick:\n" << supportToKick << endl;
  //cout << "endEffector:\n" << endEffector << endl;
  auto eeTrans = supportToKick * endEffector;
  //cout << "endEffector trans:\n" << eeTrans << endl;
  //! Define via points for final trajectory.
  //cout << "ballRadius: " << ballRadius << endl;
  const float balldx = ballRadius * ballToTargetUnit[0];
  const float balldy = ballRadius * ballToTargetUnit[1];
  Matrix4f postImpactPose3, postImpactPose2, postImpactPose1;
  postImpactPose3.setIdentity();
  postImpactPose3.block(0, 3, 3, 1) = eeTrans.block(0, 3, 3, 1); // Same orientation as left foot
  postImpactPose2 = postImpactPose3;
  postImpactPose2(2, 3) = 0.0; // The height of the ankle
  postImpactPose1.setIdentity();
  postImpactPose1(0, 3) = ballPosition[0];
  postImpactPose1(1, 3) = ballPosition[1];
  postImpactPose1(2, 3) = ballPosition[2];
  //! Define via points for constant velocity phase.
  impactPose.setIdentity();
  impactPose(0, 3) = ballPosition[0] - balldx;
  impactPose(1, 3) = ballPosition[1] - balldy;
  impactPose(2, 3) = ballPosition[2];
  //! Define via points for preImpactPose phase.
  Matrix4f preImpactPose2, preImpactPose1;
  preImpactPose2.setIdentity();
  preImpactPose2(0, 3) = (impactPose(0, 3) + eeTrans(0, 3)) / 2;
  preImpactPose2(1, 3) = (impactPose(1, 3) + eeTrans(1, 3)) / 2;
  preImpactPose2(2, 3) = impactPose(2, 3);
  preImpactPose2(0, 3) =
    preImpactPose2(0, 3) - ballToTargetUnit[0] * ballRadius;
  preImpactPose2(1, 3) =
    preImpactPose2(1, 3) - ballToTargetUnit[1] * ballRadius;
  preImpactPose1 = eeTrans;
  //cout << "EndEffector: " << endEffector << endl;
  //cout << "ImpactPose: " << impactPose << endl;
  //findBestEEAndImpactPose();
  //cout << "ImpactPose: " << impactPose << endl;
  unsigned chainSize = kM->getChainSize(kickLeg);
  unsigned chainStart = kM->getChainStart(kickLeg);
  cPoses.clear();
  vector<Matrix4f> cPosesT;
  cPoses.push_back(preImpactPose1);
  cPoses.push_back(preImpactPose2);
  cPoses.push_back(impactPose);
  cPoses.push_back(postImpactPose1);
  cPoses.push_back(postImpactPose2);
  cPoses.push_back(postImpactPose3);
  for (int i = 0; i < cPoses.size(); ++i) {
    cout << "cPose[" << i << "]:\n" << cPoses[i] << endl;
    cPosesT.push_back(torsoToSupport * cPoses[i]);
    //cout << "cPosesT[" << i << "]:\n" << cPosesT[i] << endl;
  }

  MatrixXf jointPos;
  jointPos.resize(cPosesT.size(), chainSize);
  jointPos.setZero();
  for (int i = 0; i < cPosesT.size(); ++i) {
    vector < VectorXf > angles;
    if (kickLeg == CHAIN_L_LEG) angles = kM->inverseLeftLeg(
      endEffector,
      cPosesT[i]);
    else angles = kM->inverseRightLeg(endEffector, cPosesT[i]);
    if (angles.size() != 0) {
      jointPos.row(i) = angles[0].transpose();
    } else {
      ERROR(
        "The required cartesian pose " << i << " is out of the configuration space of given chain.")
      return;
    }
  }

  //! Setting up pre-impact trajectory.
  //! Required cartesian velocities at initial and final poses
  vector<VectorXf> cBoundVels(2);
  for (int i = 0; i < cBoundVels.size(); ++i) {
    cBoundVels[i].resize(6); //x-y-z, r-p-y
    cBoundVels[i].setZero();
  }
  if (!desImpactVelKnown) {
		computeDesImpactVel(jointPos.row(2));  
  }
  cBoundVels[1].block(0, 0, 3, 1) = desImpactVel; // x-y-z velocity
  //! Cartesian velocities in torso frame.
  for (int i = 0; i < cBoundVels.size(); ++i)
    cBoundVels[i].block(0, 0, 3, 1) =
      torsoToSupport.block(0, 0, 3, 3) * cBoundVels[i].segment(0, 3);
  //! Find last impact velocity in joints space using hitPose.
  VectorXf impactJVel;
  kM->setJointPositions(chainStart, jointPos.row(2), KinematicsModule::SIM); // impact pose joints
  MatrixXf jacobian = kM->computeLimbJ(
    kickLeg,
    endEffector,
    KinematicsModule::SIM).block(0, 0, 3, 6);
  impactJVel = MathsUtils::pseudoInverseSolve(
    jacobian,
    VectorXf(cBoundVels[1].block(0, 0, 3, 1)));

  MatrixXf jointBoundVels;
  jointBoundVels.resize(2, chainSize);
  jointBoundVels.setZero();
  jointBoundVels.row(1) = impactJVel.transpose(); // Second row

  PRINT("Performing first trajectory optimization...")
  //! Second trajectory optimization
  VectorXf knots;
  knots.resize(2); // 3 poses for pre-impact/post-impact trajectory
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.2;
  auto cb1 = CubicSpline(
    cycleTime,
    chainSize,
    0,
    jointPos.block(0, 0, 3, chainSize),
    knots,
    jointBoundVels);
  auto cbopt = CbOptimizer(motionModule, kickLeg, supportLeg, &cb1);
  cbopt.setZmpCons(true);
  cbopt.optDef();
  vector<float> trajTime;
  cb1.getTrajectories(jointTrajectories, trajTime, 0);

  //! Second trajectory optimization
  PRINT("Performing second trajectory optimization...")
  vector < vector<float> > jointTrajectories2;
  jointBoundVels.setZero();
  jointBoundVels.row(0) = impactJVel.transpose(); // First row
  knots.resize(3); // 4 poses for post-impact trajectory
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.2;
  auto cb2 = CubicSpline(
    cycleTime,
    chainSize,
    0,
    jointPos.block(2, 0, 4, chainSize),
    knots,
    jointBoundVels);
  cbopt.setCB(&cb2);
  cbopt.optDef();
  cb2.getTrajectories(jointTrajectories2, trajTime, 0);

  for (int i = 0; i < jointTrajectories.size(); ++i) {
    jointTrajectories[i].insert(
      jointTrajectories[i].end(),
      jointTrajectories2[i].begin(),
      jointTrajectories2[i].end());
  }
}

void
JointSpaceKick::requestExecution()
{
  unsigned chainStart = kM->getChainStart(kickLeg);
  unsigned chainSize = kM->getChainSize(kickLeg);
  vector<unsigned> jointIds;
  for (size_t i = chainStart; i < chainStart + chainSize; ++i)
    jointIds.push_back(i);
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(chainSize);
  jointPositions.arraySetSize(chainSize);
  unsigned trajStep = 0;
  while (true) {
    for (int i = 0; i < chainSize; ++i) {
      jointPositions[i].arrayPush(jointTrajectories[i][trajStep]);
      jointTimes[i].arrayPush((trajStep + 1) * cycleTime);
    }
    ++trajStep;
    if (trajStep == jointTrajectories[0].size()) break;
  }

  cout << "jointPositions: " << jointPositions << endl;
  //cout << "jointTimes: " << jointTimes << endl;
  totalTimeToKick = (trajStep + 1) * cycleTime;
  //cout << "totaltime : " << totalTimeToKick << endl;
  kickTime = 0.f;
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  //plotJointTrajectories();
}

void
JointSpaceKick::plotKick()
{
  PRINT("Plotting kick parameters...")
  setupGnuPlotConfig();
  cout << "ballPosition: " << ballPosition << endl;
  float centerSpacing = fabsf(supportToKick(1, 3) / 2);
  float offset = supportLeg == CHAIN_L_LEG ? centerSpacing : -centerSpacing;
  cout << "offset: " << offset << endl;
  cout << "ballPosition-offset: " << ballPosition[1] - offset << endl;
  auto eeTrans = supportToKick * endEffector;
  for (int i = 0; i < cPoses.size(); ++i)
    plotPoint(-(cPoses[i](1, 3) + offset), cPoses[i](0, 3), "", 1);

  gp << "set object 1 circle at " << -(ballPosition[1] + offset) << "," << ballPosition[0] << " size 0.05 fs transparent solid 0.35 fc rgb 'red'\n";
  string contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveLeftXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";
  contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveRightXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";

  if (supportLeg == CHAIN_R_LEG) {
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing - 0.025 << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  } else {
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -(centerSpacing + 0.025) << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  }
  gp << "replot\n";
  cin.get();
}

void
JointSpaceKick::plotJointTrajectories()
{
  Gnuplot gp;
  vector < pair<float, float> > times_pos;
  gp << "set xrange [0:20]\nset yrange [0:20]\n";
  gp << "plot" << gp.file1d(times_pos) << "with lines title 'Joint Trajectories'" << endl;
  int chainSize = kM->getChainSize(kickLeg);
  for (int i = 1; i < chainSize; ++i) {
    int trajStep = 0;
    while (true) {
      gp << "set terminal wxt " << i << endl;
      float pos = jointTrajectories[i][trajStep];
      float t = (trajStep + 1) * cycleTime;
      times_pos.push_back(make_pair(t, pos));
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }
    gp << "plot" << gp.file1d(times_pos) << "with lines title 'joint Trajectories " << i << " position.'" << endl;
    cin.get();
  }
}
