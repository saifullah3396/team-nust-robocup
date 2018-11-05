/**
 * @file MotionModule/TrajectoryPlanner/TrajectoryPlanner.h
 *
 * This file implements a class for creating smooth trajectories for the 
 * joints based on requried conditions.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Aug 2017  
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"

template <typename Scalar>
TrajectoryPlanner<Scalar>::TrajectoryPlanner(MotionModule* motionModule) :
  kM(motionModule->getKinematicsModule()), stepSize(0.005)
{
}

template <typename Scalar> 
bool TrajectoryPlanner<Scalar>::cartesianPlanner(
  vector<vector<Scalar> >& traj,
  const unsigned& chainIndex, 
  const Matrix<Scalar, 4, 4>& endEffector,
  const vector<Matrix<Scalar, 4, 4>>& cPoses, 
  const vector<Matrix<Scalar, Dynamic, 1>>& cBoundVels,
  const bool& solveInitPose, 
  const bool& timeOpt)
{
  /* unsigned nPoses = cPoses.size();
   if (nPoses < 2) 
   return false;
   unsigned chainSize = kM->getChainSize(chainIndex);
   unsigned chainStart = kM->getChainStart(chainIndex);
   bool success = true;
   Matrix<Scalar, Dynamic, Dynamic> jointBoundVels;
   Matrix<Scalar, Dynamic, Dynamic> jointPos;
   jointPos.resize(cPoses.size(), chainSize);
   jointBoundVels.resize(2, chainSize);
   jointPos.setZero();
   jointBoundVels.setZero();
   int startPose = 1;
   if (!solveInitPose) {
   jointPos.block(0, 0, 1, chainSize) = 
   kM->getChainPositions(KinematicsModule::ACTUAL, chainIndex).transpose();
   } else {
   startPose = 0;
   }
   switch (chainIndex) {
   case CHAIN_HEAD:
   cout << "Inverse kinematics for this chain is not defined yet." << endl;
   return false;
   break;
   case CHAIN_L_ARM:
   cout << "Inverse kinematics for this chain is not defined yet." << endl;
   return false;
   break;
   case CHAIN_R_ARM:
   cout << "Inverse kinematics for this chain is not defined yet." << endl;
   return false;
   break;
   case CHAIN_L_LEG:
   for (int i = startPose; i < cPoses.size(); ++i) {
   vector<Matrix<Scalar, Dynamic, 1>> angles = kM->inverseLeftLeg(endEffector, cPoses[i]);
   if (angles.size() != 0) {
   jointPos.block(i, 0, 1, chainSize) = angles[0].transpose();
   } else {
   cout << "The required cartesian pose " << i
   << " is out of the configuration space of given chain."
   << endl;
   return false;
   }
   }
   break;
   case CHAIN_R_LEG:
   for (int i = startPose; i < cPoses.size(); ++i) {
   vector<Matrix<Scalar, Dynamic, 1>> angles = kM->inverseRightLeg(endEffector, cPoses[i]);
   if (angles.size() != 0) {
   cout << "i: " << i << endl;
   cout << "angles1: " << angles[0][0] * 180 / M_PI << endl;
   cout << "angles2: " << angles[0][1] * 180 / M_PI << endl;
   cout << "angles3: " << angles[0][2] * 180 / M_PI << endl;
   cout << "angles4: " << angles[0][3] * 180 / M_PI << endl;
   cout << "angles5: " << angles[0][4] * 180 / M_PI << endl;
   cout << "angles6: " << angles[0][5] * 180 / M_PI << endl;
   jointPos.block(i, 0, 1, chainSize) = angles[0].transpose();
   } else {
   cout << "The required cartesian pose " << i
   << " is out of the configuration space of given chain."
   << endl;
   return false;
   }
   }
   break;
   }
   int j = 0;
   for (int i = 0; i < cBoundVels.size(); ++i) {
   Matrix<Scalar, Dynamic, 1> joints = jointPos.block(j, 0, 1, chainSize).transpose();
   kM->setJointPositions(KinematicsModule::SIM, chainStart, joints);
   Matrix<Scalar, Dynamic, Dynamic> jacobian = kM->computeLimbJ(KinematicsModule::SIM, chainIndex, endEffector).block(0, 0, 3, 6); // Using only linear velocity jacobian
   Matrix<Scalar, Dynamic, 1> jVels = MathsUtils::pseudoInverseSolve(jacobian, Matrix<Scalar, Dynamic, 1>(cBoundVels[i].block(0, 0, 3, 1)));
   //Matrix<Scalar, Dynamic, 1> jVels = kM->cartToJointVels(KinematicsModule::SIM, chainIndex, cBoundVels[i], endEffector);
   jointBoundVels.block(i, 0, 1, chainSize) = jVels.transpose();
   j = j + (nPoses - 1);
   }
   cout << "cBoundVelss." << endl << cBoundVels[1] << endl;
   cout << "jointBoundVels." << endl << jointBoundVels << endl;
   //FIXME: Only cubic splines are defined yet.
   Matrix<Scalar, Dynamic, 1> knots;
   knots.resize(nPoses - 1);
   for (int i = 0; i < knots.size(); ++i)
   knots[i] = 0.2;
   cout << "jointPos:" << jointPos << endl;
   CubicSpline cubicSpline(this, 
   chainSize,
   0,
   jointPos,
   knots,
   jointBoundVels);
   Matrix<Scalar, Dynamic, 1> maxVels;
   cout << "chainSize: " << chainSize << endl;
   maxVels = kM->getChainVelLimits(chainIndex);
   for (int i = 0; i < maxVels.size(); ++i)
   maxVels[i] -= 0.015;
   cout << "cMaxVels." << endl << maxVels << endl;
   if (timeOpt)
   cubicSpline.optimizeKnots(maxVels);
   vector<Scalar> trajTime;
   cubicSpline.getTrajectories(traj, trajTime, 0, stepSize);
   return success;*/
}

template <typename Scalar>
void TrajectoryPlanner<Scalar>::jointsPlanner(
  vector<vector<Scalar> >& traj,
  const unsigned& chainIndex, 
  const Matrix<Scalar, Dynamic, Dynamic>& jointPositions,
  const Matrix<Scalar, Dynamic, Dynamic>& jointBoundVels, 
  const Matrix<Scalar, Dynamic, 1>& knots)
{
  /*unsigned chainSize = kM->getChainSize(chainIndex);
   unsigned nPoses = jointPositions.rows();
   ASSERT(knots.size() == nPoses - 1);
   CubicSpline cubicSpline(
   chainSize,
   0,
   jointPositions,
   knots,
   jointBoundVels);
   vector<Scalar> trajTime;
   cubicSpline.getTrajectories(traj, trajTime, 0, stepSize);  */
  /*RowMatrix<Scalar, Dynamic, 1> maxVels;
   cout << "chainSize: " << chainSize << endl;
   maxVels.resize(chainSize);
   maxVels[0] = 7.45;
   maxVels[1] = 6.46;
   maxVels[2] = 7.45;
   maxVels[3] = 6.46;
   maxVels[4] = 7.45;*/
  // maxVels[5] = 4.1;
  //cubicSpline.optimizeKnots(maxVels);
  //vector<Scalar> trajTime;
  //cubicSpline.getTrajectories(traj, trajTime, 0, stepSize);
}

/*
 bool TrajectoryPlanner::step(Vector3f &splinePosition, Vector3f &splineVelocity, Vector3f &splineAcceleration)
 {
 if(trajStep > bSpline->getMaxKnot())
 return false;
 if(bSpline->generateSpline(trajStep)) {
 cout << "trajStep: " << trajStep << endl;
 for(unsigned i = 0; i < 3; ++i) {
 vector<VectorXd> spline = bSpline->getSpline();
 splinePosition[i] = spline[0][i];
 splineVelocity[i] = spline[1][i];
 splineAcceleration[i] = spline[2][i];
 }
 }
 else {
 }
 trajStep = trajStep + stepSize;	
 return true;
 }

 void TrajectoryPlanner::setTrajectoryTime(Scalar trajectoryTime)
 {
 this->trajectoryTime = trajectoryTime;
 }

 void TrajectoryPlanner::setTrajectoryKnots()
 {
 Scalar knotStep = trajectoryTime/4;
 vector<double> knotVector;
 knotVector.push_back(0);
 knotVector.push_back(0);
 knotVector.push_back(0);
 knotVector.push_back(0);
 knotVector.push_back(knotStep);
 knotVector.push_back(2*knotStep);
 knotVector.push_back(3*knotStep);
 knotVector.push_back(4*knotStep);
 knotVector.push_back(4*knotStep);
 knotVector.push_back(4*knotStep);
 knotVector.push_back(4*knotStep);
 bSpline->setKnotVector(knotVector);
 trajStep = bSpline->getMinKnot();
 }

 void TrajectoryPlanner::setReferencePoints(vector<Vector3f> trajectoryRefPoints)
 {
 this->trajectoryRefPoints = trajectoryRefPoints;
 }

 void TrajectoryPlanner::defineViaPoints(Vector3f initialVelocity, Vector3f finalVelocity)
 {
 bSpline->setupSpline();
 for(unsigned i = 0; i < trajectoryRefPoints.size() ; i++) {
 bSpline->setControlPoint(Maths::firstThree(trajectoryRefPoints[i]).cast<double> (), 3*i);
 }
 
 Vector3f controlPoint1 = Maths::firstThree(trajectoryRefPoints[0]) + (trajectoryTime/4)/(3)*(Maths::firstThree(initialVelocity));
 bSpline->setControlPoint(controlPoint1.cast<double> (), 1);
 Vector3f controlPoint2 = (0.5)*(controlPoint1  + Maths::firstThree(trajectoryRefPoints[1]));
 bSpline->setControlPoint(controlPoint2.cast<double> (), 2);
 Vector3f controlPoint5 = Maths::firstThree(trajectoryRefPoints[2]) - (trajectoryTime/4)/(3)*(Maths::firstThree(finalVelocity));
 bSpline->setControlPoint(controlPoint5.cast<double> (), 5);
 Vector3f controlPoint4 = (0.5)*(controlPoint5  + Maths::firstThree(trajectoryRefPoints[1]));
 bSpline->setControlPoint(controlPoint4.cast<double> (), 4);
 
 controlPointsLog.open("/home/sensei/team-nust-robocup-v3/ProcessingModule/MotionModule/KickModule/Logs/SplinePoints.txt", fstream::app|fstream::out);
 controlPointsLog << (trajectoryRefPoints[0])[0] << "    " << (trajectoryRefPoints[0])[1] << "    " << (trajectoryRefPoints[0])[2] << endl;
 controlPointsLog << controlPoint1[0] << "    " << controlPoint1[1] << "    " << controlPoint1[2]  << endl;
 controlPointsLog << controlPoint2[0] << "    " << controlPoint2[1] << "    " << controlPoint2[2]  << endl;
 controlPointsLog << (trajectoryRefPoints[1])[0] << "    " << (trajectoryRefPoints[1])[1] << "    " << (trajectoryRefPoints[1])[2] << endl;
 controlPointsLog << controlPoint4[0] << "    " << controlPoint4[1] << "    " << controlPoint4[2] << endl;
 controlPointsLog << controlPoint5[0] << "    " << controlPoint5[1] << "    " << controlPoint5[2] << endl;
 controlPointsLog << (trajectoryRefPoints[2])[0] << "    " << (trajectoryRefPoints[2])[1] << "    " << (trajectoryRefPoints[2])[2] << endl;
 cout << "here1" << endl;
 controlPointsLog.close();
 }
 */

template class TrajectoryPlanner<MType>;
