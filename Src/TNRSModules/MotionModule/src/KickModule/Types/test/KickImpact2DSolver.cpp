/**
 * @file MotionModule/include/KickModule/Types/KickImpact2DSolver.cpp
 *
 * This file implements the class to KickImpact2DSolver
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/KickModule/Types/KickImpact2DSolver.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"

template<typename Scalar>
KickImpact2DSolver::KickImpact2DSolver(JSE2DImpKick* jse2DImpKick) :
  kickPtr(jse2DImpKick),
  kM(jse2DImpKick->getKinematicsModule())
{
}

template<typename Scalar>
double KickImpact2DSolver::costFunction(const vector<double>& vars, vector<double>& grad,
  void *data)
{
  //cout << "Finding f.. " << endl;
  double f = 0;
  // angle of contact gives us the end-effector contact point
  auto contactNormal = Vector3f(cos(vars[3]), sin(vars[3]), 0.f);
  kickPtr->setEndEffectorXY(vars[3]);
  //cout << "End effectorXY\n";
  //cout << kickPtr->endEffector << endl;
  // Set end-effector position based on bezier parameter t.
  //kickPtr->setEndEffectorZX(vars[2]);
  //cout << "end effectorZX set" << endl;
  Matrix4f tX, tY;
  Matrix4f newImpactPose;// = kickPtr->impactPose;
  newImpactPose.setIdentity();
  newImpactPose(0, 3) = kickPtr->ballPosition[0] - contactNormal[0] * kickPtr->ballRadius;
  newImpactPose(1, 3) = kickPtr->ballPosition[1] - contactNormal[1] * kickPtr->ballRadius;
  newImpactPose(2, 3) = kickPtr->ballPosition[2];
  
  MathsUtils::makeRotationX(tX, vars[0]);
  MathsUtils::makeRotationY(tY, vars[1]);
  newImpactPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  newImpactPose = kickPtr->torsoToSupport * newImpactPose;
  //cout << "newImpactPose\n" << newImpactPose << endl;
  
  vector < VectorXf > angles;
  if (kickPtr->kickLeg == CHAIN_L_LEG) 
    angles = kM->inverseLeftLeg(kickPtr->endEffector, newImpactPose);
  else 
    angles = kM->inverseRightLeg(kickPtr->endEffector, newImpactPose);
  //cout << "angles found " << angles.size() << endl;
  if (angles.empty())
    return 1e6;
  float vm;
  Vector3f direction = 
		kickPtr->torsoToSupport.block(0, 0, 3, 3) * contactNormal;
  kM->setChainPositions(kickPtr->kickLeg, angles[0], KinematicsModule::SIM);
  kM->computeVirtualMass(
    kickPtr->kickLeg,
    direction,
    kickPtr->endEffector,
    vm,
    KinematicsModule::SIM);
  //cout << "virtual mass; " << vm << endl;  
  auto ee = kickPtr->endEffector;
  // Should be outer most end-effector point
  f = 1 / sqrt(ee(0, 3) * ee(0, 3) + ee(1, 3) * ee(1, 3));
  //cout << "f: " << endl;
  float cR = 1.f; // fully elastic collision
  float phi, m1, m2, v1, v2, theta1, theta2;
  phi = vars[3];
  m1 = kickPtr->ballMass;
  m2 = vm;
  v1 = iBallVel[0]; // initial ball velocity
  theta1 = iBallVel[1]; // initial ball angle
  v2 = vars[4];
  theta2 = vars[5];
  
  /*cout << "v1: " << v1 << endl;
  cout << "v2: " << v2 << endl;
  cout << "m1: " << m1 << endl;
  cout << "m2: " << m2 << endl;
  cout << "theta1: " << theta1 * 180.f/M_PI << endl;
  cout << "theta2: " << theta2 * 180.f/M_PI << endl;
  cout << "phi: " << phi * 180.f/M_PI << endl;*/
  
  float v1xr = v1 * cos(theta1 - phi);
  float v2xr = v2 * cos(theta2 - phi);
  float v1yr = v1 * sin(theta1 - phi);
  //float v2yr = v2 * sin(theta2 - phi);
  float sumM = (m1 + m2);
  float cp = cos(phi), sp = sin(phi);
	
  //cout << "v1xr: " << v1xr << endl;
  //cout << "v2xr: " << v2xr << endl;	
  //cout << "v1yr: " << v1yr << endl;
	
  // Carrying out momentum calculations in 2D
  // Final ball velocity rotated in 1D
  float v1fxr = ((cR + 1) * m2 * v2xr + (-cR * m2 + m1) * v1xr) / sumM;
  // v1fyr = v1yr;
	//v2fxr = ((cR + 1) * m1 * v1xr + (cR * m1 - m2) * v2xr) / sumM;

	//cout << "v1fxr: " << v1fxr << endl;

  // Angular momentum remains conserved as well
  double R = kickPtr->ballRadius;
  // Ball's angular velocity in direction of motion after impact
  // Note that here omegafxr = omegaxr = v1xr / R; that is the angular
  // velocity before the impact remains the same after impact
  double omegafxr = v1xr / R;
  double omegafyr = v1yr / R;
  //cout << "omegafxr: " << omegafxr << endl;
  //cout << "omegafyr: " << omegafyr << endl;
  
  // Finding distance based on velocities after impact
  // Motion under kinetic friction or slip
  double accelConst = - 2.0 * kickPtr->sf * gConst;
  double v1fxrAtRolling = 2.0 / 7.0 * R * omegafxr + 5.0 / 7.0 * v1fxr;
  double dBeforeRolling = (v1fxrAtRolling * v1fxrAtRolling - v1fxr * v1fxr) / accelConst;
  double dAfterRolling = v1fxrAtRolling / kickPtr->coeffDamping;
  double distancexr = dBeforeRolling + dAfterRolling;
  
  // Starts with angular velocity omegafyr and ends at 2/7 * omegafyr * R as it slips with friction
  double v1fyrAtRolling = 2.0 / 7.0 * R * omegafxr;
  double distanceyr = (v1fyrAtRolling * v1fyrAtRolling - v1yr * v1yr) / accelConst;

  //cout << "distancexr rotated: " << distancexr << endl;
  //cout << "distanceyr rotated: " << distanceyr << endl;

  double distancex = distancexr * cp + distanceyr * sp;
  double distancey = distancexr * sp + distanceyr * cp;

  //cout << "distancexr: " << distancexr << endl;
  //cout << "distanceyr: " << distanceyr << endl;

	// Final ball velocity in 2D
	/*Vector2f fBallVel;
	fBallVel[0] = v1fxr * cp + v1yr * sp;
	fBallVel[1] = v1fxr * sp + v1yr * cp;
  
  Vector2f fBallAngVel;
  fBallAngVel[0] = omegafxr * cp + omegafyr * sp;
	fBallAngVel[1] = omegafxr * sp + omegafyr * cp;*/
  
  //cout << "fBallVel: " << fBallVel << endl;
  //cout << "fBallVelDes: " << fBallVelDes << endl;
	// Minimize the difference between computed ball velocity and 
	// the desired ball velocity
	//f = abs(fBallVel[0] - fBallVelDes[0]) + abs(fBallVel[1] - fBallVelDes[1]);
  f = abs(distancex - 1.0) + abs(distancey - 0);
  //cout << "f: " << f << endl;
  return f;
}

template<typename Scalar>
void KickImpact2DSolver::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* vars, double* grad, void* data)
{
  //cout << "Solving Inequality Constraints..." << endl;
  /*VectorXf fVel;
  float vel = vars[3];
  fVel.resize(3);
  fVel.setZero();
  fVel[0] = vel * kickPtr->ballToTargetUnit[0];
  fVel[1] = vel * kickPtr->ballToTargetUnit[1];
  fVel[2] = 0.0;
  fVel.block(0, 0, 3, 1) = kickPtr->torsoToSupport.block(0, 0, 3, 3) * fVel.segment(0, 3);
  MatrixXf jacobian = kM->computeLimbJ(
    kickPtr->kickLeg,
    kickPtr->endEffector,
    KinematicsModule::SIM).block(0, 0, 3, 6);
  VectorXf jVels = MathsUtils::pseudoInverseSolve(jacobian, fVel);
  VectorXf velCons = jVels.cwiseAbs() - kM->getChainVelLimits(kickPtr->kickLeg);
  //cout << jVels << endl;
  //cout << kM->getChainVelLimits(kickPtr->kickLeg) << endl;
  for (int i = 0; i < nCons; ++i) {
    result[i] = velCons[i];
  }
  //cout << "Finished solving Inequality Constraints..." << endl;*/
}

/*double
KickImpact2DSolver::getDesBallVel(const double& phi)
{
  //! Solving ball distance equation which is made up of two parts;
  //! Distance covered by the ball under static friction
  //! 2as1 = vf1^2 - vi1^2
  //!   where vf1 = 2/7*R*wi1 + 5/7*vi1
  //! => s1 = (vf1^2 - vi1^2) / (-2*mu_s*g)
  //! The distance covered after rolling under damped motion
  //! s2 = vf1 / damping;
  //!   since vi2 = vf1;
  //! The resulting equation can be found by s = s1 + s2;
  //! After the impact the angular velocity remains the same in the direction of its 
  //! initial motion
  double& R = ballRadius;
  //! Find the angular velocity in the direction of final motion of ball
  //! This is necessary since initial and final angles can differ and therefore
  //! component of angular velocity in final direction of motion is taken
  double omega = iBallVel[0] / R * cos(iBallVel[1] - phi);
  double a = 12 / (49 * sf * gConst);
  double b = 5.0 / (7 * coeffDamping) - 10 * ballRadius * omega / (49 * sf * gConst);
  double c =  
    -targetDistance 
    - 2 * R * R * omega * omega / (49 * sf * gConst);
    + 2 * R * omega / (7 * coeffDamping);
  double sol1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  if (sol1 < 0) {
    double sol2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    desBallVel = sol2;
  } else {
    desBallVel = sol1;
  }
  
}*/

template<typename Scalar>
void KickImpact2DSolver::optDef()
{
	//fBallVelDes[0] = kickPtr->desBallVel * kickPtr->ballToTargetUnit[0];
	//fBallVelDes[1] = kickPtr->desBallVel * kickPtr->ballToTargetUnit[1];
	
	iBallVel[0] = kickPtr->ballVelocity.norm();
	iBallVel[1] = atan2(kickPtr->ballVelocity[1], kickPtr->ballVelocity[0]);
	
	//float contactAngle; // phi opt var 1
	
  //! Objective function to minimize is virtualMass x velocity;
  //! Hessian for this objective function is unknown.
  //! Gradient for this function is unknown.
  //! 6 variables; 
  //! Euler Angle about X, 
  //! Euler Angle about Y, 
  //! end-effector contour parameter t
  //! Angle of contact with ball
  //! End effector initial velocity
  //! End effector initial velocity angle
  unsigned numVars = 6;
  nlopt::opt opt(nlopt::LN_COBYLA, numVars);
  vector<double> lb(numVars), ub(numVars), var0(numVars), constraintTols;
  //These are not euler angles rather they are fixed angle rotations.
  lb[0] = -5.0 * M_PI / 180.0; //! Lower bound for x-angle. 
  lb[1] = -10.0 * M_PI / 180.0; //! Lower bound for y-angle. 
  lb[2] = 0.0; //! Lower bound for parameterized curve [0...1].
  lb[3] = kickPtr->targetAngle - 15 * M_PI / 180; // -M_PI / 2;//! Lower bound for the angle of impact with the ball. -90 degrees
  lb[4] = -0.5; //! Lower bound for the velocity magnitude of the end effector.
  if (kickPtr->kickLeg == CHAIN_R_LEG)
		lb[5] = -10.f * M_PI / 180.f; //! Upper bound for the end effector angle of velocity. 10 degrees
	else
		lb[5] = -M_PI / 2; //! Upper bound for the end effector angle of velocity. -90 degrees
  
  ub[0] = 5.0 * M_PI / 180.0; //! Upper bound for x-angle. 
  ub[1] = 10.0 * M_PI / 180.0; //! Upper bound for y-angle. 
  ub[2] = 1.0; //! Upper bound for parameterized curve [0...1].
  ub[3] = kickPtr->targetAngle + 15 * M_PI / 180; // M_PI / 2;//! Upper bound for the angle of impact with the ball. 90 degrees
  ub[4] = 1.2f; //! Upper bound for the velocity magnitude of the end effector.
  if (kickPtr->kickLeg == CHAIN_R_LEG)
		ub[5] = M_PI / 2; //! Upper bound for the end effector angle of velocity. -90 degrees
	else
		ub[5] = 10.f * M_PI / 180.f; //! Upper bound for the end effector angle of velocity. 10 degrees
		
  var0[0] = 0.f;
  var0[1] = 0.f;
  var0[2] = 0.f;
  var0[3] = kickPtr->targetAngle;
  var0[4] = 0.5f;
  var0[5] = kickPtr->targetAngle;

  //! Joint velocity contraint for leg joints;
  unsigned nCons = kM->getChainSize(kickPtr->kickLeg);
  for (int i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }

  //opt.add_inequality_mconstraint(KickImpact2DSolver::ineqWrapper, this, constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(KickImpact2DSolver::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  //cout << "Starting optimization... " << endl;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  }// else {
    /*cout << "Found minimum at the variables:\n";
    cout << "Euler Angle about X: " << var0[0] * 180.f/ M_PI << endl;
		cout << "Euler Angle about Y: " << var0[1] * 180.f/ M_PI << endl;
		cout << "End-effector ZX Param: " << var0[2] << endl;
		cout << "Contact Angle " << var0[3] * 180.f/ M_PI << endl;
		cout << "Velocity" << var0[4] << endl;
		cout << "Velocity angle" << var0[5] * 180.f/M_PI  << endl;
		cout << "with f: " << minf << endl;*/
  //}
  cout << "end effector Velocity" << var0[4] << endl;

  kickPtr->setEndEffectorZX(var0[2]);
  Matrix4f tX, tY;
  MathsUtils::makeRotationX(tX, var0[0]);
  MathsUtils::makeRotationY(tY, var0[1]);
  kickPtr->impactPose.setIdentity();
  kickPtr->impactPose(0, 3) = kickPtr->ballPosition[0] - cos(var0[3]) * kickPtr->ballRadius;
  kickPtr->impactPose(1, 3) = kickPtr->ballPosition[1] - sin(var0[3]) * kickPtr->ballRadius;
  kickPtr->impactPose(2, 3) = kickPtr->ballPosition[2];
  kickPtr->impactPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  kickPtr->desImpactVel[0] = var0[4] * cos(var0[5]);
  kickPtr->desImpactVel[1] = var0[4] * sin(var0[5]);
  kickPtr->desImpactVel[2] = 0.f;
  
  cout << "desImpactVel1 : " << kickPtr->desImpactVel << endl;
  kickPtr->desImpactVelKnown = true;
  //cout << "Optimized endEffector pose:\n " << kickPtr->endEffector << endl;
  //cout << "Max Velocity in Given Direction:\n " << var0[3] << endl;
  //cout << "Best EndEffector Position: " << kickPtr->endEffector << endl;
}

