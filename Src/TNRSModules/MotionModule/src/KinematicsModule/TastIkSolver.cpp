/**
 * @file MotionModule/include/KinematicsModule/TaskIkSolver.cpp
 *
 * This file implements the class TaskIkSolver
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include <qpOASES.hpp>
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"

USING_NAMESPACE_QPOASES

template <typename Scalar>
TaskIkSolver<Scalar>::TaskIkSolver(
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const unsigned& maxIterations,
  const vector<bool>& activeJoints,
  const Scalar& dt,
  const Scalar& costThres,
  const Scalar& costVarThres) :
  maxIterations(maxIterations), 
  dt(dt),
  costThres(costThres),
  costVarThres(costVarThres),
  activeJoints(activeJoints),
  km(km), 
  nDof(km->getNJoints()),
  dofPLimitGain(0.5),
  type(JointStateType::SIM),
  initiated(false)
{
  assert(activeJoints.size() == nDof);
}

template <typename Scalar>
TaskIkSolver<Scalar>::~TaskIkSolver()
{
  delete qp;
  qp = NULL;
}

template <typename Scalar>
Scalar TaskIkSolver<Scalar>::computeCost()
{
  Scalar cost = 0;
  for (size_t i = 0; i < tasks.size(); ++i) {
    cost += tasks[i]->computeCost(dt);
  }
  return cost;
}

template <typename Scalar>
Eigen::Matrix<Scalar, Dynamic, 1> TaskIkSolver<Scalar>::step()
{
  ASSERT(P.rows == P.cols && P.rows = nDof);
  ASSERT(v.rows == nDof);
  P.setZero();
  v.setZero();
  for (size_t i = 0; i < tasks.size(); ++i) {
    Eigen::Matrix<Scalar, Dynamic, Dynamic> J = tasks[i]->getJacobian();
    //cout << "Task ; " << i << endl;
    //cout << "J: " << J.transpose() << endl;
    Eigen::Matrix<Scalar, Dynamic, Dynamic> res = tasks[i]->getGain() * tasks[i]->computeResidual(dt);
    //cout << "res:" << res << endl;
    //Eigen::Matrix<Scalar, Dynamic, Dynamic> jInv = MathsUtils::pseudoInverse(J);
    //Eigen::Matrix<Scalar, Dynamic, 1> qd = jInv * res;
    //cout << "qd: " << jInv * res << endl;
    //cout << "res: " << res << endl;
    //return qd;
    //cout << "res: " << res << endl;
    //cout << "tasks[i]->getWeight(): " << tasks[i]->getWeight() << endl;
    //cout << "tasks[i]->getGain: " << tasks[i]->getGain() << endl;
    P += tasks[i]->getWeight() * J.transpose() * J;
    v += tasks[i]->getWeight() * - res.transpose() * J;
    //cout << "P: " << P << endl;
    //cout << "v: " << v << endl;
  } 
  //cout << "P: " << P << endl;
  //cout << "v: " << v << endl;
  //! Set up velocity constraints matrix by choosing the minimum from
  //! maximum joint velocity and the maximum distance the joint can move
  //! due to its kinematic constraint
  
  for (size_t i = 0; i < nDof; ++i) {
    auto joint = km->getJoint(i);
    auto maxDiffV =
      dofPLimitGain * (joint->maxPosition - joint->states[(unsigned)type]->position) / dt;
    auto minDiffV =
      dofPLimitGain * (joint->minPosition - joint->states[(unsigned)type]->position) / dt;
    maxV[i] = maxDiffV; //min(joint->maxVelocity, maxDiffV);
    minV[i] = minDiffV; //max(-joint->maxVelocity, minDiffV);
  }
  auto Z = Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor>::Zero(nDof, nDof);
  auto E = Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor>::Identity(nDof, nDof);
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> P0, G0;
  Eigen::Matrix<Scalar, Dynamic, 1> v0;
  Eigen::Matrix<Scalar, Dynamic, 1> h0;
  P0.resize(nDof * 2, nDof * 2);
  v0.resize(nDof * 2);
  h0.resize(nDof * 3);
  G0.resize(nDof * 3, nDof * 2);
  P0.setZero();
  v0.setZero();
  G0.setZero();
  P0.block(0, 0, nDof, nDof) = P;
  P0.block(nDof, nDof, nDof, nDof) = 1e-5 * E;
  v0.segment(0, nDof) = v;
  v0.segment(nDof, nDof) = -1e-3 * Eigen::Matrix<Scalar, Dynamic, 1>::Ones(nDof);
  //cout << "P0:/n" << P0 << endl;
  //cout << "v0:/n" << v0 << endl;

  G0 << E, E / dt, -E, E / dt, Z, -E;
  h0 << maxV, -minV, Eigen::Matrix<Scalar, Dynamic, 1>::Zero(nDof);
  h << maxV, -minV;
  //cout << "P0:/n" << P0 << endl;
  //cout << "v0:/n" << v0 << endl;
  //cout << "G0:/n" << G0 << endl;
  //cout << "h0:/n" << h0 << endl;

  real_t* qpP = P.data();
  real_t* qpV = v.data();
  real_t* qpG = G.data();
  real_t* qpH = h.data();

  int_t nWSR = 1000;
  qp->setPrintLevel(PrintLevel::PL_LOW);
  if (!initiated) {
    qp->init(qpP, qpV, qpG, NULL, NULL, NULL, qpH, nWSR, 0);
    initiated = true;
  } else {
    qp->hotstart(qpP, qpV, qpG, NULL, NULL, NULL, qpH, nWSR, 0);
  }
  real_t qpQd[nDof];
  qp->getPrimalSolution(qpQd);

  Eigen::Matrix<Scalar, Dynamic, 1> qd =
    Eigen::Map<Eigen::Matrix<Scalar, Dynamic, 1> >(qpQd, nDof, 1);
  //cout << "qd: " << qd.transpose() << endl;
  //cout << "objVel: " << qp->getObjVal() << endl;
  //cout << "J1 * qd - k * v1: " << tasks[0]->getJacobian() * qd - tasks[0]->getGain() * tasks[0]->computeResidual(dt)<< endl;
  //cout << "J2 * qd - k * v2: " << tasks[1]->getJacobian() * qd - tasks[1]->getGain() * tasks[1]->computeResidual(dt)<< endl;
  return qd;
}

template <typename Scalar>
Eigen::Matrix<Scalar, Dynamic, 1> TaskIkSolver<Scalar>::solve()
{
  km->setStateFromTo(JointStateType::ACTUAL, JointStateType::SIM);
  Scalar cost = 1e6, prevCost;
  qp = new SQProblem(nDof, nDof * 2);
  qp->reset();
  Options myOptions;
  myOptions.setToMPC( );
  myOptions.printLevel = PL_LOW;
  qp->setOptions(myOptions);

  //! Get maximum velocity of each joint and set up velocity constraints
  //! matrices
  maxV.resize(nDof);
  minV.resize(nDof);
  for (size_t i = 0; i < nDof; ++i) {
    auto joint = km->getJoint(i);
    maxV[i] = joint->maxVelocity;
    minV[i] = -joint->maxVelocity;
  }
  P.resize(nDof, nDof);
  v.resize(nDof);
  G.resize(nDof * 2, nDof);
  h.resize(nDof * 2);
  G << 
    Eigen::Matrix<Scalar, Dynamic, Dynamic>::Identity(nDof, nDof),
    -Eigen::Matrix<Scalar, Dynamic, Dynamic>::Identity(nDof, nDof);
  //cout << "Solving sqp ik problem..." << endl;
  Eigen::Matrix<Scalar, Dynamic, 1> joints = km->getJointPositions(0, nDof, JointStateType::SIM);
  for (size_t i = 0; i < maxIterations; ++i) {
    //auto tStart = high_resolution_clock::now();
    prevCost = cost;
    cost = computeCost();
    //cout << "Cost: " << cost << endl;
    Scalar costVar = abs(cost - prevCost) / prevCost;
    if (abs(cost) < costThres || costVar < costVarThres)
      break;
    Eigen::Matrix<Scalar, Dynamic, 1> jointStep = step() * dt;
    for (size_t j = 0; j < activeJoints.size(); ++j) {
      if (activeJoints[j]) {
        //cout << "j:" << j << endl;
        //cout << "joint: " << joints[j] * 180 / M_PI << endl;
        //cout << "jointStep: " << jointStep[j] * 180 / M_PI << endl;
        joints[j] += jointStep[j];
        //cout << "new joint: " << joints[j] * 180 / M_PI << endl;
      }
    }
    //cout << "joints: " << joints.transpose() * 180 / M_PI << endl;
    km->setJointPositions(0, joints, type);
    //duration<double> timeSpan = high_resolution_clock::now() - tStart;
    //PRINT("Iter Time: " << timeSpan.count() << "seconds.");
  }
  cout << "joints: " << joints.transpose() * 180 / M_PI << endl;
  return joints;
}
  
template class TaskIkSolver<MType>;
