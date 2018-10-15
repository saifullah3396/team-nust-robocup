/**
 * @file MotionModule/include/KinematicsModule/TaskIkSolver.cpp
 *
 * This file implements the class TaskIkSolver
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"

template <typename Scalar>
TaskIkSolver<Scalar>::TaskIkSolver(const KinematicsModulePtr& km) :
  km(km)
{
}

/*
template <typename Scalar>
void TaskIkSolver<Scalar>::getComTask()
{
  // Support foot and other foot transformations
  Matrix<Scalar, 4, 4> supportT, otherT;
  supportT = getForwardEffector(supportLeg, FEET_BASE, type);
  otherT = getForwardEffector(otherLeg, FEET_BASE, type);
  Matrix<Scalar, 3, 1> supportEE = supportT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 1> WotherEE = otherT.block(0, 3, 3, 1);
  Matrix<Scalar, 3, 3> rTorso = // torso rotation in supportFoot
    MathsUtils::getTInverse(supportT).block(0, 0, 3, 3);
  Matrix<Scalar, 6, 6> rxTorso; // torso rotation 6x6
  rxTorso << 
    rTorso, 
    Matrix<Scalar, 3, 3>::Zero(), 
    Matrix<Scalar, 3, 3>::Zero(), 
    rTorso;
  Matrix<Scalar, 6, 6> rxSupport;
  rxSupport << 
    Matrix<Scalar, 3, 3>::Identity(), 
    MathsUtils::makeSkewMat(rTorso * eeBase),
    Matrix<Scalar, 3, 3>::Zero(), 
    Matrix<Scalar, 3, 3>::Identity();
  //! Support leg jacobian matrix rotated to world frame
  supportJacobian = 
    rxTorso * computeLimbJ(supportLeg, FEET_BASE, type);
  supportComJacobian = rTorso * computeLimbComJ(supportLeg, type);
  //! Other leg jacobian matrix rotated to world frame
  otherJacobian = 
    rxTorso * computeLimbJ(otherLeg, FEET_BASE, type);  
  otherComJacobian = rTorso * computeLimbComJ(otherLeg, type);
  //! Center of mass in support leg frame
  Matrix<Scalar, 3, 1> comSupport;
    kM->getComWrtBase(supportLeg, FEET_BASE, comSupport, type);  
  Matrix<Scalar, 3, Dynamic> comJacobian;
  unsigned supportLegSize = linkChains[supportLeg]->size;
  comJacobian.resize(3, supportLegSize);
  comJacobian.block(0, 0, 3, supportLegSize) =
    -supportJacobian.block(0, 0, 3, supportLegSize) +
    MathsUtils::makeSkewMat(comSupport) *
    supportJacobian.block(3, 0, 3, supportLegSize) +
    supportComJacobian;
  //Add torso ortientation as a separate task later
  //comJacobian.block(3, 0, 3, supportLegSize) = 
  //  -supportJacobian.block(3, 0, 3, supportLegSize);
  Matrix<Scalar, 6, 6> rxOther;
  rxOther << 
    Matrix<Scalar, 3, 3>::Identity(), 
    MathsUtils::makeSkewMat(rTorso * otherEE),
    Matrix<Scalar, 3, 3>::Zero(), 
    Matrix<Scalar, 3, 3>::Identity();
  supportJacobianInOther = rxOther.inverse() * rxSupport * supportJacobian;
  otherJacobianInv = MathsUtils::pseudoInverse(otherJacobian);
  comJacobian.block(0, 0, 3, supportLegSize) =
    comJacobian.block(0, 0, 3, supportLegSize) +
    otherComJacobian * otherComJacobian * supportJacobianInOther;
  Matrix<Scalar, 3, 1> comOtherDiff = // If cartesian space
    otherComJacobian * otherJacobianInv * otherLegVelocities;  
  Matrix<Scalar, 3, 1> comOtherDiff = // If joint space
    otherComJacobian * otherLegVelocities;
  comVelDes = comVelDes - comOtherDiff;
}*/

template <typename Scalar>
Scalar TaskIkSolver<Scalar>::computeCost(const Scalar& dt)
{
  Scalar cost = 0;
  for (size_t i = 0; i < tasks.size(); ++i)
    cost += task->computeCost(dt);
  return cost;
}

template <typename Scalar>
Scalar TaskIkSolver<Scalar>::buildQPMatrices(const Scalar& dt)
{
  
}

def __build_qp_matrices(self, dt):
        n = self.nb_active_dofs
        q = self.robot.q[self.active_dofs]
        P = zeros((n, n))
        v = zeros(n)
        with self.__lock:
            for task in self.tasks.itervalues():
                J = task.jacobian()[:, self.active_dofs]
                r = task.residual(dt)
                P += task.weight * dot(J.T, J)
                v += task.weight * dot(-r.T, J)
        qd_max_doflim = self.doflim_gain * (self.q_max - q) / dt
        qd_min_doflim = self.doflim_gain * (self.q_min - q) / dt
        qd_max = minimum(self.qd_max, qd_max_doflim)
        qd_min = maximum(self.qd_min, qd_min_doflim)
return (P, v, qd_max, qd_min)

template <typename Scalar>
double TaskIkSolver<Scalar>::optimize()
{
  double f = 0;
  auto type = JointStateType::SIM;
  setStateFromTo(JointStateType::ACTUAL, JointStateType::SIM);
  
  // 1 / 2 x' P x + R'x
  // P = J'J and R = -KvJ
  // K is the proportional gain for velocity [0-1]
  // Here x is joint velocities
  double K = 0.5; // sqp matrices
  Matrix<Scalar, Dynamic, Dynamic> P = comJacobian.transpose() * comJacobian;
  Matrix<Scalar, Dynamic, 1> R = -K * comVelDes * J;
  // sqp constraint matrix Gx <= H
  Matrix<Scalar, Dynamic, Dynamic> G;
  Matrix<Scalar, Dynamic, 1> h;
  G.resize(supportLegSize * 4, supportLegSize)
  h.resize(supportLegSize * 4);
  G.setZero();
  int i = 0;
  //[ 1  0  0  0]
  //[-1  0  0  0]
  //[ 0  1  0  0]
  //[ 0 -1  0  0]
  //[ 0  0  1  0]
  //[ 0  0 -1  0]
  //[ 0  0  0  1]
  //[ 0  0  0 -1]
  for (size_t j = 0; j < G.cols; ++j) {
    G(i, j) = 1;
    G(i+1, j) = -1;
    G(i+2, j) = 1;
    G(i+3, j) = -1;
    i += 4;
  }
  double KLim = 0.5;
  auto chainStart = kM->getLinkChain(supportLeg)->start;
  auto chainSize = kM->getLinkChain(supportLeg)->size;
  for (size_t i = 0; i < chainSize; ++i) {
    auto joint = kM->getJoint(i + chainStart);
    h[i] = joint->maxVelocity;
    h[i+1] = -joint->maxVelocity;
    h[i+2] = KLim * (joint->maxPosition - joint->states[type]->position) / step;
    h[i+3] = KLim * (joint->minPosition - joint->states[type]->position) / step;
  }
  cout << "G: " << G << endl;
  cout << "H: " << h << endl;
  return f;
}
  
template class TaskIkSolver<MType>;
