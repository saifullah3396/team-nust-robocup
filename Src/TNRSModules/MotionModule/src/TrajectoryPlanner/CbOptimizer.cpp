/**
 * @file MotionModule/TrajectoryPlanner/CbOptimizer.h
 *
 * This file implements the class to CbOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/TrajectoryPlanner/CbOptimizer.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"

#define MAX_ZMP_X 0.03
#define MAX_ZMP_Y 0.01

template<typename Scalar>
double CbOptimizer<Scalar>::costFunction(
  const vector<double>& knots, 
  vector<double>& grad,
  void *data)
{
  //PRINT("Computing cost")
  auto nKnots = cb->getNKnots();
  if (!grad.empty()) {
    for (int i = 0; i < nKnots; ++i)
      grad[i] = 1.0;
  }
  double f = 0;
  for (int i = 0; i < nKnots; ++i)
    f = f + knots[i];
  return f;
}

template<typename Scalar>
void CbOptimizer<Scalar>::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* knots, double* grad, void* data)
{
  //PRINT("Computing ineqConstraints")
  //if (!grad.empty()) {
  // }
  auto nKnots = cb->getNKnots();
  Matrix<Scalar, Dynamic, Dynamic> times;
  times.resize(nKnots + 1, 1);
  times.setZero();
  for (int i = 1; i < times.size(); ++i) {
    times(i, 0) = times(i - 1, 0) + knots[i - 1];
  }
  Matrix<Scalar, Dynamic, Dynamic> timesRep = times.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesU = timesRep.block(1, 0, nKnots, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesL = timesRep.block(0, 0, nKnots, this->chainSize);
  Matrix<Scalar, Dynamic, 1> knotsEigen;
  knotsEigen.resize(nKnots);
  for (int i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots[i];
  cb->evaluateCoeffs(knotsEigen);
  
  auto coeffs = cb->getCoeffs();
  auto knotsRep = cb->getRepKnots();
  auto bAccels = cb->getBAccels();
  auto bAccelsL = cb->getBAccelsL();
  auto bAccelsU = cb->getBAccelsU();  
  
  Matrix<Scalar, Dynamic, Dynamic> epVels =
    (-3 * coeffs[0].cwiseProduct(knotsRep.cwiseProduct(knotsRep)) + coeffs[2] - coeffs[3]).cwiseAbs() - this->velLimits.replicate(
      nKnots,
      1);
  Map<Matrix<Scalar, 1, Dynamic> > epConstraints(epVels.data(), epVels.size());
  Matrix<Scalar, Dynamic, Dynamic> inflexionCheck =
    (bAccelsL.array().cwiseProduct(bAccelsU.array()) < 0).matrix().template cast<Scalar>();
  Matrix<Scalar, Dynamic, Dynamic> midTs = timesL + knotsRep.cwiseProduct(
    bAccelsL.cwiseQuotient(bAccelsL - bAccelsU));
  Matrix<Scalar, Dynamic, Dynamic> tDiffU = timesU - midTs;
  Matrix<Scalar, Dynamic, Dynamic> tDiffL = midTs - timesL;
  Matrix<Scalar, Dynamic, Dynamic> midVels =
    -3 * coeffs[0].cwiseProduct(tDiffU.cwiseProduct(tDiffU)) + 3 * coeffs[1].cwiseProduct(
      tDiffL.cwiseProduct(tDiffL)) + coeffs[2] - coeffs[3];
  midVels = inflexionCheck.cwiseProduct(
    midVels.cwiseAbs() - this->velLimits.replicate(nKnots, 1));
  Map<Matrix<Scalar, 1, Dynamic> > midpConstraints(midVels.data(), midVels.size());
  Matrix<Scalar, 1, Dynamic> totalConstraints;
  totalConstraints.resize(nCons);
  //cout << "ConSize:" << totalConstraints.size() << endl;
  
  if (zmpCons || torqueCons) {
    auto dynCons = computeDynCons(times, coeffs);
    totalConstraints << epConstraints, midpConstraints, dynCons;
  } else if (eeCons) {
    auto eeConstraints = computeEECons(times, coeffs);
    totalConstraints << epConstraints, midpConstraints, eeConstraints;
  } else {
    totalConstraints << epConstraints, midpConstraints;
  }
  
  //cout << "totalConstraints: " << endl;
  //cout << totalConstraints << endl;
  
  for (int i = 0; i < nCons; ++i) {
    result[i] = totalConstraints[i];
  }
}

template<typename Scalar>
Matrix<Scalar, 1, Dynamic> CbOptimizer<Scalar>::computeDynCons(
  const Matrix<Scalar, Dynamic, Dynamic>& times, 
  const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs)
{
  Matrix<Scalar, Dynamic, Dynamic> timesSeqU, timesSeqU2, timesSeqU3, timesSeqL, timesSeqL2, timesSeqL3;
  Matrix<Scalar, Dynamic, Dynamic> pos, vel, acc;
  pos.resize(consEvalSteps, this->chainSize);
  vel.resize(consEvalSteps, this->chainSize);
  acc.resize(consEvalSteps, this->chainSize);
  timesSeqU.resize(consEvalSteps, 1);
  timesSeqU2.resize(consEvalSteps, 1);
  timesSeqU3.resize(consEvalSteps, 1);
  timesSeqL.resize(consEvalSteps, 1);
  timesSeqL2.resize(consEvalSteps, 1);
  timesSeqL3.resize(consEvalSteps, 1);
    
  int knotIndex = 0;
  vector<int> matIndexForKnots;
  for (int i = 0; i < consEvalSteps; ++i) {
    float timesSeq = (times(times.rows()-1, 0) - times(0, 0)) * i / (consEvalSteps - 1);
    if (i < consEvalSteps - 1) {
      if (timesSeq > times(knotIndex + 1, 0)) {
        ++knotIndex;
        matIndexForKnots.push_back(i);
      }
    }
    timesSeqU(i, 0) = times(knotIndex + 1, 0) - timesSeq;
    timesSeqL(i, 0) = timesSeq - times(knotIndex, 0);
    timesSeqU2(i, 0) = timesSeqU(i, 0) * timesSeqU(i, 0);
    timesSeqU3(i, 0) = timesSeqU2(i, 0) * timesSeqU(i, 0);
    timesSeqL2(i, 0) = timesSeqL(i, 0) * timesSeqL(i, 0);
    timesSeqL3(i, 0) = timesSeqL2(i, 0) * timesSeqL(i, 0);
  }
   
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU = timesSeqU.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU2 = timesSeqU2.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU3 = timesSeqU3.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL = timesSeqL.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL2 = timesSeqL2.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL3 = timesSeqL3.replicate(1, this->chainSize);
  knotIndex = 0;
  int jIndex = 0;
  Matrix<Scalar, 1, Dynamic> cons;
  //cout << "zmp" << endl;
  cons.resize(2 * consEvalSteps);
  for (int i = 0; i < consEvalSteps; ++i) {
    if (!matIndexForKnots.empty()) {
      if ( i == matIndexForKnots[jIndex] ) {
        ++knotIndex;
        ++jIndex;
      }
    }
    pos.block(i, 0, 1, this->chainSize) = 
    coeffs[0].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU3.block(i, 0, 1, this->chainSize)) + 
    coeffs[1].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL3.block(i, 0, 1, this->chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL.block(i, 0, 1, this->chainSize)) + 
    coeffs[3].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU.block(i, 0, 1, this->chainSize));
    vel.block(i, 0, 1, this->chainSize) = 
    -3 * coeffs[0].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU2.block(i, 0, 1, this->chainSize)) + 
    3 * coeffs[1].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL2.block(i, 0, 1, this->chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, this->chainSize) -
    coeffs[3].block(knotIndex, 0, 1, this->chainSize);
    acc.block(i, 0, 1, this->chainSize) = 
    6 * coeffs[0].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU.block(i, 0, 1, this->chainSize)) - 
    6 * coeffs[1].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL.block(i, 0, 1, this->chainSize));
    this->kM->setChainState(
      this->chainIndex,
      pos.block(i, 0, 1, this->chainSize).transpose(), 
      vel.block(i, 0, 1, this->chainSize).transpose(),
      acc.block(i, 0, 1, this->chainSize).transpose(),
      JointStateType::SIM
    );
    Matrix<Scalar, 2, 1> zmp = this->kM->computeZmp(CHAIN_L_LEG, JointStateType::SIM);
    cons[i*2] = abs(zmp[0]) - MAX_ZMP_X;
    cons[i*2+1] = abs(zmp[1]) - MAX_ZMP_Y;
  }
  //cout << cons << endl;
  return cons;
}

template<typename Scalar>
Matrix<Scalar, 1, Dynamic> CbOptimizer<Scalar>::computeEECons(
  const Matrix<Scalar, Dynamic, Dynamic>& times, 
  const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs)
{
  Matrix<Scalar, Dynamic, Dynamic> timesSeqU, timesSeqU2, timesSeqU3, timesSeqL, timesSeqL2, timesSeqL3;
  Matrix<Scalar, Dynamic, Dynamic> pos, vel, acc;
  pos.resize(consEvalSteps, this->chainSize);
  vel.resize(consEvalSteps, this->chainSize);
  acc.resize(consEvalSteps, this->chainSize);
  timesSeqU.resize(consEvalSteps, 1);
  timesSeqU2.resize(consEvalSteps, 1);
  timesSeqU3.resize(consEvalSteps, 1);
  timesSeqL.resize(consEvalSteps, 1);
  timesSeqL2.resize(consEvalSteps, 1);
  timesSeqL3.resize(consEvalSteps, 1);
    
  int knotIndex = 0;
  vector<int> matIndexForKnots;
  for (int i = 0; i < consEvalSteps; ++i) {
    float timesSeq = (times(times.rows()-1, 0) - times(0, 0)) * i / (consEvalSteps - 1);
    if (i < consEvalSteps - 1) {
      if (timesSeq > times(knotIndex + 1, 0)) {
        ++knotIndex;
        matIndexForKnots.push_back(i);
      }
    }
    timesSeqU(i, 0) = times(knotIndex + 1, 0) - timesSeq;
    timesSeqL(i, 0) = timesSeq - times(knotIndex, 0);
    timesSeqU2(i, 0) = timesSeqU(i, 0) * timesSeqU(i, 0);
    timesSeqU3(i, 0) = timesSeqU2(i, 0) * timesSeqU(i, 0);
    timesSeqL2(i, 0) = timesSeqL(i, 0) * timesSeqL(i, 0);
    timesSeqL3(i, 0) = timesSeqL2(i, 0) * timesSeqL(i, 0);
  }
   
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU = timesSeqU.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU2 = timesSeqU2.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepU3 = timesSeqU3.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL = timesSeqL.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL2 = timesSeqL2.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesSeqRepL3 = timesSeqL3.replicate(1, this->chainSize);
  knotIndex = 0;
  int jIndex = 0;
  Matrix<Scalar, 1, Dynamic> cons;
  cons.resize(3 * consEvalSteps);
  //cout << "zmp: " << endl;
  for (int i = 0; i < consEvalSteps; ++i) {
    if (!matIndexForKnots.empty()) {
      if ( i == matIndexForKnots[jIndex] ) {
        ++knotIndex;
        ++jIndex;
      }
    }
    pos.block(i, 0, 1, this->chainSize) = 
    coeffs[0].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU3.block(i, 0, 1, this->chainSize)) + 
    coeffs[1].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL3.block(i, 0, 1, this->chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL.block(i, 0, 1, this->chainSize)) + 
    coeffs[3].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU.block(i, 0, 1, this->chainSize));
    vel.block(i, 0, 1, this->chainSize) = 
    -3 * coeffs[0].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepU2.block(i, 0, 1, this->chainSize)) + 
    3 * coeffs[1].block(knotIndex, 0, 1, this->chainSize).cwiseProduct(timesSeqRepL2.block(i, 0, 1, this->chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, this->chainSize) -
    coeffs[3].block(knotIndex, 0, 1, this->chainSize);
    this->kM->setChainPositions(
      this->chainIndex,
      pos.block(i, 0, 1, this->chainSize).transpose(),
      JointStateType::SIM
    );
    auto J = this->kM->computeLimbJ(this->chainIndex, endEffector, JointStateType::SIM);
    auto eeVel = J * vel.block(i, 0, 1, this->chainSize).transpose();
    //cons[i*3] = abs(eeVel(0,0) - eeVelMax[0]);
    //cons[i*3+1] = abs(eeVel(1,0) - eeVelMax[1]);
    //cons[i*3+2] = abs(eeVel(2,0) - eeVelMax[2]);
  }
  return cons;
}

template<typename Scalar>
void CbOptimizer<Scalar>::optDef()
{
  //PRINT("Performing optimization")
  //! Set initial joint states of the robot
  this->kM->setStateFromTo(JointStateType::ACTUAL, JointStateType::SIM);
    
  //!Objective function to minimize is sum_0^n{knots}
  //!Hessian for this objective function is zero matrix.
  //!Gradient for this function is matrix with each element equal to one.
  auto nKnots = cb->getNKnots();
  auto cpDiff = cb->getCpDiff();
  //cout << "nKnots; " << nKnots << endl;
  //cout << "cpDiff; " << cpDiff << endl;
  nlopt::opt opt(nlopt::LN_COBYLA, nKnots);
  Matrix<Scalar, Dynamic, 1> lbEigen = (cpDiff.cwiseQuotient(
    this->velLimits.replicate(nKnots, 1))).cwiseAbs().rowwise().maxCoeff();
  vector<double> lb, knots0, constraintTols;
  for (int i = 0; i < nKnots; ++i) {
    lb.push_back(lbEigen[i]);
    knots0.push_back(lbEigen[i]);
  }
  // Vel upper and lower hence twice
  unsigned nCons = nKnots * this->velLimits.size() * 2; 
  if (zmpCons)
    nCons += consEvalSteps * 2; // X-Y ZMP
  if (torqueCons)
    nCons += consEvalSteps * this->chainSize;
  if (eeCons) // end-effector cartesian velocity bounds
    nCons += eeVelMax.size();
    
  //cout << "nCons: " << nCons << endl;
  for (int i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }
  opt.add_inequality_mconstraint(
    CbOptimizer<Scalar>::ineqWrapper,
    this,
    constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_min_objective(CbOptimizer<Scalar>::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  nlopt::result result = opt.optimize(knots0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    for (int i = 0; i < nKnots; ++i) {
      knots0[i] = ceil(knots0[i] / this->stepSize) * this->stepSize;
    }
    //cout << "Found minimum at the knots: \n";
    //for (int i = 0; i < nKnots; ++i) {
    // cout << knots0[i] << endl;
    //}
    //cout << "with f: " << minf << endl;
  }
  Matrix<Scalar, Dynamic, 1> knotsEigen;
  knotsEigen.resize(nKnots);
  for (int i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots0[i];
  cb->evaluateCoeffs(knotsEigen);
  //cb->plotSpline(100, 0.0);
}

template class CbOptimizer<MType>;
