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
#define MAX_ZMP_Y 0.015

double
CbOptimizer::costFunction(
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

void
CbOptimizer::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* knots, double* grad, void* data)
{
  //PRINT("Computing ineqConstraints")
  //if (!grad.empty()) {
  // }
  auto nKnots = cb->getNKnots();
  MatrixXf times;
  times.resize(nKnots + 1, 1);
  times.setZero();
  for (int i = 1; i < times.size(); ++i) {
    times(i, 0) = times(i - 1, 0) + knots[i - 1];
  }
  MatrixXf timesRep = times.replicate(1, chainSize);
  MatrixXf timesU = timesRep.block(1, 0, nKnots, chainSize);
  MatrixXf timesL = timesRep.block(0, 0, nKnots, chainSize);
  VectorXf knotsEigen;
  knotsEigen.resize(nKnots);
  for (int i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots[i];
  cb->evaluate(knotsEigen);
  
  auto coeffs = cb->getCoeffs();
  auto knotsRep = cb->getRepKnots();
  auto bAccels = cb->getBAccels();
  auto bAccelsL = cb->getBAccelsL();
  auto bAccelsU = cb->getBAccelsU();  
  
  MatrixXf epVels =
    (-3 * coeffs[0].cwiseProduct(knotsRep.cwiseProduct(knotsRep)) + coeffs[2] - coeffs[3]).cwiseAbs() - velLimits.replicate(
      nKnots,
      1);
  Map<RowVectorXf> epConstraints(epVels.data(), epVels.size());
  MatrixXf inflexionCheck =
    (bAccelsL.array().cwiseProduct(bAccelsU.array()) < 0).matrix().cast<float>();
  MatrixXf midTs = timesL + knotsRep.cwiseProduct(
    bAccelsL.cwiseQuotient(bAccelsL - bAccelsU));
  MatrixXf tDiffU = timesU - midTs;
  MatrixXf tDiffL = midTs - timesL;
  MatrixXf midVels =
    -3 * coeffs[0].cwiseProduct(tDiffU.cwiseProduct(tDiffU)) + 3 * coeffs[1].cwiseProduct(
      tDiffL.cwiseProduct(tDiffL)) + coeffs[2] - coeffs[3];
  midVels = inflexionCheck.cwiseProduct(
    midVels.cwiseAbs() - velLimits.replicate(nKnots, 1));
  Map<RowVectorXf> midpConstraints(midVels.data(), midVels.size());
  RowVectorXf totalConstraints;
  totalConstraints.resize(nCons);
  //cout << "ConSize:" << totalConstraints.size() << endl;
  if (zmpCons || torqueCons) {
    auto dynCons = computeDynCons(times, coeffs);
    totalConstraints << epConstraints, midpConstraints, dynCons;
  } else {
    totalConstraints << epConstraints, midpConstraints;
  }
  
  //cout << "totalConstraints: " << endl;
  //cout << totalConstraints << endl;
  
  for (int i = 0; i < nCons; ++i) {
    result[i] = totalConstraints[i];
  }
}

RowVectorXf CbOptimizer::computeDynCons(
  const MatrixXf& times, 
  const vector<MatrixXf> coeffs)
{
  MatrixXf timesSeqU, timesSeqU2, timesSeqU3, timesSeqL, timesSeqL2, timesSeqL3;
  MatrixXf pos, vel, acc;
  pos.resize(consEvalSteps, chainSize);
  vel.resize(consEvalSteps, chainSize);
  acc.resize(consEvalSteps, chainSize);
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
   
  MatrixXf timesSeqRepU = timesSeqU.replicate(1, chainSize);
  MatrixXf timesSeqRepU2 = timesSeqU2.replicate(1, chainSize);
  MatrixXf timesSeqRepU3 = timesSeqU3.replicate(1, chainSize);
  MatrixXf timesSeqRepL = timesSeqL.replicate(1, chainSize);
  MatrixXf timesSeqRepL2 = timesSeqL2.replicate(1, chainSize);
  MatrixXf timesSeqRepL3 = timesSeqL3.replicate(1, chainSize);
  knotIndex = 0;
  int jIndex = 0;
  RowVectorXf cons;
  cons.resize(2 * consEvalSteps);
  //cout << "zmp: " << endl;
  for (int i = 0; i < consEvalSteps; ++i) {
    if (!matIndexForKnots.empty()) {
      if ( i == matIndexForKnots[jIndex] ) {
        ++knotIndex;
        ++jIndex;
      }
    }
    pos.block(i, 0, 1, chainSize) = 
    coeffs[0].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepU3.block(i, 0, 1, chainSize)) + 
    coeffs[1].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepL3.block(i, 0, 1, chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepL.block(i, 0, 1, chainSize)) + 
    coeffs[3].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepU.block(i, 0, 1, chainSize));
    vel.block(i, 0, 1, chainSize) = 
    -3 * coeffs[0].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepU2.block(i, 0, 1, chainSize)) + 
    3 * coeffs[1].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepL2.block(i, 0, 1, chainSize)) + 
    coeffs[2].block(knotIndex, 0, 1, chainSize) -
    coeffs[3].block(knotIndex, 0, 1, chainSize);
    acc.block(i, 0, 1, chainSize) = 
    6 * coeffs[0].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepU.block(i, 0, 1, chainSize)) - 
    6 * coeffs[1].block(knotIndex, 0, 1, chainSize).cwiseProduct(timesSeqRepL.block(i, 0, 1, chainSize));
    kM->setChainState(
      chainIndex,
      pos.block(i, 0, 1, chainSize).transpose(), 
      vel.block(i, 0, 1, chainSize).transpose(),
      acc.block(i, 0, 1, chainSize).transpose(),
      KinematicsModule::SIM
    );
    Vector2f zmp = kM->computeZmp(CHAIN_L_LEG, KinematicsModule::SIM);
    //cout << zmp << endl;
    cons[i*2] = abs(zmp[0]) - MAX_ZMP_X;
    cons[i*2+1] = abs(zmp[1]) - MAX_ZMP_Y;
  }
  return cons;
}

void
CbOptimizer::optDef()
{
  PRINT("Performing optimization")
  //! Set initial joint states of the robot
  kM->setSimToActual();
    
  //!Objective function to minimize is sum_0^n{knots}
  //!Hessian for this objective function is zero matrix.
  //!Gradient for this function is matrix with each element equal to one.
  auto nKnots = cb->getNKnots();
  auto cpDiff = cb->getCpDiff();
  nlopt::opt opt(nlopt::LN_COBYLA, nKnots);
  VectorXf lbEigen = (cpDiff.cwiseQuotient(
    velLimits.replicate(nKnots, 1))).cwiseAbs().rowwise().maxCoeff();
  vector<double> lb, knots0, constraintTols;
  for (int i = 0; i < nKnots; ++i) {
    lb.push_back(lbEigen[i]);
    knots0.push_back(lbEigen[i]);
  }
  // Vel upper and lower hence twice
  unsigned nCons = nKnots * velLimits.size() * 2; 
  if (zmpCons)
    nCons += consEvalSteps * 2; // X-Y ZMP
  if (torqueCons)
    nCons += consEvalSteps * chainSize;
  //cout << "nCons: " << nCons << endl;
  for (int i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }
  opt.add_inequality_mconstraint(
    CbOptimizer::ineqWrapper,
    this,
    constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_min_objective(CbOptimizer::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  nlopt::result result = opt.optimize(knots0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    for (int i = 0; i < nKnots; ++i) {
      knots0[i] = ceil(knots0[i] / stepSize) * stepSize;
    }
    cout << "Found minimum at the knots: \n";
    for (int i = 0; i < nKnots; ++i) {
      cout << knots0[i] << endl;
    }
    cout << "with f: " << minf << endl;
  }
  VectorXf knotsEigen;
  knotsEigen.resize(nKnots);
  for (int i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots0[i];
  cb->evaluate(knotsEigen);
  //cb->plotSpline(100, 0.0);
}

