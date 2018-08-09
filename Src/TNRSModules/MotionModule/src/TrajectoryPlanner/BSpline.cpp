/**
 * @file TrajectoryPlanner/BSpline.h
 *
 * This file implements the BSpline generator.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017  
 */

#include "MotionModule/include/TrajectoryPlanner/BSpline.h"

BSpline::BSpline() :
  splineDim(3), splineOrder(4)
{
  //!Setting Defaults
  //!X-Y-Z Dimension: splineDim
  //!Order 4 for cubic B-Splines: splineOrder

  //!Default Knot Vector for Clamped End Conditions
  knotVector.push_back(0);
  knotVector.push_back(0);
  knotVector.push_back(0);
  knotVector.push_back(0);
  knotVector.push_back(1);
  knotVector.push_back(2);
  knotVector.push_back(3);
  knotVector.push_back(4);
  knotVector.push_back(4);
  knotVector.push_back(4);
  knotVector.push_back(4);

  derivativeOrder.push_back(0);
  derivativeOrder.push_back(1);
  derivativeOrder.push_back(2);
  nDerivatives = derivativeOrder.size();

  //!Number of Control Points
  nControlPoints = knotVector.size() - splineOrder;

  controlPoints.resize(nControlPoints);
  for (unsigned i = 0; i < nControlPoints; ++i) {
    for (unsigned j = 0; j < splineDim; ++j) {
      controlPoints[i].resize(splineDim);
      controlPoints[i][j] = i + 1;
    }
  }
  minKnot = maxKnot = knotVector[0];

  for (unsigned i = 1; i < knotVector.size(); ++i) {
    minKnot = min(minKnot, knotVector[i]);
    maxKnot = max(maxKnot, knotVector[i]);
  }
  setupSpline();
}

BSpline::~BSpline()
{
}

void
BSpline::setControlPoint(VectorXd controlPoint, unsigned index)
{
  if (index < controlPoints.size()) if ((controlPoints[index]).size() == controlPoint.size()) this->controlPoints[index] =
    controlPoint;
  else cout << "Number of Control Points cannot be greater than nKnot" << endl;
}

void
BSpline::setKnotVector(vector<double> knotVector)
{
  this->knotVector = knotVector;
  nControlPoints = knotVector.size() - splineOrder;
  minKnot = maxKnot = knotVector[0];

  for (unsigned i = 1; i < knotVector.size(); ++i) {
    minKnot = min(minKnot, knotVector[i]);
    maxKnot = max(maxKnot, knotVector[i]);
  }
}

void
BSpline::setupSpline()
{
  bSpline.resize(nDerivatives);
  for (unsigned i = 0; i < splineDim; ++i) {
    bSpline[i].resize(splineDim);
  }
}

bool
BSpline::generateSpline(double step)
{
  assert(controlPoints.size() == knotVector.size() - splineOrder);

  unsigned nB = (splineOrder * (splineOrder + 1)) / 2;
  vector<double> basisVector(nB);

  int knotLocation;
  knotLocation = locateKnot(step);
  if (knotLocation < 0) {
    cerr << "Out of range\n";
    return false;
  }
  bSplineTable(knotLocation, step, basisVector);
  for (unsigned dim = 0; dim < splineDim; ++dim) //!Dimensions X-Y-Z
    evaluateSpline(knotLocation, dim, basisVector);

  /*cout << knot;
   for (unsigned i = 0; i < nDerivatives; ++i) 
   cout << ' ' << resSpline[0][i];
   cout << '\n';*/
  return true;
}

int
BSpline::locateKnot(const double &step)
{
  if (step < knotVector.front()) return -1;
  if (step >= knotVector.back()) return -1;
  static int lo = 0, hi = 1;

  if (!(knotVector[lo] <= step && step < knotVector[hi])) {
    int k;
    lo = 0;
    hi = knotVector.size() - 1;
    while (hi - lo > 1) {
      k = (hi + lo) >> 1;
      if (knotVector[k] > step) hi = k;
      else lo = k;
    }
    assert(knotVector[lo] <= step && step < knotVector[hi]);
  }
  return lo;
}

void
BSpline::bSplineTable(const int &knotLocation, const double &step,
  vector<double> &basisVector)
{
  int degree = splineOrder - 1; //! degree = k - 1
  unsigned basisLength = 1;
  double *basis0 = new double[splineOrder];
  double *basis1 = new double[splineOrder];
  double M, t1, t2;

  basisVector[0] = basis0[0] = 1;
  for (int i = 1; i <= degree; ++i) {
    basis1[0] = 0;
    for (int j = 1; j <= i; ++j) {
      t1 = knotVector[knotLocation + j];
      t2 = knotVector[knotLocation + j - i];
      M = basis0[j - 1] / (t1 - t2);
      basis1[j - 1] += (t1 - step) * M;
      basis1[j] = (step - t2) * M;
    }
    for (int j = 0; j <= i; ++j)
      basisVector[basisLength++] = basis0[j] = basis1[j];
  }

  delete[] basis0;
  delete[] basis1;
}

void
BSpline::evaluateSpline(const int &knotLocation, const int &dim,
  vector<double> &basisVector)
{
  assert(bSpline.size() >= derivativeOrder.size());
  unsigned i, maxDerivative = derivativeOrder[0], nDerivative =
    derivativeOrder.size();
  for (i = 1; i < nDerivative; ++i) {
    maxDerivative = max(maxDerivative, derivativeOrder[i]);
  }

  unsigned nA = 0;
  for (i = 0; i <= maxDerivative; ++i) {
    nA += splineOrder - i;
  }

  double *A = new double[nA];
  unsigned degree = splineOrder - 1, q, p, r, s, z;
  q = knotLocation - degree;

  for (i = 0; i < splineOrder; ++i) {
    A[i] = controlPoints[q + i][dim];
  }

  s = 0;
  q = splineOrder;
  for (p = 1; p <= maxDerivative; ++p) {
    r = degree - p;
    z = knotLocation + r;
    for (i = knotLocation; i <= z; ++i, ++q, ++s)
      A[q] = (A[s + 1] - A[s]) / (knotVector[i + 1] - knotVector[i - r]);
    ++s;
  }

  unsigned oA = 0, oB = (splineOrder * (splineOrder - 1)) / 2;
  double sum, f = 1;

  for (i = p = 0; i < nDerivative; ++i) {
    for (r = derivativeOrder[i]; p < r; ++p) {
      q = degree - p;
      f *= q;
      oA += q + 1;
      oB -= q;
    }

    assert(p == derivativeOrder[i]);
    // SUM = sum_{q = knotLocation - splineOrder + p}^knotLocation A_q^{(p)} N_{q, splineOrder - p}(x)
    sum = 0;
    for (q = 0, r = degree - p; q <= r; ++q)
      sum += A[oA + q] * basisVector[oB + q];
    // result: factor*SUM 
    bSpline[i][dim] = f * sum;
  }
  delete[] A;
}
