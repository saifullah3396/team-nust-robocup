/**
 * @file Utils/include/BSpline.h
 *
 * This file implements the class BSpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/BSpline.h"
#include "Utils/include/Solvers/BSplineNormalFinder.h"
#include "Utils/include/PlotEnv.h"

using namespace GnuPlotEnv;

template <typename Scalar>
BSpline<Scalar>::BSpline(
  const unsigned& degree,
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Matrix<Scalar, Dynamic, 1>& knots, 
  const Scalar& stepSize) :
  Spline<Scalar>(
    degree,
    dim, 
    controlPoints, 
    knots,
    stepSize,  
    BASIS)
{
}

template <typename Scalar>
BSpline<Scalar>::BSpline(
  const unsigned& degree,
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Scalar& splineTime, 
  const Scalar& stepSize) :
  Spline<Scalar>(
    degree,
    dim, 
    controlPoints, 
    splineTime,
    stepSize,  
    BASIS)
{
}
template <typename Scalar>
BSpline<Scalar>::BSpline(const string& filePath) : 
  Spline<Scalar>(filePath)
{
  BSpline::splineFromXml(filePath);
  validateParameters();
  setup();
}
template <typename Scalar>
void BSpline<Scalar>::validateParameters()
{
  ASSERT(this->controlPoints.rows() == this->nKnots - order);
}
template <typename Scalar>
void BSpline<Scalar>::setup()
{
  //!Setting Defaults
  //!X-Y-Z Dimension: splineDim
  //!Order 4 for cubic B-Splines: order
  //!Default Knot Vector for Clamped End Conditions
  /*this->knots.push_back(0);
  this->knots.push_back(0);
  this->knots.push_back(0);
  this->knots.push_back(0);
  this->knots.push_back(1);
  this->knots.push_back(2);
  this->knots.push_back(3);
  this->knots.push_back(4);
  this->knots.push_back(4);
  this->knots.push_back(4);
  this->knots.push_back(4);*/
  
  order = this->degree + 1;
  minKnot = maxKnot = this->knots[0];

  for (unsigned i = 1; i < this->knots.size(); ++i) {
    minKnot = min(minKnot, this->knots[i]);
    maxKnot = max(maxKnot, this->knots[i]);
  }
  validateParameters();
  vector<vector<Scalar> > spline;
  vector<Scalar> time;
  evaluateSpline(spline, time);
  
  //cout << "Bspline:" << endl;
  //cout << "this->knots:\n" << this->knots << endl;
  //cout << "this->degree: " << this->degree << endl;
  //cout << "order: " << order << endl;
  //cout << "this->dim: " << this->dim << endl;
  //cout << "this->controlPoints: " << this->controlPoints << endl;
}

template <typename Scalar>
void BSpline<Scalar>::evaluateSpline(
    vector<vector<Scalar> >& spline,
    vector<Scalar>& splineTime,
    const unsigned& derivative = 2) 
{
  derivativeOrder.clear();
  for (size_t i = 0; i < derivative + 1; ++i)
    derivativeOrder.push_back(i);  
  nDerivatives = derivativeOrder.size();
  
  bSpline.resize(nDerivatives);
  for (size_t i = 0; i < nDerivatives; ++i) {
    bSpline[i].resize(maxKnot / this->stepSize + 1, this->dim);
  }

  splineTime.clear();
  Scalar timeStep = 0.f;
  vector<Matrix<Scalar, Dynamic, 1> > splineAtStep;
  while (timeStep <= maxKnot + 1e-6) {
    int index = round(timeStep / this->stepSize);
    //cout << "index: " << index << endl;
    //cout << "timeStep: " << timeStep << endl;
    generateSplineAtStep(splineAtStep, timeStep);
    for (unsigned i = 0; i < splineAtStep.size(); ++i) {//!Derivative orders
      bSpline[i].block(index, 0, 1, this->dim) = splineAtStep[i].transpose();
    }
    //cout << "splineAtStep[i]: " << bSpline[0] << endl;
    splineTime.push_back(timeStep);    
    timeStep += this->stepSize;
  }
}

template <typename Scalar>
void BSpline<Scalar>::setKnots(const Matrix<Scalar, Dynamic, 1>& knots) 
{
  this->knots = knots;
  minKnot = maxKnot = this->knots[0];

  for (unsigned i = 1; i < this->knots.size(); ++i) {
    minKnot = min(minKnot, this->knots[i]);
    maxKnot = max(maxKnot, this->knots[i]);
  }
  validateParameters();
}

template <typename Scalar>
bool BSpline<Scalar>::generateSplineAtStep(vector<Matrix<Scalar, Dynamic, 1>>& splineAtStep, const Scalar& step)
{
  unsigned nB = (order * (order + 1)) / 2;
  vector<Scalar> basisVector(nB);

  int knotLocation;
  knotLocation = locateKnot(step);
  if (knotLocation < 0) {
    cerr << "Out of range\n";
    return false;
  }
  bSplineTable(knotLocation, step, basisVector);
  
  splineAtStep.resize(derivativeOrder.size());
  for (size_t i = 0; i < nDerivatives; ++i) {
    splineAtStep[i].resize(this->dim);
  }
  
  for (size_t d = 0; d < this->dim; ++d) {
    auto res = evaluateSplineAtBasis(knotLocation, d, basisVector);
    for (size_t j = 0; j < derivativeOrder.size(); ++j) {
      splineAtStep[j][d] = res[j];
    }
  }
  //cout << "basisVector: " << endl;
  //for (unsigned i = 0; i < basisVector.size(); ++i) //!Dimensions X-Y-Z
  //  cout << basisVector[i] << "  ";
  //cout << endl;
  /*cout << knot;
   for (unsigned i = 0; i < nDerivatives; ++i) 
   cout << ' ' << resSpline[0][i];
   cout << '\n';*/
  return true;
}

template <typename Scalar>
int BSpline<Scalar>::locateKnot(Scalar step)
{
  if (MathsUtils::almostEqual(step, this->knots[this->nKnots-1], Scalar(1e-5)))
    step = this->knots[this->nKnots-1] - Scalar(1e-5);
  else if (MathsUtils::almostEqual(step, this->knots[0], Scalar(1e-5)))
    step = this->knots[0] + Scalar(1e-5);
  if (step < this->knots[0]) return -1;
  if (step >= this->knots[this->nKnots-1]) return -1;
  
  static int lo = 0, hi = 1;
  if (!(this->knots[lo] <= step && step < this->knots[hi])) {
    int k;
    lo = 0;
    hi = this->knots.size() - 1;
    while (hi - lo > 1) {
      k = (hi + lo) >> 1;
      if (this->knots[k] > step) hi = k;
      else lo = k;
    }
    assert(this->knots[lo] <= step && step < this->knots[hi]);
  }
  return lo;
}

template <typename Scalar>
void BSpline<Scalar>::bSplineTable(
  const int& knotLocation, const Scalar& step, vector<Scalar>& basisVector)
{
  unsigned basisLength = 1;
  Scalar *basis0 = new Scalar[order];
  Scalar *basis1 = new Scalar[order];
  Scalar M, t1, t2;

  basisVector[0] = basis0[0] = 1;
  for (int i = 1; i <= this->degree; ++i) {
    basis1[0] = 0;
    for (int j = 1; j <= i; ++j) {
      t1 = this->knots[knotLocation + j];
      t2 = this->knots[knotLocation + j - i];
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

template <typename Scalar>
vector<Scalar> BSpline<Scalar>::evaluateSplineAtBasis(
  const int& knotLocation, 
  const int& d, 
  vector<Scalar>& basisVector)
{
  unsigned i, maxDerivative = derivativeOrder[0], nDerivative =
    derivativeOrder.size();
  for (i = 1; i < nDerivative; ++i) {
    maxDerivative = max(maxDerivative, derivativeOrder[i]);
  }

  unsigned nA = 0;
  for (i = 0; i <= maxDerivative; ++i) {
    nA += order - i;
  }

  Scalar *A = new Scalar[nA];
  unsigned q, p, r, s, z;
  q = knotLocation - this->degree;

  for (i = 0; i < order; ++i) {
    A[i] = this->controlPoints(q + i, d);
  }

  s = 0;
  q = order;
  for (p = 1; p <= maxDerivative; ++p) {
    r = this->degree - p;
    z = knotLocation + r;
    for (i = knotLocation; i <= z; ++i, ++q, ++s)
      A[q] = (A[s + 1] - A[s]) / (this->knots[i + 1] - this->knots[i - r]);
    ++s;
  }

  unsigned oA = 0, oB = (order * (order - 1)) / 2;
  Scalar sum, f = 1;

  vector<Scalar> res(nDerivative);
  for (i = p = 0; i < nDerivative; ++i) {
    for (r = derivativeOrder[i]; p < r; ++p) {
      q = this->degree - p;
      f *= q;
      oA += q + 1;
      oB -= q;
    }

    assert(p == derivativeOrder[i]);
    // SUM = sum_{q = knotLocation - order + p}^knotLocation A_q^{(p)} N_{q, order - p}(x)
    sum = 0;
    for (q = 0, r = this->degree - p; q <= r; ++q)
      sum += A[oA + q] * basisVector[oB + q];
    // result: factor*SUM
    res[i] = f * sum;
  }
  delete[] A;
  return res;
}

template <typename Scalar>
bool BSpline<Scalar>::findNormalToVec(
  const Matrix<Scalar, 3, 1>& normal,
  Matrix<Scalar, 3, 1>& splinePoint,
  const Matrix<Scalar, 2, 1>& tBounds)
{
  if (this->dim != 3) {
    cout << "Normal can only be found to a 3D-spline." << endl;
    return false;
  }
  auto normalFinder = 
    BSplineNormalFinder<Scalar>(this, normal, tBounds);  
  normalFinder.optDef();
  if (normalFinder.getSuccess()) {
    splinePoint = normalFinder.getResSplinePoint();
    return true;
  } else return false;
}

template <typename Scalar>
void BSpline<Scalar>::plotSpline()
{
  PlotEnv<Scalar>::set_terminal_std("wxt");
  if (bSpline.empty()) {
    cout << "Evaluate the bspline before plotting it." << endl;
    return;
  }
  Matrix<Scalar, 2, 1> xRange;
  Matrix<Scalar, 2, 1> yRange;
  Matrix<Scalar, 2, 1> zRange;
  xRange[0] = -1.0;
  xRange[1] = 1.0;
  yRange[0] = -1.0;
  yRange[1] = 1.0;
  zRange[0] = -1.0;
  zRange[1] = 1.0;
  
  for (size_t i = 0; i < bSpline.size(); ++i) {
    auto pe = 
      PlotEnv<Scalar>(
        "B-Spline", "x-Axis", "y-Axis", "z-Axis", xRange, yRange, zRange);
    pe.plot3D(
      bSpline[i].block(0, 0, bSpline[i].rows(), 1), 
      bSpline[i].block(0, 1, bSpline[i].rows(), 1),
      bSpline[i].block(0, 2, bSpline[i].rows(), 1)
    );
    pe.showonscreen();
    //while(true);
  }
}

template class BSpline<float>;
template class BSpline<double>;
