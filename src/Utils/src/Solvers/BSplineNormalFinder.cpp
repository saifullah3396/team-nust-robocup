/**
 * @file Utils/include/BSplineNormalFinder.cpp
 *
 * This file implements the class BSplineNormalFinder
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "Utils/include/Solvers/BSplineNormalFinder.h"

template <typename Scalar>
double BSplineNormalFinder<Scalar>::costFunction(
  const vector<double>& vars, 
  vector<double>& grad,
  void *data)
{
  double f = 0;
  vector<Matrix<Scalar, Dynamic, 1>> splineAtStep;
  //cout << "t param: " << vars[0] << endl;
  bSplinePtr->generateSplineAtStep(splineAtStep, vars[0]);
  //cout << "splineAtStep[0]: " << splineAtStep[0] << endl;
  Matrix<Scalar, Dynamic, 1> unit = splineAtStep[1] / splineAtStep[1].norm();
  double angle = acos(unit.dot(normal));
  //cout << "angle: " << angle * 180 / M_PI << endl;
  
  f = abs(angle * 180 / M_PI - 90);
  //cout << "f: " << f << endl;
  //cout << M_PI / 2 * 180 << endl;
  return f;
}

template <typename Scalar>
void BSplineNormalFinder<Scalar>::optDef()
{
  if (bSplinePtr->getDim() != 3) {
    cout << "This solver only works for bsplines in 3-dimensions." << endl;
    return;
  }
  cout << "finding End effector..." << endl;
  //! Set bspline to find first order derivative
  bSplinePtr->derivativeOrder.clear();
  for (size_t i = 0; i < 2; ++i) // 0 to  1
    bSplinePtr->derivativeOrder.push_back(i);  
  bSplinePtr->nDerivatives = bSplinePtr->derivativeOrder.size();
  //! Objective function to minimize the (angle - 90) between the given normal
  //! and the bspline curve.
  //! Hessian for this objective function is unknown.
  //! Gradient for this function is unknown.
  //! 1 variable; Curve t parameter
  nlopt::opt opt(nlopt::LN_COBYLA, 1);
  vector<double> lb(1), ub(1), var0, constraintTols;
  //These are not euler angles rather they are fixed angle rotations.
  cout << tBounds << endl;
  lb[0] = tBounds[0]; //! Lower bound for parameterized curve.
  ub[0] = tBounds[1]; //! Upper bound for parameterized curve.
  for (int i = 0; i < lb.size(); ++i)
    var0.push_back(tBounds[0]);

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(BSplineNormalFinder<Scalar>::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  }
  resT = var0[0];
  vector<Matrix<Scalar, Dynamic, 1>> splineAtStep;
  bSplinePtr->generateSplineAtStep(splineAtStep, resT);
  resSplinePoint = splineAtStep[0];
  success = true;
}

template class BSplineNormalFinder<float>;
template class BSplineNormalFinder<double>;
