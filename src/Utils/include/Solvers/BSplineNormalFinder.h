/**
 * @file Utils/include/BSplineNormalFinder.h
 *
 * This file declares the class BSplineNormalFinder
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/BSpline.h"
#include "Utils/include/NLOptimizer.h"
#include "Utils/include/MathsUtils.h"

/**
 * @class BSplineNormalFinder
 * @brief A class to solve for the normal to a given bspline
 */
template <typename Scalar>
class BSplineNormalFinder : public NLOptimizer
{
public:
  /**
   * Constructor
   */
  BSplineNormalFinder(
    BSpline<Scalar>* bSplinePtr,
    const Matrix<Scalar, 3, 1>& normal,
    const Matrix<Scalar, 2, 1>&tBounds) : 
    bSplinePtr(bSplinePtr),
    normal(normal),
    tBounds(tBounds)
  {
    normalMag = normal.squaredNorm();
  }
  
  /**
   * Destructor
   */
  ~BSplineNormalFinder()
  {
  }

  /**
   * Optimizes the knots for minimum time under given constraints
   */
  virtual void optDef();

  Scalar getResParam() { return resT; }
  Matrix<Scalar, 3, 1> getResSplinePoint() { return resSplinePoint; }

protected:
  /**
   * Evaluates the minimum time objective function
   */
  double
  costFunction(const vector<double> &vars, vector<double> &grad, void *data);
  
  BSpline<Scalar>* bSplinePtr;
  Matrix<Scalar, 3, 1> normal;
  Matrix<Scalar, 2, 1> tBounds;
  Scalar normalMag;
  Scalar resT;
  Matrix<Scalar, Dynamic, 1> resSplinePoint;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
