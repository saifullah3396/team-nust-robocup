/**
 * @file Utils/include/CubicSpline.h
 *
 * This file implements the class CubicSpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017  
 */

#include "Utils/include/CubicSpline.h"

template <typename Scalar>
CubicSpline<Scalar>::CubicSpline(
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Matrix<Scalar, Dynamic, 1>& knots, 
  const Scalar& stepSize, 
  const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds) :
  PolySpline<Scalar>(
    3, // Cubic
    dim, 
    controlPoints, 
    knots,
    stepSize,  
    boundaryConds)
{
}

template <typename Scalar>
CubicSpline<Scalar>::CubicSpline(
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Scalar& splineTime, 
  const Scalar& stepSize, 
  const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds) :
  PolySpline<Scalar>(
    3, // Cubic
    dim, 
    controlPoints, 
    splineTime,
    stepSize,  
    boundaryConds)
{
}

template <typename Scalar>
CubicSpline<Scalar>::CubicSpline(const string& filePath) : 
  PolySpline<Scalar>(filePath)
{
}

template <typename Scalar>
void CubicSpline<Scalar>::setup()
{
  //! this is defined for clamped spline need update.
  //cout << "setting up spline" << endl;
  this->coeffs.resize(4);
  this->nKnots = this->knots.size();
  this->controlPointsU = this->controlPoints.block(1, 0, this->nKnots, this->dim);
  this->controlPointsL = this->controlPoints.block(0, 0, this->nKnots, this->dim);
  this->controlPointsDiff = this->controlPointsU - this->controlPointsL;
  //cout << "this->controlPointsDiff" << this->controlPointsDiff << endl;
  this->A.resize(this->nKnots + 1, this->nKnots + 1);
  this->b.resize(this->nKnots + 1, this->dim);
  this->bAccels.resize(this->nKnots + 1, this->dim);
  this->bAccelsU.resize(this->nKnots, this->dim);
  this->bAccelsL.resize(this->nKnots, this->dim);
  this->A.setZero();
  this->b.setZero();
  this->bAccels.setZero();
  this->bAccelsU.setZero();
  this->bAccelsL.setZero();
  genParams();
  //plotSpline(100, 0.0);
}

template <typename Scalar> 
void CubicSpline<Scalar>::genParams()
{
  //! this is defined for clamped spline need update.
  try {
    this->knotsRep = this->knots.replicate(1, this->dim);
    //cout << "this->knots:" << this->knots << endl;
    Matrix<Scalar, Dynamic, Dynamic> diff = this->controlPointsDiff.cwiseQuotient(this->knotsRep);
    for (int i = 0; i < this->dim; ++i) {
      this->b.block(0, i, this->nKnots + 1, 1) << 
        6 * (diff(0, i) - this->boundaryConds(0, i)), 
        6 * (diff.block(1, i, this->nKnots - 1, 1) - diff.block(0, i, this->nKnots - 1, 1)), 
        6 * (this->boundaryConds(1, i) - diff(this->nKnots - 1, i));
    }
    genInvAMat();
    genCoeffs();
    //plotSpline(100, 0.0);
  } catch (exception &e) {
    cout << e.what() << endl;
  }
}

template <typename Scalar>
void CubicSpline<Scalar>::genInvAMat()
{
  //! this is defined for clamped spline need update.
  Matrix<Scalar, Dynamic, 1> upperLower, middle;
  upperLower.resize(this->nKnots);
  middle.resize(this->nKnots + 1);
  upperLower << this->knots;
  middle << 2 * this->knots(0), 2 * (this->knots.block(0, 0, this->nKnots - 1, 1) + this->knots.block(
    1,
    0,
    this->nKnots - 1,
    1)), 2 * this->knots(this->nKnots - 1);
  for (int i = 0; i < this->nKnots + 1; ++i) {
    for (int j = 0; j < this->nKnots + 1; ++j) {
      if (i == j) this->A(i, j) = middle(i);
      else if (i - j == -1) this->A(i, j) = upperLower(i);
      else if (i - j == 1) this->A(i, j) = upperLower(j);
    }
  }
  this->invA = this->A.inverse();
}

template <typename Scalar>
void CubicSpline<Scalar>::genCoeffs()
{
  //! this is defined for clamped spline need update.
  for (int i = 0; i < this->dim; ++i) {
    this->bAccels.block(0, i, this->nKnots + 1, 1) << this->invA * this->b.block(0, i, this->nKnots + 1, 1);
  }
  this->bAccels = this->invA * this->b;
  this->bAccelsL = this->bAccels.block(0, 0, this->nKnots, this->dim);
  this->bAccelsU = this->bAccels.block(1, 0, this->nKnots, this->dim);
  this->coeffs[0] = this->bAccelsL.cwiseQuotient(6 * this->knotsRep);
  this->coeffs[1] = this->bAccelsU.cwiseQuotient(6 * this->knotsRep);
  this->coeffs[2] = this->controlPointsU.cwiseQuotient(this->knotsRep) - this->bAccelsU.cwiseProduct(
    this->knotsRep / 6);
  this->coeffs[3] = this->controlPointsL.cwiseQuotient(this->knotsRep) - this->bAccelsL.cwiseProduct(
    this->knotsRep / 6);
}

template class CubicSpline<float>;
template class CubicSpline<double>;
