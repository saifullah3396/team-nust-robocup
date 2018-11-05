/**
 * @file Utils/include/Filters/KalmanFilter.cpp
 *
 * This file implements the class KalmanFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#include "Utils/include/Filters/KalmanFilter.h"

template <typename Scalar>
KalmanFilter<Scalar>::KalmanFilter(
  const boost::shared_ptr<ProcessModel<Scalar> >& model,
  const unsigned& measSize) :
  model(model),
  measSize(measSize)
{
  this->stateSize = model->getStateSize();
  this->H.resize(measSize, stateSize);
  this->H.setZero();
  this->P.resize(stateSize, stateSize);
  this->P.setIdentity();
  this->P * 0.001;
}

template <typename Scalar>
KalmanFilter<Scalar>::  KalmanFilter(
  const boost::shared_ptr<ProcessModel<Scalar> >& model,
  const unsigned& measSize,
  const Matrix<Scalar, Dynamic, Dynamic>& H) :
  model(model),
  measSize(measSize),
  H(H)
{
  this->stateSize = model->getStateSize();
  ASSERT(this->H.rows() == measSize && this->H.cols() == stateSize);
  this->Ht = this->H.transpose();
  this->P.resize(stateSize, stateSize);
  this->P.setIdentity();
  R.resize(measSize, measSize);
  R.setZero();
}

template <typename Scalar>
KalmanFilter<Scalar>::~KalmanFilter()
{
}

template <typename Scalar>
void KalmanFilter<Scalar>::predict()
{
  model->update();
  P = model->computeErrorCov(P);
}

template <typename Scalar>
void KalmanFilter<Scalar>::correct(const Matrix<Scalar, Dynamic, 1>& meas)
{
  Matrix<Scalar, Dynamic, 1> state = model->getState();
  Matrix<Scalar, Dynamic, Dynamic> K = P * Ht * (R + H * P * Ht).inverse();
  model->setState(state + K * (meas - H * state));
  P = P - K * H * P;
  model->setUpdated(false);
}

template class KalmanFilter<float>;
template class KalmanFilter<double>;

