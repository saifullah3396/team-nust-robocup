/**
 * @file Utils/include/Filters/KalmanFilter.h
 *
 * This file declares the class KalmanFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once

#include "Utils/include/MathsUtils.h"
#include "Utils/include/Filters/ProcessModel.h"

/**
 * @class KalmanFilter
 * @brief Defines the base class for defining any kalman filter variant
 */
template <typename Scalar>
class KalmanFilter
{
public:
  /**
   * @brief KalmanFilter Initializes the filter
   * @param model process model
   * @param measSize measurement size
   */
  KalmanFilter(
    const boost::shared_ptr<ProcessModel<Scalar> >& model,
    const unsigned& measSize);

  /**
   * @brief KalmanFilter Initializes the filter
   * @param model process model
   * @param measSize measurement size
   * @param H measurement matrix
   */
  KalmanFilter(
    const boost::shared_ptr<ProcessModel<Scalar> >& model,
    const unsigned& measSize,
    const Matrix<Scalar, Dynamic, Dynamic>& H);

  /**
   * @brief ~KalmanFilter Destructor
   */
  ~KalmanFilter();

  /**
   * @brief predict Performs the filter prediction step
   */
  void predict();

  /**
   * @brief correct Performs the filter correction step
   * @param meas Input measurement
   */
  void correct(const Matrix<Scalar, Dynamic, 1>& meas);

  /**
   * @brief setMeasMatrix sets the measurement matrix
   * @param H measurement matrix
   */
  void setMeasMatrix(const Matrix<Scalar, Dynamic, Dynamic>& H)
  {
    ASSERT(H.rows() == measSize && H.cols() == stateSize)
    this->H = H;
    this->Ht = H.transpose();
  }

  /**
   * @brief setMeasNoiseCov sets the measurement noise covariance matrix
   * @param R measurement noise covariance matrix
   */
  void setMeasNoiseCov(const Matrix<Scalar, Dynamic, Dynamic>& R)
  {
    ASSERT(R.rows() == measSize && R.cols() == measSize)
    this->R = R;
  }

  /**
   * @brief setMeasNoiseCov sets the measurement noise covariance at the specified index
   * @param r measurement noise covariance
   */
  void setMeasNoiseCov(
    const Scalar& r,
    const unsigned i,
    const unsigned j)
  {
    ASSERT(i < measSize && j < measSize)
    this->R(i, j) = r;
  }

  /**
   * @brief setErrorNoiseCov sets the state error noise covariance matrix
   * @param P measurement noise covariance matrix
   */
  void setErrorNoiseCov(const Matrix<Scalar, Dynamic, Dynamic>& P)
  {
    ASSERT(P.rows() == stateSize && P.cols() == stateSize)
    this->P = P;
  }

private:
  //! Process model for the system update
  boost::shared_ptr<ProcessModel<Scalar> > model;

  //! Size of the state vector
  unsigned stateSize;

  //! Size of the measurements
  unsigned measSize;

  //! Measurement matrix
  Matrix<Scalar, Dynamic, Dynamic> H;

  //! Measurement matrix transpose
  Matrix<Scalar, Dynamic, Dynamic> Ht;

  //! Measurement noise covariance matrix
  Matrix<Scalar, Dynamic, Dynamic> R;

  //! State estimate error covariance
  Matrix<Scalar, Dynamic, Dynamic> P;
};
