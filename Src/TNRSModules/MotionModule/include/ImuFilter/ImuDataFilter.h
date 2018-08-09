/**
 * @file ImuDataFilter/ImuDataFilter.h
 *
 * This file declares the class ImuDataFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "Utils/RandomLib/Random.hpp"
#include "QuaternionsHelper.h"
#include "Utils/include/MathsUtils.h"

using namespace Utils;
using namespace RandomLib;

/**
 * @class ImuDataFilter
 * @brief The class filters the imu data for attitude measurement and
 *   acceleration data correction.
 */
template<typename T>
  class ImuDataFilter
  {
  public:
    /**
     * @brief Default constructor for this class.
     */
    ImuDataFilter(DAQModule* daqModule);

    /**
     * @brief Default destructor for this class.
     */
    ~ImuDataFilter()
    {
    }

    /**
     * @brief This function sets the initial parameters of the filter.
     * @return void
     */
    void
    initiate();

    /**
     * @brief This function performs the main filter update step.
     * @return void
     */
    void
    update();

    /**
     * @brief This function performs the prediction step of the kalman
     *   filter.
     * @return void
     */
    void
    predict();

    /**
     * @brief This function solve the process model of the system.
     * @param xdot: State derivative.
     * @param F: F-matrix.
     * @param G: G-matrix.
     * @return void
     */
    void
    process(Matrix<T, Dynamic, 1>& xdot, Matrix<T, Dynamic, Dynamic>& F,
      Matrix<T, Dynamic, Dynamic>& G);

    /**
     * @brief This function solve the measurement model of the accelerometer
     *   using only the gravity term.
     * @param H: H-matrix.
     * @return void
     */
    void
    measurement(Matrix<T, 3, 1>& acc, Matrix<T, Dynamic, Dynamic>& H);

    /**
     * @brief This function performs the correction step of the ekf.
     * @param z: Actual measurements.
     * @param zhat: Predicted measurements.
     * @param H: H-matrix.
     * @param R: R-matrix.
     * @return void
     */
    void
    correction(Matrix<T, Dynamic, 1> z, Matrix<T, Dynamic, 1> zhat,
      Matrix<T, Dynamic, Dynamic> H, Matrix<T, Dynamic, Dynamic> R);

    /**
     * @brief This function performs the correction step for the acceleration
     *   measurements.
     * @return void
     */
    void
    correctAccMeasurement();

    /**
     * @brief This function copies the state elements from the state vector
     * @param q: The orientation of the imu in the form of quaternions.
     * @param pos: Position.
     * @param vel: Velocity.
     * @param omegaBias: Angular velocity bias.
     * @param accBias: Acceleration bias.
     * @return void
     */
    void
    getState(Quaternion<T>& q, Matrix<T, 3, 1>& pos, Matrix<T, 3, 1>& vel,
      Matrix<T, 3, 1>& omegaBias, Matrix<T, 3, 1>& accBias);

  private:
    //!Number of elements in the state vector.
    const size_t nState = 16;

    //!System state vector.
    Matrix<T, Dynamic, 1> x;

    //!State covariance matrix.
    Matrix<T, Dynamic, Dynamic> P;

    //!State covariance matrix.
    const Matrix<T, 3, 1> gravVec = Matrix<T, 3, 1>(0, 0, gConst);

    //!Gyrometer covariance.
    const T gyroCov = 0.01;

    //!Accelerometer covariance.
    const T accCov = 0.1;

    //!Gravity covariance.
    const T gravCov = 0.1;

    //!Observation noise.
    Matrix<T, Dynamic, Dynamic> Q;

    //!Measurement noise.
    const Matrix<T, 3, 3> gravR = Matrix<T, 3, 3>::Identity() * gravCov;

    //!Acceleraton input in current step.
    Matrix<T, 3, 1> acc;

    //!Gyro input in current step.
    Matrix<T, 3, 1> gyro;

    //!The variable that checks whether the filter has been initiated.
    bool initiated;

    //!Time step after initiation.
    float timeStep;

    //!Cycle time.
    const float cycleTime = 0.01;
  };
