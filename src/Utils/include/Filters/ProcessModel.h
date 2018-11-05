/**
 * @file Utils/inlclude/Filters/ProcessModel.h
 *
 * This file declares the class ProcessModel
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once

#include "Utils/include/MathsUtils.h"

/**
 * @class ProcessModel
 * @brief Defines the base class for defining a filter process update
 */
template <typename Scalar>
class ProcessModel
{
public:
  /**
   * @brief ProcessModel Initializes the process model based
   *   on the given data
   * @param stateSize size of the state vector
   * @param inputSize size of the control input vector
   */
  ProcessModel(
    const unsigned& stateSize,
    const unsigned& inputSize);

  /**
   * @brief ProcessModel Initializes the process model based
   *   on the given data
   * @param stateSize size of the state vector
   * @param inputSize size of the control input vector
   * @param A state transition matrix
   * @param B control input matrix
   */
  ProcessModel(
    const unsigned& stateSize,
    const unsigned& inputSize,
    const Matrix<Scalar, Dynamic, Dynamic>& A,
    const Matrix<Scalar, Dynamic, 1>& B);

  /**
   * @brief ProcessModel Initializes the process model based
   *   on the given data
   * @param stateSize size of the state vector
   * @param inputSize size of the control input vector
   * @param A state transition matrix
   * @param B control input matrix
   * @param C system output matrix
   */
  ProcessModel(
    const unsigned& stateSize,
    const unsigned& inputSize,
    const Matrix<Scalar, Dynamic, Dynamic>& A,
    const Matrix<Scalar, Dynamic, 1>& B,
    const Matrix<Scalar, Dynamic, Dynamic>& C);

  /**
   * @brief ~ProcessModel Destructor
   */
  ~ProcessModel();

  /**
   * @brief update The state-transition update
   */
  void update();

  /**
   * @brief getOutput returns the output defined by C * state
   */
  Matrix<Scalar, Dynamic, 1> getOutput();

  /**
   * @brief computeErrorCov computes the error cov for the next state based
   *   on the given previous error cov
   * @param prevP prev error covariance matrix
   */
  Matrix<Scalar, Dynamic, Dynamic>
    computeErrorCov(const Matrix<Scalar, Dynamic, Dynamic>& prevP);

  /**
   * @brief setStateMatrix sets the state transition matrix
   * @param A state transition matrix
   */
  void setStateMatrix(const Matrix<Scalar, Dynamic, Dynamic>& A)
  {
    ASSERT(A.rows() == stateSize && A.cols() == stateSize)
    this->A = A;
    this->At = A.transpose();
  }

  /**
   * @brief setNoiseCovMatrix sets the process noise covariance matrix
   * @param Q rocess noise covariance matrix
   */
  void setNoiseCovMatrix(const Matrix<Scalar, Dynamic, Dynamic>& Q)
  {
    ASSERT(Q.rows() == stateSize && Q.cols() == stateSize)
    this->Q = Q;
  }

  /**
   * @brief setControlMatrix sets the control matrix
   * @param B control input matrix
   */
  void setControlMatrix(const Matrix<Scalar, Dynamic, 1>& B)
  {
    ASSERT(B.rows() == stateSize && B.cols() == 1)
    this->B = B;
  }

  /**
   * @brief setOutputMatrix sets the system output matrix
   * @param C system output matrix
   */
  void setOutputMatrix(const Matrix<Scalar, Dynamic, Dynamic>& C)
  {
    ASSERT(C.cols() == stateSize)
    this->C = C;
  }

  /**
   * @brief setState sets the state vector
   * @param state state vector
   */
  void setState(const Matrix<Scalar, Dynamic, 1>& state)
  {
    ASSERT(state.rows() == stateSize && state.cols() == 1)
    this->state = state;
  }

  /**
   * @brief setControl sets the control input vector
   * @param input control input vector
   */
  void setControl(const Matrix<Scalar, 1, Dynamic>& input)
  {
    ASSERT(input.rows() == 1 && input.cols() == inputSize)
    this->input = input;
  }

  /**
   * @brief setControl sets the control input
   * @param input control input
   */
  void setControl(const Scalar& input)
  {
    ASSERT(input.rows() == 1 && input.cols() == 1)
    this->input[0] = input;
  }

  /**
   * @brief getStateSize returns the state size
   */
  unsigned getStateSize()
    { return stateSize; }

  /**
   * @brief getInputSize returns the input size
   */
  unsigned getInputSize()
    { return inputSize; }

  /**
   * @brief getStateMatrix returns the state matrix
   */
  Matrix<Scalar, Dynamic, Dynamic> getStateMatrix()
    { return A; }

  /**
   * @brief getControlMatrix returns the control matrix
   */
  Matrix<Scalar, Dynamic, 1> getControlMatrix()
    { return B; }

  /**
   * @brief getControlMatrix returns the system output matrix
   */
  Matrix<Scalar, Dynamic, Dynamic> getOutputMatrix()
    { return C; }

  /**
   * @brief getState returns the state
   */
  Matrix<Scalar, Dynamic, 1> getState()
    { return state; }

  /**
   * @brief getControl returns the control input
   */
  Matrix<Scalar, 1, Dynamic> getInput()
    { return input; }

  /**
   * @brief setUpdated Whether the model has been updated for this cycle.
   *   This is used by the filter to update only once for prediction step since
   *   the model can be updated externally as well
   * @param updated true or false
   */
  void setUpdated(const bool& updated) { this->updated = updated; }
private:
  //! State transition matrix
  Matrix<Scalar, Dynamic, Dynamic> A;

  //! State transition matrix transpose
  Matrix<Scalar, Dynamic, Dynamic> At;

  //! Control input matrix
  Matrix<Scalar, Dynamic, 1> B;

  //! System output matrix
  Matrix<Scalar, Dynamic, Dynamic> C;

  //! State vector
  Matrix<Scalar, Dynamic, 1> state;

  //! Control input vector
  Matrix<Scalar, 1, Dynamic> input;

  //! Process noise covariance matrix
  Matrix<Scalar, Dynamic, Dynamic> Q;

  //! Size of the state vector
  unsigned stateSize;

  //! Size of the control vector
  unsigned inputSize;

  //! Whether the update has been called already
  bool updated;
};
