/**
 * @file Utils/src/Filters/ProcessModel.cpp
 *
 * This file implements the class ProcessModel
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#include "Utils/include/Filters/ProcessModel.h"
#include "Utils/include/DebugUtils.h"

template <typename Scalar>
ProcessModel<Scalar>::ProcessModel(
  const unsigned& stateSize,
  const unsigned& inputSize) :
  inputSize(inputSize),
  stateSize(stateSize),
  updated(false)
{
  this->A.resize(stateSize, stateSize);
  this->B.resize(stateSize);
  this->state.resize(stateSize);
  this->input.resize(inputSize);
  this->A.setIdentity();
  this->At = A.transpose();
  this->B.setIdentity();
  this->state.setZero();
  this->input.setZero();
}

template <typename Scalar>
ProcessModel<Scalar>::ProcessModel(
  const unsigned& stateSize,
  const unsigned& inputSize,
  const Matrix<Scalar, Dynamic, Dynamic>& A,
  const Matrix<Scalar, Dynamic, 1>& B) :
  inputSize(inputSize),
  stateSize(stateSize),
  A(A),
  B(B)
{
  ASSERT(this->A.rows() == stateSize && this->A.cols() == stateSize)
  ASSERT(this->B.rows() == stateSize && this->B.cols() == 1)
  this->At = this->A.transpose();
  this->state.resize(stateSize);
  this->input.resize(inputSize);
  this->state.setZero();
  this->input.setZero();
}

template <typename Scalar>
ProcessModel<Scalar>::ProcessModel(
  const unsigned& stateSize,
  const unsigned& inputSize,
  const Matrix<Scalar, Dynamic, Dynamic>& A,
  const Matrix<Scalar, Dynamic, 1>& B,
  const Matrix<Scalar, Dynamic, Dynamic>& C) :
  inputSize(inputSize),
  stateSize(stateSize),
  A(A),
  B(B),
  C(C)
{
  ASSERT(this->A.rows() == stateSize && this->A.cols() == stateSize)
  ASSERT(this->B.rows() == stateSize && this->B.cols() == 1)
  ASSERT(this->C.cols() == stateSize)
  this->At = this->A.transpose();
  this->state.resize(stateSize);
  this->input.resize(inputSize);
  this->state.setZero();
  this->input.setZero();
}

template <typename Scalar>
ProcessModel<Scalar>::~ProcessModel()
{
}

template <typename Scalar>
void ProcessModel<Scalar>::update()
{
  if (!updated) {
    state = A * state + B * input;
    updated = true;
  }
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> ProcessModel<Scalar>::getOutput()
{
  return C * state;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
ProcessModel<Scalar>::computeErrorCov(
  const Matrix<Scalar, Dynamic, Dynamic>& prevP)
{
  return A * prevP * At + Q;
}

template class ProcessModel<float>;
template class ProcessModel<double>;
