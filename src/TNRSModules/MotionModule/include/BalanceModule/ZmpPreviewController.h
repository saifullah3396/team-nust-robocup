/**
 * @file MotionModule/BalanceModule/ZmpPreviewController.h
 *
 * This file declares the class ZmpPreviewController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/MathsUtils.h"

template <typename Scalar>
class ProcessModel;

/**
 * @class ZmpPreviewController
 * @brief The class for the implementation of zmp controller based
 * on Kajita's inverted-cart-table model.
 */
template <typename Scalar>
class ZmpPreviewController
{
public:
  /**
   * Default constructor for this class.
   */
  ZmpPreviewController(boost::shared_ptr<ProcessModel<Scalar> > model);

  /**
   * Default destructor for this class.
   */
  ~ZmpPreviewController()
  {
  }

  /**
   * Initiates the controller.
   */
  void initController();

  /**
   * Updates the system model based on the new control input
   *
   * @param zmpRef: zmp references for the given dimension
   *
   * @return Matrix<Scalar, 3, 1> next state vector
   */
  virtual Matrix<Scalar, Dynamic, 1> step(const Matrix<Scalar, Dynamic, 1>& zmpRef);

  /**
   * Sets the number of previewed steps for the preview controller.
   *
   * @param nPreviews: number of previews
   */
  void setPreviewLength(const unsigned& nPreviews)
  {
    this->nPreviews = nPreviews;
  }

protected:
  /**
   * Solves the discrete algebraic riccati equation to get the
   * optimal controller gains.
   *
   * @param stateSize: Number of state variables
   * @param A: State-Transition matrix
   * @param B: System Input matrix
   * @param Q: Cost weighting matrix
   * @param R: Cost weighting parameter
   *
   * @return Matrix<Scalar, Dynamic, Dynamic>
   */
  Matrix<Scalar, Dynamic, Dynamic> dare(
    const unsigned& stateSize,
    Matrix<Scalar, Dynamic, Dynamic>& A,
    Matrix<Scalar, Dynamic, 1>& B,
    Matrix<Scalar, Dynamic, Dynamic>& Q,
    const Scalar& R
  );

  //! Number of previews for the preview controller.
  int nPreviews;

  //! Error matrices, preview gain matrix, controlInputs (com jerk),
  Scalar intError, trueIntError;

  //! Observer gain
  Matrix<Scalar, Dynamic, 1> obsGain;

  //!  State-transition matrix for the system.
  Matrix<Scalar, Dynamic, Dynamic> matAl;

  //! Optimal gain matrices.
  Matrix<Scalar, Dynamic, Dynamic> kGain, gGain;

  Matrix<Scalar, Dynamic, 1> matPrevGain;

  //! Process model
  boost::shared_ptr<ProcessModel<Scalar> > model;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
