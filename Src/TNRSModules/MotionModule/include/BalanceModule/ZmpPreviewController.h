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
#include "Utils/include/DataUtils.h"
#include "Utils/include/EnvConsts.h"
#include "Utils/include/MathsUtils.h"

/**
 * @class ZmpPreviewController
 * @brief The class for the implementation of zmp controller based
 * on Kajita's inverted-cart-table model.
 */
class ZmpPreviewController
{
public:
  /**
   * Default constructor for this class.
   */
  ZmpPreviewController() : 
    intError(0.f),
    zmpError(0.f),
    prevGain(0.f),
    controlInput(0.f),
    zmpPosition(0.f),
    timeStep(0.f)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~ZmpPreviewController()
  {
  }

  /**
   * Initiates the controller.
   */
  void
  initController();

  /**
   * Updates the system model and generates the next state based on
   * the control input.
   *
   * @param comState: center of mass state (p, v, a) in single dimension
   * @param zmpRef: zmp references for the given dimension
   *
   * @return Vector3f next state vector
   */
  Vector3f step(const Vector3f& comState, const VectorXf& zmpRef);

  /**
   * Sets the constant center of mass height (assumption in Kajita's
   * cart-table model).
   *
   * @param comHeight: height in meters
   */
  void
  setComHeight(float comHeight)
  {
    this->comHeight = comHeight;
  }

  /**
   * Sets the number of previewed steps for the preview controller.
   *
   * @param nPreviews: number of previews
   */
  void
  setPreviewLength(int nPreviews)
  {
    this->nPreviews = nPreviews;
  }

  /**
   * Sets the sampling time for the digital controller.
   *
   * @param samplingTime: time in secs
   */
  void
  setSamplingTime(float samplingTime)
  {
    this->samplingTime = samplingTime;
  }

private:
  /**
   * Sets the current state of the system.
   *
   * @param state: center of mass state
   */
  void
  setState(const Vector3f& state);

  /**
   * Solves the discrete algebraic riccati equation to get the
   * optimal controller gains.
   *
   * @param A: State-Transition matrix
   * @param B: System Input matrix
   * @param Q: Cost weighting matrix
   * @param R: Cost weighting parameter
   *
   * @return MatrixXf
   */
  MatrixXf
  dare(Matrix4f& A, Vector4f& B, Matrix4f& Q, float R);

  //! Cost weighting parameter for finding optimal gains for the system.
  static constexpr float R = 1e-6;

  //! Number of previews for the preview controller.
  int nPreviews;

  //! Center of mass height.
  float comHeight;

  //! Sampling time.
  float samplingTime;

  //! Time-step of iterations.
  float timeStep;

  //! State-matrices in current and next iterations 
  Vector3f state;

  //! Error matrices, preview gain matrix, controlInputs (com jerk),
  float intError, zmpError, prevGain, controlInput, zmpPosition;

  //!  State-transition matrix for the system.
  Matrix3f matA;

  //!  Augmented state and gain matrices for optimal control design.
  Matrix4f matAAug, matQ, matPAAug;

  //! Input matrix for the system.
  Vector3f matB;

  //! Augmented input and identity matrices.
  Vector4f matBAug, matI;

  //! Output matrix for the system.
  Matrix<float, 1, 3> matC;

  //! Augmented Output matrix for the system.
  Matrix<float, 1, 4> matCAug;

  //! Optimal gain matrices.
  MatrixXf kGain, gGain, matPrevGain, prevState;

  //! File streams for data-logging.
  fstream zmpLog, zmpRefLog, comRefLog, pGainLog, comVRefLog;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
