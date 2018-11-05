/**
 * @file MotionModule/BalanceModule/ZmpPreviewController.cpp
 *
 * This file implements the class ZmpPreviewController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018 
 */

#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "Utils/include/ConfigManager.h"
#include "Utils/include/EnvConsts.h"
#include "Utils/include/Filters/ProcessModel.h"

template<typename Scalar>
ZmpPreviewController<Scalar>::ZmpPreviewController(
  boost::shared_ptr<ProcessModel<Scalar> > model) :
  model(model),
  intError(0.0),
  trueIntError(0.0)
{
}

template<typename Scalar>
void ZmpPreviewController<Scalar>::initController()
{
  unsigned stateSize = model->getStateSize();
  //! Get the actual state matrices
  Matrix<Scalar, Dynamic, Dynamic> matA = model->getStateMatrix();
  Matrix<Scalar, Dynamic, 1> matB = model->getControlMatrix();
  Matrix<Scalar, Dynamic, Dynamic> matC = model->getOutputMatrix();
  //!  Augmented state and gain matrices for optimal control design.
  //cout << "matA:\n" << matA << endl;
  //cout << "matB:\n" << matB << endl;
  //cout << "matC:\n" << matC << endl;
  Matrix<Scalar, Dynamic, 1> matI(stateSize + 1, 1);
  matI << 1.0, Matrix<Scalar, Dynamic, 1>::Zero(stateSize, 1);
  //cout << "matI: " << matI << endl;
  Matrix<Scalar, Dynamic, Dynamic> matCA = matC * matA;
  //cout << "matCA: " << matCA << endl;
  Matrix<Scalar, 1, 1> matCB = matC * matB;
  //cout << "matCB: " << matCB << endl;
  Matrix<Scalar, Dynamic, Dynamic> mat1CA(1, stateSize + 1);
  mat1CA << 1, matCA;
  //cout << "mat1CA: " << mat1CA << endl;
  Matrix<Scalar, Dynamic, Dynamic> mat0A(stateSize, stateSize + 1);
  //cout << "mat0A: " << mat0A << endl;
  mat0A << Matrix<Scalar, Dynamic, 1>::Zero(stateSize, 1), matA;
  Matrix<Scalar, Dynamic, Dynamic> matAAug(stateSize + 1, stateSize + 1), matCAug(1, stateSize + 1);
  Matrix<Scalar, Dynamic, 1> matBAug(stateSize + 1, 1);
  matAAug << mat1CA, mat0A;
  //cout << "matAAug:\n" << matAAug << endl;
  matBAug <<
    matCB,
    matB;
  //cout << "matBAug:\n" << matBAug << endl;
  matCAug << 1.0, Matrix<Scalar, 1, Dynamic>::Zero(1, stateSize);
  //cout << "matCAug:\n" << matCAug << endl;

  Matrix<Scalar, Dynamic, Dynamic> matQ(stateSize + 1, stateSize + 1);
  matQ.setZero();
  matQ(0, 0) = 1.0;
  Scalar R = 1e-6;
  gGain = dare(stateSize+1, matAAug, matBAug, matQ, R);
  Scalar temp = 1 / (R + (matBAug.transpose() * gGain * matBAug)(0, 0));
  kGain = temp * matBAug.transpose() * gGain * matAAug;
  Matrix<Scalar, Dynamic, Dynamic> matPAAug = matAAug - matBAug * kGain;
  matPrevGain.resize(nPreviews, 1);
  matPrevGain[0] = -kGain(0, 0);
  Matrix<Scalar, Dynamic, Dynamic> prevState = -matPAAug.transpose() * gGain * matI;
  for (int n = 1; n < nPreviews; ++n) {
      matPrevGain[n] = temp * (matBAug.transpose() * prevState)(0, 0);
      prevState = matPAAug.transpose() * prevState;
  }
  /*// using duality system<=>observer
  Matrix<Scalar, 3, 3> obsQ;
  obsQ.setZero();
  Matrix<Scalar, 3, 3> obsA = matA.transpose();
  Matrix<Scalar, 3, 1> obsB = matC.transpose();
  Matrix<double, Dynamic, Dynamic>
    P = dare(obsA, obsB, obsQ, 10).transpose();
  obsGain = pow((10 + (obsB.transpose() * P * obsB)(0, 0)), -1) * obsB.transpose() * P * obsA;
  matAl = matA - obsGain * matC;
  cout << "obsGain" << obsGain << endl;
  cout << "matAl\n" << matAl << endl;
  EigenSolver<Matrix<Scalar, 3, 3> > eigensolver(matAl);
  cout << eigensolver.eigenvalues() << endl;*/
}

template<typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> ZmpPreviewController<Scalar>::dare(
  const unsigned& size,
  Matrix<Scalar, Dynamic, Dynamic>& A,
  Matrix<Scalar, Dynamic, 1>& B,
  Matrix<Scalar, Dynamic, Dynamic>& Q,
  const Scalar& R)
{
  bool converged = false;
  Matrix<double, Dynamic, Dynamic> P(size, size);
  P.setIdentity();
  for (int i = 0; i < 50000; ++i) {
    Matrix<Scalar, Dynamic, Dynamic> AX = (A.template cast<double>()).transpose() * P;
    Matrix<Scalar, Dynamic, Dynamic> AXA = AX * (A.template cast<double>());
    Matrix<Scalar, Dynamic, Dynamic> AXB = AX * (B.template cast<double>());
    double M =
      (((B.template cast<double>()).transpose() * P * (B.template cast<double>())).array() + double(
        R))(0, 0);
    Matrix<Scalar, Dynamic, Dynamic> Pnew =
      AXA - AXB * (1.0 / M) * AXB.transpose() + (Q.template cast<double>());
    double relError = (Pnew - P).norm() / Pnew.norm();
    P = Pnew;
    if (relError < 1e-10) {
      converged = true;
      break;
    }
  }
  return P.cast<Scalar>();
}

template<typename Scalar>
Matrix<Scalar, Dynamic, 1> ZmpPreviewController<Scalar>::step(const Matrix<Scalar, Dynamic, 1>& zmpRef)
{
  Matrix<Scalar, Dynamic, 1> state = model->getState();
  Scalar prevGain = 0;
  for (unsigned i = 0; i < nPreviews; ++i) {
    //cout << "matPrevGain(i, 0):" << matPrevGain[i] << endl;
    //cout << "zmpRef[i]: " << zmpRef[i] << endl;
    prevGain = prevGain + matPrevGain[i] * zmpRef[i];
    //cout << "prevGain: " << prevGain << endl;
  }
  //cout << "State: " << state << endl;
  //cout << "zmp:" << model->getOutput()(0, 0) << endl;
  //cout << "zmpRef[0]:" << zmpRef[0] << endl;
  //cout << "error:" << model->getOutput()(0, 0) - zmpRef[0] << endl;
  //cout << "kGain: " << kGain << endl;
  trueIntError = trueIntError + (model->getOutput()(0, 0) - zmpRef[0]);
  Scalar controlInput =
    -kGain(0, 0) * trueIntError - (kGain.block(0, 1, 1, 3) * state)(0, 0) - prevGain;
  //cout << "kGain00: " << -kGain(0, 0) << endl;
  //cout << "kGain.block(0, 1, 1, 3): " << kGain.block(0, 1, 1, 3) << endl;
  //cout << "prevGain: " << prevGain << endl;

  model->setControl(controlInput);
  model->setUpdated(false);
  model->update();
  //cout << "State new: " << model->getState() << endl;
  return model->getState();
  //intError = intError + ((matC * comState)(0, 0) - zmpRef[0]);
  //controlInput =
  //  -kGain(0, 0) * intError - (kGain.block(0, 1, 1, 3) * comState)(0, 0) - prevGain;
  //Matrix<Scalar, 3, 1> estState = matAl * comState + matB * controlInput + obsGain * (matC * state)(0, 0);
}

template class ZmpPreviewController<MType>;
