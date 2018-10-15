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

template<typename Scalar>
void
ZmpPreviewController<Scalar>::setState(const Matrix<Scalar, 3, 1>& state)
{
  this->state = state;
}

template<typename Scalar>
void
ZmpPreviewController<Scalar>::initController()
{
  Matrix<Scalar, Dynamic, Dynamic> matCA;
  Matrix<Scalar, Dynamic, Dynamic> matCB;

  matA << 1.f, samplingTime, pow(samplingTime, 2) / 2, 0.f, 1.f, samplingTime, 0.f, 0.f, 1.f;

  matB << pow(samplingTime, 3) / 6, pow(samplingTime, 2) / 2, samplingTime;

  matC << 1.f, 0.f, -comHeight / gConst;

  matI << 1.f, 0.f, 0.f, 0.f;

  matCA = matC * matA;
  matCB = matC * matB;

  matAAug << 1.f, matCA(0, 0), matCA(0, 1), matCA(0, 2), 0.f, matA(0, 0), matA(
    0,
    1), matA(0, 2), 0.f, matA(1, 0), matA(1, 1), matA(1, 2), 0.f, matA(2, 0), matA(
    2,
    1), matA(2, 2);

  matBAug << matCB(0, 0), matB(0, 0), matB(1, 0), matB(2, 0);
  matCAug << 1.f, 0.f, 0.f, 0.f;

  matQ.setIdentity(4, 4);
  matQ(0, 0) = 1.f;
  matQ(1, 1) = 0.f;
  matQ(2, 2) = 0.f;
  matQ(3, 3) = 0.f;

  gGain = dare(matAAug, matBAug, matQ, R);

  Scalar temp = pow((R + (matBAug.transpose() * gGain * matBAug)(0, 0)), -1);
  kGain = temp * matBAug.transpose() * gGain * matAAug;
  matPAAug = matAAug - matBAug * kGain;

  matPrevGain.resize(nPreviews, 1);
  for (int n = 0; n < nPreviews; ++n) {
    if (n == 0) {
      matPrevGain(n, 0) = -kGain(0, 0);
      prevState = -matPAAug.transpose() * gGain * matI;
    } else {
      matPrevGain(n, 0) = temp * (matBAug.transpose() * prevState)(0, 0);
      prevState = matPAAug.transpose() * prevState;
    }
  }
  /*cout << "matA	" << matA << endl;
   cout << "matB	" << matB << endl;
   cout << "matC	" << matC << endl;
   cout << "matAAug	" << matAAug << endl;
   cout << "matBAug	" << matBAug << endl;
   cout << "matCAug	" << matCAug << endl;
   cout << "matPrevGain	" << matPrevGain << endl;
   cout << "kGain		" << kGain << endl;
   cout << "gGain		" << gGain << endl;
   cout << "matQ		" << matQ << endl;
   cout << "R		" << R << endl;*/
}

template<typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
ZmpPreviewController<Scalar>::dare(Matrix<Scalar, 4, 4>& A, Matrix<Scalar, 4, 1>& B, Matrix<Scalar, 4, 4>& Q, Scalar R)
{
  bool converged = false;
  Matrix<double, Dynamic, Dynamic> P(4, 4);
  P.setIdentity();
  for (int i = 0; i < 10000; ++i) {
    Matrix<Scalar, Dynamic, Dynamic> AX = (A.template cast<double>()).transpose() * P;
    Matrix<Scalar, Dynamic, Dynamic> AXA = AX * (A.template cast<double>());
    Matrix<Scalar, Dynamic, Dynamic> AXB = AX * (B.template cast<double>());
    double M =
      (((B.template cast<double>()).transpose() * P * (B.template cast<double>())).array() + double(
        R))(0, 0);
    Matrix<Scalar, 4, 4> Pnew =
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
Matrix<Scalar, 3, 1>
ZmpPreviewController<Scalar>::step(const Matrix<Scalar, 3, 1>& comState, const Matrix<Scalar, Dynamic, 1>& zmpRef)
{
  setState(comState);
  prevGain = 0;
  for (unsigned i = 0; i < nPreviews; ++i) {
    prevGain = prevGain + matPrevGain(i, 0) * zmpRef[i];
  }
  zmpPosition = (matC * state)(0, 0);
  zmpError = zmpPosition - zmpRef[0];
  intError = intError + zmpError;// * samplingTime / 2;
  controlInput =
    -kGain(0, 0) * intError - (kGain.block(0, 1, 1, 3) * state)(0, 0) - prevGain;
  Matrix<Scalar, 3, 1> nextState = matA * state + matB * controlInput;
  return nextState;
}

template class ZmpPreviewController<MType>;
