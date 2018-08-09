/**
 * @file MotionModule/BalanceModule/ZmpPreviewController.cpp
 *
 * This file implements the class ZmpPreviewController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018 
 */

#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "ConfigManager/include/ConfigManager.h"

void
ZmpPreviewController::setState(const Vector3f& state)
{
  this->state = state;
}

void
ZmpPreviewController::initController()
{
  /*#ifdef DEBUG_BUILD
  string logsDir = ConfigManager::getLogsDirPath() + string("BalanceModule/");
  zmpRefLog.open(
	(logsDir + "ZmpRef.txt").c_str(),
    std::ofstream::out | std::ofstream::trunc);
  zmpRefLog.close();
  zmpLog.open(
	(logsDir + "Zmp.txt").c_str(),
    std::ofstream::out | std::ofstream::trunc);
  zmpLog.close();
  comRefLog.open(
	(logsDir + "ComRef.txt").c_str(),
    std::ofstream::out | std::ofstream::trunc);
  comRefLog.close();
  comVRefLog.open(
	(logsDir + "ComVRef.txt").c_str(),
    std::ofstream::out | std::ofstream::trunc);
  comVRefLog.close();
  pGainLog.open(
	(logsDir + "pGain.txt").c_str(),
    std::ofstream::out | std::ofstream::trunc);
  pGainLog.close();
  #endif*/
  
  MatrixXf matCA;
  MatrixXf matCB;

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

  float temp = pow((R + (matBAug.transpose() * gGain * matBAug)(0, 0)), -1);
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

Matrix<float, Dynamic, Dynamic>
ZmpPreviewController::dare(Matrix4f& A, Vector4f& B, Matrix4f& Q, float R)
{
  bool converged = false;
  MatrixXd P(4, 4);
  P.setIdentity();
  for (int i = 0; i < 10000; ++i) {
    MatrixXd AX = (A.cast<double>()).transpose() * P;
    MatrixXd AXA = AX * (A.cast<double>());
    MatrixXd AXB = AX * (B.cast<double>());
    double M =
      (((B.cast<double>()).transpose() * P * (B.cast<double>())).array() + double(
        R))(0, 0);
    Matrix4d Pnew =
      AXA - AXB * (1.0 / M) * AXB.transpose() + (Q.cast<double>());
    double relError = (Pnew - P).norm() / Pnew.norm();
    P = Pnew;
    if (relError < 1e-10) {
      converged = true;
      break;
    }
  }
  return P.cast<float>();
}

Vector3f
ZmpPreviewController::step(const Vector3f& comState, const VectorXf& zmpRef)
{
  setState(comState);
  prevGain = 0;
  for (unsigned i = 0; i < nPreviews; ++i) {
    prevGain = prevGain + matPrevGain(i, 0) * zmpRef[i];
  }
  zmpPosition = (matC * state)(0, 0);
  zmpError = zmpPosition - zmpRef[0];
  intError = intError + zmpError * samplingTime / 2;
  controlInput =
    -kGain(0, 0) * intError - (kGain.block(0, 1, 1, 3) * state)(0, 0) - prevGain;
  Vector3f nextState = matA * state + matB * controlInput;
  //timeStep = timeStep + samplingTime;
  return nextState;
  
  /*#ifdef DEBUG_BUILD
  string logsDir = ConfigManager::getLogsDirPath() + string("BalanceModule/");
  zmpRefLog.open(
	logsDir + "ZmpRef.txt",
    fstream::app | fstream::out
  );
  zmpRefLog << zmpPreviewedX[0] << "   " << zmpPreviewedY[0] << "\n";
  zmpRefLog.close();
  #endif*/
 
  //cout << "zmpError[0]	" << zmpPosition[0] << endl;
  //cout << "zmpError[1]	" << zmpPosition[1] << endl;

  //cout << "zmpError[0]	" << zmpError[0] << endl;
  //cout << "zmpError[1]	" << zmpError[1] << endl;
  //cout << "intError[0]	" << intError[0] << endl;
  //cout << "intError[1]	" << intError[1] << endl;

  /*#ifdef DEBUG_BUILD
  zmpLog.open(
	(logsDir + "Zmp.txt").c_str(),
    fstream::app | fstream::out
  );
  zmpLog << zmpPosition[0] << "    " << zmpPosition[1] << "\n";
  zmpLog.close();

  pGainLog.open(
	(logsDir + "pGain.txt").c_str(),
    fstream::app | fstream::out
  );
  pGainLog << prevGain[0] << "    " << prevGain[1] << "\n";
  pGainLog.close();

  comRefLog.open(
	(logsDir + "comRef.txt").c_str(),
    fstream::app | fstream::out
  );
  comRefLog << newStateX(0, 0) << "    " << newStateY(0, 0) << "\n";
  comRefLog.close();

  comVRefLog.open(
	(logsDir + "comVRef.txt").c_str(),
    fstream::app | fstream::out
  );
  comVRefLog << newStateX(1, 0) << "    " << newStateY(1, 0) << "\n";
  comVRefLog.close();
  #endif*/
  
  //timeStep = timeStep + samplingTime;
}
