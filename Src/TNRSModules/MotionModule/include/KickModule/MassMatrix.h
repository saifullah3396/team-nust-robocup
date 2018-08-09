#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

#ifndef MASS_MATRIX_H
#define MASS_MATRIX_H

namespace KickUtils
{

  inline Matrix<float, 5, 5>
  calcMassMatrix(const unsigned& legIndex, const VectorXf& jointAngles)
  {
    Matrix<float, 5, 5> M;
    float T1, T2, T3, T4, T5, T6;
    T1 = 0;
    T2 = jointAngles[0];
    T3 = jointAngles[1];
    T4 = jointAngles[2];
    T5 = jointAngles[3];
    T6 = jointAngles[4];
    if (legIndex == CHAIN_L_LEG) {
      M(0, 0) =
        cos(T3 * 2.0) * 2.74001505614E-3 - cos(T3 * 2.0 + T4 + T5) * 9.18996E-5 + sin(
          T3 * 2.0) * 2.8893758832E-5 + cos(T3 * 2.0 + T4 * 2.0) * 1.074369128777E-3 + sin(
          T3 * 2.0 + T4 + T5) * 6.0372E-6 + sin(T3 * 2.0 + T4 * 2.0) * 6.7397753136E-5 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 - cos(
          T3 * 2.0 + T4 * 2.0 + T5) * 9.45646884E-5 + sin(
          T3 * 2.0 + T4 * 2.0 + T5) * 6.2122788E-6 + cos(T4) * 2.86831552E-3 - cos(
          T5) * 9.45646884E-5 + sin(T4) * 1.3654326E-4 + sin(T5) * 6.2122788E-6 + cos(
          T3 * 2.0 + T4) * 2.86831552E-3 + sin(T3 * 2.0 + T4) * 1.3654326E-4 + cos(
          T3 * 2.0 + T4 * 2.0 + T5 * 2.0) * 3.1339776E-6 - sin(
          T3 * 2.0 + T4 * 2.0 + T5 * 2.0) * 4.135482E-7 + (cos(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T6) * sin(T2) * 3.3E-3 + cos(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 - cos(T6) * sin(T2) * 5.67072E-4 + cos(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) + (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T2) * cos(T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(
          T2) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) + 4.840309020603E-3;
      M(0, 1) =
        cos(T3 + T4) * (-3.07222335E-6) + sin(T3 + T4) * 3.747917376E-5 + cos(
          T2) * 2.94141701664E-5 - cos(T3) * 1.188446064E-6 - cos(T3 + T4 + T5) * 1.750788E-8 + sin(
          T2) * 3.71155324225901E-6 + sin(T3) * 1.17982029144E-4 - sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T6) * sin(
          T2) * 5.67072E-4 + cos(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(
          T2) * sin(T6) * 5.5658976E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 2) =
        cos(T3 + T4) * (-3.07222335E-6) + sin(T3 + T4) * 3.747917376E-5 - cos(
          T2) * 2.69987522866E-5 - cos(T3 + T4 + T5) * 1.750788E-8 - sin(T2) * 4.198737047740991E-6 - sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 - cos(T6) * sin(T2) * 5.67072E-4 + cos(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 3) =
        cos(T2) * 3.9020298434E-6 - cos(T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 1.820787182259012E-6 - sin(
          T3 + T4 + T5) * 2.6650884E-7 + sin(T2) * (cos(T3 + T4 + T5) * 2.542E3 - cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T6) * sin(
          T2) * 5.67072E-4 + cos(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(
          T2) * sin(T6) * 5.5658976E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 - cos(T2) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 + cos(T2) * cos(T6) * 5.67072E-4 + cos(T2) * sin(
          T6) * 5.5658976E-3 + sin(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 4) =
        cos(T3 + T4 + T5) * 4.128464489040001E-4 - (cos(T2) * cos(T6) * (-3.239E-2) + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * (cos(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T6) * sin(T2) * 5.67072E-4 + cos(
          T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) + (cos(T6) * sin(T2) * 3.239E-2 - sin(
          T2) * sin(T6) * 3.3E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * (sin(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T2) * cos(T6) * 5.67072E-4 + cos(
          T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.67072E-4) - cos(T2) * sin(T3 + T4 + T5) * 2.105636022000012E-6 - sin(
          T2) * sin(T3 + T4 + T5) * 3.734584149999999E-7;
      M(1, 0) =
        cos(T3 + T4) * (-3.07222335E-6) + sin(T3 + T4) * 3.747917376E-5 + cos(
          T2) * 2.94141701664E-5 - cos(T3) * 1.188446064E-6 - cos(T3 + T4 + T5) * 1.750788E-8 + sin(
          T2) * 3.71155324225901E-6 + sin(T3) * 1.17982029144E-4 - sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T6) * sin(
          T2) * 3.3E-3 + cos(T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(
          T6) * 3.239E-2 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + cos(T2) * cos(
          T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.239E-2 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(1, 1) =
        cos(T2 * 2.0) * 3.341647803599997E-5 - sin(T2 * 2.0) * 5.809967302607E-6 - cos(
          T4 + T5) * 1.837992E-4 + sin(T4 + T5) * 1.20744E-5 + cos(T4) * 5.73663104E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 2.7308652E-4 + sin(T5) * 1.24245576E-5 + sin(
          T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + pow(
          cos(T2),
          2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
            T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + pow(sin(T2), 2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
            T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + (cos(T3 + T4) * 1.029E-1 + cos(T3) * (1.0 / 1.0E1) + sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(T3 + T4) * 1.7682336E-2 + cos(
          T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + 8.603747842947E-3;
      M(1, 2) =
        cos(T2 * 2.0) * 1.117364428399999E-5 - sin(T2 * 2.0) * 3.697011234057E-6 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 + cos(T4) * 2.86831552E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 1.3654326E-4 + sin(T5) * 1.24245576E-5 + sin(
          T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + (cos(
          T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + cos(T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + pow(
          cos(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 2.634632242195E-3;
      M(1, 3) =
        cos(T2 * 2.0) * (-1.786979863E-5) + sin(T2 * 2.0) * 8.832120402999999E-9 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 - cos(T5) * 9.45646884E-5 + sin(
          T5) * 6.2122788E-6 + sin(T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(T3 + T4) * 1.7682336E-2 + cos(
          T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) - pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 - pow(
          sin(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.6395464181E-5;
      M(1, 4) =
        -cos(T3 + T4 + T5) * (cos(T2) * 3.734584149999999E-7 - sin(T2) * 2.105636022000012E-6) - sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.7682336E-2 + cos(
          T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 - cos(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + cos(
          T2) * (cos(T6) * sin(T2) * 3.239E-2 - sin(T2) * sin(T6) * 3.3E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 3.239E-2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-6 + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + sin(
          T2) * (cos(T2) * cos(T6) * (-3.239E-2) + cos(T2) * sin(T6) * 3.3E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 3.239E-2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-6;
      M(2, 0) =
        cos(T3 + T4) * (-3.07222335E-6) + sin(T3 + T4) * 3.747917376E-5 - cos(
          T2) * 2.69987522866E-5 - cos(T3 + T4 + T5) * 1.750788E-8 - sin(T2) * 4.198737047740991E-6 - sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 - cos(T6) * sin(T2) * 3.3E-3 + cos(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T2) * cos(T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(
          T2) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(2, 1) =
        cos(T2 * 2.0) * 1.117364428399999E-5 - sin(T2 * 2.0) * 3.697011234057E-6 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 + cos(T4) * 2.86831552E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 1.3654326E-4 + sin(T5) * 1.24245576E-5 + sin(
          T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(
          T3 + T4) * 1.029E-1 + cos(T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + pow(
          cos(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 2.634632242195E-3;
      M(2, 2) =
        cos(T2 * 2.0) * 1.117364428399999E-5 - sin(T2 * 2.0) * 3.697011234057E-6 - cos(
          T5) * 1.891293768E-4 + sin(T5) * 1.24245576E-5 + sin(T2) * (cos(T2) * 8.7192501205E-6 + sin(
          T2) * 8.729104631600002E-5) + pow(cos(T2), 2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
            T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + pow(sin(T2), 2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
            T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + (cos(T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + 2.634632242195E-3;
      M(2, 3) =
        cos(T2 * 2.0) * (-1.786979863E-5) + sin(T2 * 2.0) * 8.832120402999999E-9 - cos(
          T5) * 9.45646884E-5 + sin(T5) * 6.2122788E-6 + sin(T2) * (cos(T2) * 8.7192501205E-6 + sin(
          T2) * 8.729104631600002E-5) + (sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) - pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 - pow(sin(T2), 2.0) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.6395464181E-5;
      M(2, 4) =
        -cos(T3 + T4 + T5) * (cos(T2) * 3.734584149999999E-7 - sin(T2) * 2.105636022000012E-6) + cos(
          T2) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T6) * sin(T2) * 3.239E-2 - sin(T2) * sin(T6) * 3.3E-3 + cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.239E-2) * 1.7184E-6 + sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * (-3.239E-2) + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * 1.7184E-6 - cos(T2) * sin(
          T3 + T4 + T5) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) - sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.7682336E-2 + sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(3, 0) =
        cos(T2) * 3.9020298434E-6 - cos(T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 1.820787182259012E-6 - sin(
          T3 + T4 + T5) * 2.6650884E-7 + sin(T2) * (cos(T3 + T4 + T5) * 2.542E3 - cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T6) * sin(
          T2) * 3.3E-3 + cos(T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(
          T6) * 3.239E-2 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6 - cos(T2) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T2) * cos(T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(
          T2) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(3, 1) =
        cos(T2 * 2.0) * (-1.786979863E-5) + sin(T2 * 2.0) * 8.832120402999999E-9 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 - cos(T5) * 9.45646884E-5 + sin(
          T5) * 6.2122788E-6 + sin(T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + (sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(T3 + T4) * 1.029E-1 + cos(
          T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) - pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 - pow(
          sin(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.6395464181E-5;
      M(3, 2) =
        cos(T2 * 2.0) * (-1.786979863E-5) + sin(T2 * 2.0) * 8.832120402999999E-9 - cos(
          T5) * 9.45646884E-5 + sin(T5) * 6.2122788E-6 + sin(T2) * (cos(T2) * 8.7192501205E-6 + sin(
          T2) * 8.729104631600002E-5) + (sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(
          T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) + cos(
          T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) - pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 - pow(sin(T2), 2.0) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 - sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.6395464181E-5;
      M(3, 3) =
        cos(T2 * 2.0) * (-1.786979863E-5) + sin(T2 * 2.0) * 8.832120402999999E-9 + sin(
          T2) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) + (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + pow(
          cos(T2),
          2.0) * pow(
          cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
            T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + pow(sin(T2), 2.0) * pow(
          cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
            T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + cos(T2) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) + 5.6395464181E-5;
      M(3, 4) =
        -cos(T3 + T4 + T5) * (cos(T2) * 3.734584149999999E-7 - sin(T2) * 2.105636022000012E-6) - sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(T6) * 3.239E3) * (sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 - cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 - cos(T2) * sin(
          T3 + T4 + T5) * (cos(T2) * 8.7192501205E-6 + sin(T2) * 8.729104631600002E-5) - cos(
          T2) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T6) * sin(T2) * 3.239E-2 - sin(
          T2) * sin(T6) * 3.3E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * 1.7184E-6 + sin(T2) * sin(
          T3 + T4 + T5) * (cos(T2) * 3.5302413233E-4 + sin(T2) * 8.7192501205E-6) - sin(
          T2) * (cos(T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * (-3.239E-2) + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * 1.7184E-6;
      M(4, 0) =
        cos(T3 + T4 + T5) * 4.128464489040001E-4 - (cos(T2) * cos(T6) * (-5.5658976E-3) + cos(
          T2) * sin(T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) * (cos(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T6) * sin(T2) * 3.3E-3 + cos(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) + (cos(T6) * sin(T2) * 5.5658976E-3 - sin(
          T2) * sin(T6) * 5.67072E-4 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * (sin(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + cos(T2) * cos(T6) * 3.3E-3 + cos(
          T2) * sin(T6) * 3.239E-2 + sin(T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 3.239E-2 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 3.3E-3) - cos(T2) * sin(T3 + T4 + T5) * 2.105636022000012E-6 - sin(
          T2) * sin(T3 + T4 + T5) * 3.734584149999999E-7;
      M(4, 1) =
        -cos(T2) * (cos(T3 + T4 + T5) * 3.734584149999999E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.7192501205E-6 - sin(T2) * sin(T3 + T4 + T5) * 3.5302413233E-4) + sin(
          T2) * (cos(T3 + T4 + T5) * 2.105636022000012E-6 - cos(T2) * sin(
          T3 + T4 + T5) * 8.729104631600002E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.7192501205E-6) + sin(
          T2) * (cos(T2) * cos(T6) * (-5.5658976E-3) + cos(T2) * sin(T6) * 5.67072E-4 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.5658976E-3) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.0E-5 + cos(
          T2) * (cos(T6) * sin(T2) * 5.5658976E-3 - sin(T2) * sin(T6) * 5.67072E-4 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.0E-5 - sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.029E-1 + cos(
          T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(4, 2) =
        -cos(T2) * (cos(T3 + T4 + T5) * 3.734584149999999E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.7192501205E-6 - sin(T2) * sin(T3 + T4 + T5) * 3.5302413233E-4) + sin(
          T2) * (cos(T3 + T4 + T5) * 2.105636022000012E-6 - cos(T2) * sin(
          T3 + T4 + T5) * 8.729104631600002E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.7192501205E-6) + cos(
          T2) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T6) * sin(T2) * 5.5658976E-3 - sin(T2) * sin(T6) * 5.67072E-4 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * 1.0E-5 + sin(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 - sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * cos(T6) * (-5.5658976E-3) + cos(T2) * sin(T6) * 5.67072E-4 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.5658976E-3) * 1.0E-5 - sin(T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(
          T6) * 3.239E3) * (cos(T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(4, 3) =
        -cos(T2) * (cos(T3 + T4 + T5) * 3.734584149999999E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.7192501205E-6 - sin(T2) * sin(T3 + T4 + T5) * 3.5302413233E-4) + sin(
          T2) * (cos(T3 + T4 + T5) * 2.105636022000012E-6 - cos(T2) * sin(
          T3 + T4 + T5) * 8.729104631600002E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.7192501205E-6) - sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 + sin(T6) * 3.239E3) * (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 - cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6 - cos(T2) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T6) * sin(T2) * 5.5658976E-3 - sin(
          T2) * sin(T6) * 5.67072E-4 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * 1.0E-5 - sin(T2) * (cos(
          T3 + T4 + T5) * 2.542E3 - cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * (-5.5658976E-3) + cos(
          T2) * sin(T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) * 1.0E-5;
      M(4, 4) =
        -cos(T3 + T4 + T5) * (cos(T3 + T4 + T5) * (-4.128464489040001E-4) + cos(
          T2) * sin(T3 + T4 + T5) * 2.105636022000012E-6 + sin(T2) * sin(
          T3 + T4 + T5) * 3.734584149999999E-7) + (cos(T2) * cos(T6) * (-3.239E-2) + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * (cos(T2) * cos(T6) * (-5.5658976E-3) + cos(
          T2) * sin(T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) + (cos(T6) * sin(T2) * 3.239E-2 - sin(
          T2) * sin(T6) * 3.3E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * (cos(T6) * sin(T2) * 5.5658976E-3 - sin(
          T2) * sin(T6) * 5.67072E-4 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) + pow(
          sin(T3 + T4 + T5),
          2.0) * pow(cos(T6) * 3.3E2 + sin(T6) * 3.239E3, 2.0) * 1.7184E-11 - sin(
          T2) * sin(T3 + T4 + T5) * (cos(T3 + T4 + T5) * 3.734584149999999E-7 + cos(
          T2) * sin(T3 + T4 + T5) * 8.7192501205E-6 - sin(T2) * sin(
          T3 + T4 + T5) * 3.5302413233E-4) - cos(T2) * sin(T3 + T4 + T5) * (cos(
          T3 + T4 + T5) * 2.105636022000012E-6 - cos(T2) * sin(T3 + T4 + T5) * 8.729104631600002E-5 + sin(
          T2) * sin(T3 + T4 + T5) * 8.7192501205E-6);
    } else {
      M(0, 0) =
        cos(T3 * 2.0) * 1.23291505614E-3 - cos(T3 * 2.0 + T4 + T5) * 9.18996E-5 + sin(
          T3 * 2.0) * 2.8893758832E-5 + cos(T3 * 2.0 + T4 * 2.0) * 7.102705428E-4 + sin(
          T3 * 2.0 + T4 + T5) * 6.0372E-6 - cos(T4 + T5) * 9.18996E-5 + sin(
          T4 + T5) * 6.0372E-6 - cos(T3 * 2.0 + T4 * 2.0 + T5) * 9.45646884E-5 + sin(
          T3 * 2.0 + T4 * 2.0 + T5) * 6.2122788E-6 + cos(T4) * 1.3805064E-3 - cos(
          T5) * 9.45646884E-5 + sin(T5) * 6.2122788E-6 + cos(T3 * 2.0 + T4) * 1.3805064E-3 + cos(
          T3 * 2.0 + T4 * 2.0 + T5 * 2.0) * 3.1339776E-6 - sin(
          T3 * 2.0 + T4 * 2.0 + T5 * 2.0) * 4.135482E-7 + (sin(T2) * 6.78195E-4 + cos(
          T2) * cos(T3) * 3.0142E-2 + cos(T2) * cos(T3) * cos(T4) * 1.48780912E-2 + cos(
          T2) * cos(T3) * sin(T4) * 1.3654326E-3 + cos(T2) * cos(T4) * sin(T3) * 1.3654326E-3 - cos(
          T2) * sin(T3) * sin(T4) * 1.48780912E-2) * (sin(T2) * 2.25E-3 + cos(
          T2) * cos(T3) * (1.0 / 1.0E1) + cos(T2) * cos(T3) * cos(T4) * 4.936E-2 + cos(
          T2) * cos(T3) * sin(T4) * 4.53E-3 + cos(T2) * cos(T4) * sin(T3) * 4.53E-3 - cos(
          T2) * sin(T3) * sin(T4) * 4.936E-2) + (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T6) * sin(T2) * 3.3E-3 + cos(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.3E-3) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(
          T6) * sin(T2) * 5.67072E-4 + cos(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(
          T2) * sin(T6) * 5.5658976E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + (cos(T2) * (-6.78195E-4) + cos(
          T3) * sin(T2) * 3.0142E-2 + cos(T3) * cos(T4) * sin(T2) * 1.48780912E-2 + cos(
          T3) * sin(T2) * sin(T4) * 1.3654326E-3 + cos(T4) * sin(T2) * sin(T3) * 1.3654326E-3 - sin(
          T2) * sin(T3) * sin(T4) * 1.48780912E-2) * (cos(T2) * (-2.25E-3) + cos(
          T3) * sin(T2) * (1.0 / 1.0E1) + cos(T3) * cos(T4) * sin(T2) * 4.936E-2 + cos(
          T3) * sin(T2) * sin(T4) * 4.53E-3 + cos(T4) * sin(T2) * sin(T3) * 4.53E-3 - sin(
          T2) * sin(T3) * sin(T4) * 4.936E-2) + (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 - cos(T2) * cos(T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(
          T2) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) + 2.959188602983E-3;
      M(0, 1) =
        cos(T3 + T4) * 3.07222335E-6 - sin(T3 + T4) * 3.747917376E-5 + cos(T2) * 2.23583150366E-5 + cos(
          T3) * 1.188446064E-6 + cos(T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 5.276318449348842E-7 - sin(
          T3) * 1.17982029144E-4 + sin(T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T6) * sin(
          T2) * 5.67072E-4 + cos(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(
          T2) * sin(T6) * 5.5658976E-3 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 2) =
        cos(T3 + T4) * 3.07222335E-6 - sin(T3 + T4) * 3.747917376E-5 - cos(T2) * 3.46309360194E-5 + cos(
          T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 7.623262264934884E-6 + sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 + cos(T6) * sin(T2) * 5.67072E-4 + cos(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T2) * cos(
          T6) * 5.67072E-4 + cos(T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 3) =
        cos(T2) * 4.769916290599998E-6 + cos(T3 + T4 + T5) * 1.750788E-8 + sin(
          T2) * 2.622928711934887E-6 + sin(T3 + T4 + T5) * 2.6650884E-7 - sin(
          T2) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 + cos(T6) * sin(T2) * 5.67072E-4 + cos(T2) * sin(
          T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.7184E-5 - cos(T2) * cos(T6) * 5.67072E-4 + cos(T2) * sin(
          T6) * 5.5658976E-3 + sin(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(0, 4) =
        cos(T3 + T4 + T5) * 4.12124499284E-4 + (cos(T2) * cos(T6) * 3.239E-2 + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * (cos(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 + cos(T6) * sin(T2) * 5.67072E-4 + cos(
          T2) * sin(T3 + T4 + T5) * 4.3681728E-3 - sin(T2) * sin(T6) * 5.5658976E-3 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) + (cos(T6) * sin(T2) * 3.239E-2 + sin(
          T2) * sin(T6) * 3.3E-3 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * (sin(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.7184E-5 - cos(T2) * cos(T6) * 5.67072E-4 + cos(
          T2) * sin(T6) * 5.5658976E-3 + sin(T2) * sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.67072E-4) - cos(T2) * sin(T3 + T4 + T5) * 2.351839871999987E-6 + sin(
          T2) * sin(T3 + T4 + T5) * 4.817082939999985E-7;
      M(1, 0) =
        cos(T3 + T4) * 3.07222335E-6 - sin(T3 + T4) * 3.747917376E-5 + cos(T2) * 2.23583150366E-5 + cos(
          T3) * 1.188446064E-6 + cos(T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 5.276318449348842E-7 - sin(
          T3) * 1.17982029144E-4 + sin(T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + cos(T6) * sin(
          T2) * 3.3E-3 + cos(T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(
          T6) * 3.239E-2 + cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T2) * cos(
          T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.239E-2 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(1, 1) =
        cos(T2 * 2.0) * 3.376534562949997E-5 + sin(T2 * 2.0) * 5.949877313296E-6 - cos(
          T4 + T5) * 1.837992E-4 + sin(T4 + T5) * 1.20744E-5 + cos(T4) * 5.73663104E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 2.7308652E-4 + sin(T5) * 1.24245576E-5 + pow(
          cos(T2),
          2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
            T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) + pow(
          sin(T2),
          2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
            T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 - sin(T2) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (cos(
          T3 + T4) * 1.029E-1 + cos(T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + cos(T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + 8.6049097915105E-3;
      M(1, 2) =
        cos(T2 * 2.0) * 1.171186142749999E-5 + sin(T2 * 2.0) * 3.921891242006E-6 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 + cos(T4) * 2.86831552E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 1.3654326E-4 + sin(T5) * 1.24245576E-5 + cos(
          T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) - sin(T2) * (cos(
          T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (cos(
          T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + cos(T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + pow(
          cos(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 2.6348423209085E-3;
      M(1, 3) =
        cos(T2 * 2.0) * (-1.78934018365E-5) - sin(T2 * 2.0) * 4.683211999399999E-8 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 - cos(T5) * 9.45646884E-5 + sin(
          T5) * 6.2122788E-6 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) - sin(
          T2) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(T3 + T4) * 1.7682336E-2 + cos(
          T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.64174157445E-5;
      M(1, 4) =
        cos(T3 + T4 + T5) * (cos(T2) * 4.817082939999985E-7 + sin(T2) * 2.351839871999987E-6) + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) + sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.7682336E-2 + cos(
          T3) * 1.7184E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + cos(
          T2) * (cos(T6) * sin(T2) * 3.239E-2 + sin(T2) * sin(T6) * 3.3E-3 - cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 3.239E-2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-6 - sin(
          T2) * (cos(T2) * cos(T6) * 3.239E-2 + cos(T2) * sin(T6) * 3.3E-3 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 3.239E-2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-6;
      M(2, 0) =
        cos(T3 + T4) * 3.07222335E-6 - sin(T3 + T4) * 3.747917376E-5 - cos(T2) * 3.46309360194E-5 + cos(
          T3 + T4 + T5) * 1.750788E-8 + sin(T2) * 7.623262264934884E-6 + sin(
          T3 + T4 + T5) * 2.6650884E-7 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T6) * sin(T2) * 3.3E-3 + cos(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 - cos(T2) * cos(T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(
          T2) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(2, 1) =
        cos(T2 * 2.0) * 1.171186142749999E-5 + sin(T2 * 2.0) * 3.921891242006E-6 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 + cos(T4) * 2.86831552E-3 - cos(
          T5) * 1.891293768E-4 + sin(T4) * 1.3654326E-4 + sin(T5) * 1.24245576E-5 + cos(
          T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) - sin(T2) * (cos(
          T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(
          T3 + T4) * 1.029E-1 + cos(T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) + pow(
          cos(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 2.6348423209085E-3;
      M(2, 2) =
        cos(T2 * 2.0) * 1.171186142749999E-5 + sin(T2 * 2.0) * 3.921891242006E-6 - cos(
          T5) * 1.891293768E-4 + sin(T5) * 1.24245576E-5 + pow(cos(T2), 2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
            T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + pow(sin(T2), 2.0) * pow(
          sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
            T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) - sin(
          T2) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (cos(
          T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + 2.6348423209085E-3;
      M(2, 3) =
        cos(T2 * 2.0) * (-1.78934018365E-5) - sin(T2 * 2.0) * 4.683211999399999E-8 - cos(
          T5) * 9.45646884E-5 + sin(T5) * 6.2122788E-6 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(
          T2) * 8.5399200479E-6) - sin(T2) * (cos(T2) * 8.5399200479E-6 - sin(
          T2) * 8.715126061599998E-5) + (sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * (cos(
          T3 + T4) * 1.7682336E-2 + sin(T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(
          T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(sin(T2), 2.0) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.64174157445E-5;
      M(2, 4) =
        cos(T3 + T4 + T5) * (cos(T2) * 4.817082939999985E-7 + sin(T2) * 2.351839871999987E-6) + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) + cos(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + cos(
          T2) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T6) * sin(T2) * 3.239E-2 + sin(T2) * sin(T6) * 3.3E-3 - cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.239E-2) * 1.7184E-6 - sin(T2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * 3.239E-2 + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * 1.7184E-6 + sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.7682336E-2 + sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5;
      M(3, 0) =
        cos(T2) * 4.769916290599998E-6 + cos(T3 + T4 + T5) * 1.750788E-8 + sin(
          T2) * 2.622928711934887E-6 + sin(T3 + T4 + T5) * 2.6650884E-7 - sin(
          T2) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(
          T3) * 1.0E3) * 1.0E-4 + cos(T6) * sin(T2) * 3.3E-3 + cos(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(T2) * cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T2) * cos(T3 + T4 + T5) * sin(
          T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (cos(T3 + T4 + T5) * -2.542E3 + cos(
          T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(
          T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T2) * cos(
          T6) * 3.3E-3 + cos(T2) * sin(T6) * 3.239E-2 + sin(T2) * sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.239E-2 + cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(3, 1) =
        cos(T2 * 2.0) * (-1.78934018365E-5) - sin(T2 * 2.0) * 4.683211999399999E-8 - cos(
          T4 + T5) * 9.18996E-5 + sin(T4 + T5) * 6.0372E-6 - cos(T5) * 9.45646884E-5 + sin(
          T5) * 6.2122788E-6 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) - sin(
          T2) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + (sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(T3 + T4) * 1.029E-1 + cos(
          T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) + pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(
          sin(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.64174157445E-5;
      M(3, 2) =
        cos(T2 * 2.0) * (-1.78934018365E-5) - sin(T2 * 2.0) * 4.683211999399999E-8 - cos(
          T5) * 9.45646884E-5 + sin(T5) * 6.2122788E-6 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(
          T2) * 8.5399200479E-6) - sin(T2) * (cos(T2) * 8.5399200479E-6 - sin(
          T2) * 8.715126061599998E-5) + (sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) * (cos(
          T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) + pow(
          cos(T2),
          2.0) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + pow(sin(T2), 2.0) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.7184E-11 + 5.64174157445E-5;
      M(3, 3) =
        cos(T2 * 2.0) * (-1.78934018365E-5) - sin(T2 * 2.0) * 4.683211999399999E-8 + (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * (sin(T3 + T4 + T5) * 4.3681728E-3 + cos(
          T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(T3 + T4 + T5) * sin(T6) * 5.67072E-4) + pow(
          cos(T2),
          2.0) * pow(
          cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
            T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 + cos(T2) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) + pow(
          sin(T2),
          2.0) * pow(
          cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
            T6) * sin(T3 + T4 + T5) * 3.3E2,
          2.0) * 1.7184E-11 - sin(T2) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + 5.64174157445E-5;
      M(3, 4) =
        cos(T3 + T4 + T5) * (cos(T2) * 4.817082939999985E-7 + sin(T2) * 2.351839871999987E-6) + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T2) * 3.521554994E-4 - sin(T2) * 8.5399200479E-6) + sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(T6) * 3.239E3) * (sin(
          T3 + T4 + T5) * 4.3681728E-3 + cos(T6) * cos(T3 + T4 + T5) * 5.5658976E-3 + cos(
          T3 + T4 + T5) * sin(T6) * 5.67072E-4) * 1.0E-5 + cos(T2) * sin(
          T3 + T4 + T5) * (cos(T2) * 8.5399200479E-6 - sin(T2) * 8.715126061599998E-5) + cos(
          T2) * (cos(T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T6) * sin(T2) * 3.239E-2 + sin(
          T2) * sin(T6) * 3.3E-3 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * 1.7184E-6 - sin(T2) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * 3.239E-2 + cos(
          T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 3.3E-3 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 3.239E-2) * 1.7184E-6;
      M(4, 0) =
        cos(T3 + T4 + T5) * 4.12124499284E-4 + (cos(T2) * cos(T6) * 5.5658976E-3 + cos(
          T2) * sin(T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) * (cos(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + cos(T6) * sin(T2) * 3.3E-3 + cos(
          T2) * sin(T3 + T4 + T5) * 2.542E-2 - sin(T2) * sin(T6) * 3.239E-2 + cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) + (cos(T6) * sin(T2) * 5.5658976E-3 + sin(
          T2) * sin(T6) * 5.67072E-4 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * (sin(T2) * (cos(
          T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - cos(T2) * cos(T6) * 3.3E-3 + cos(
          T2) * sin(T6) * 3.239E-2 + sin(T2) * sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 3.239E-2 + cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 3.3E-3) - cos(T2) * sin(T3 + T4 + T5) * 2.351839871999987E-6 + sin(
          T2) * sin(T3 + T4 + T5) * 4.817082939999985E-7;
      M(4, 1) =
        cos(T2) * (cos(T3 + T4 + T5) * 4.817082939999985E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.5399200479E-6 + sin(T2) * sin(T3 + T4 + T5) * 3.521554994E-4) - sin(
          T2) * (cos(T3 + T4 + T5) * (-2.351839871999987E-6) + cos(T2) * sin(
          T3 + T4 + T5) * 8.715126061599998E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.5399200479E-6) - sin(
          T2) * (cos(T2) * cos(T6) * 5.5658976E-3 + cos(T2) * sin(T6) * 5.67072E-4 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.5658976E-3) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.0E-5 + cos(
          T2) * (cos(T6) * sin(T2) * 5.5658976E-3 + sin(T2) * sin(T6) * 5.67072E-4 - cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * (sin(T3 + T4) * 1.029E4 - cos(
          T3 + T4 + T5) * 2.542E3 + sin(T3) * 1.0E4 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * 1.0E-5 + sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(T6) * 3.239E3) * (cos(T3 + T4) * 1.029E-1 + cos(
          T3) * (1.0 / 1.0E1) + sin(T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(
          T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(4, 2) =
        cos(T2) * (cos(T3 + T4 + T5) * 4.817082939999985E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.5399200479E-6 + sin(T2) * sin(T3 + T4 + T5) * 3.521554994E-4) - sin(
          T2) * (cos(T3 + T4 + T5) * (-2.351839871999987E-6) + cos(T2) * sin(
          T3 + T4 + T5) * 8.715126061599998E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.5399200479E-6) + cos(
          T2) * (sin(T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T6) * sin(T2) * 5.5658976E-3 + sin(T2) * sin(T6) * 5.67072E-4 - cos(
          T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(T2) * cos(
          T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * 1.0E-5 - sin(T2) * (sin(
          T3 + T4) * 1.029E4 - cos(T3 + T4 + T5) * 2.542E3 + cos(T6) * sin(
          T3 + T4 + T5) * 3.239E3 + sin(T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(
          T2) * cos(T6) * 5.5658976E-3 + cos(T2) * sin(T6) * 5.67072E-4 + cos(
          T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 - cos(T3 + T4 + T5) * sin(
          T2) * sin(T6) * 5.5658976E-3) * 1.0E-5 + sin(T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(
          T6) * 3.239E3) * (cos(T3 + T4) * 1.029E-1 + sin(T3 + T4 + T5) * 2.542E-2 + cos(
          T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6;
      M(4, 3) =
        cos(T2) * (cos(T3 + T4 + T5) * 4.817082939999985E-7 + cos(T2) * sin(
          T3 + T4 + T5) * 8.5399200479E-6 + sin(T2) * sin(T3 + T4 + T5) * 3.521554994E-4) - sin(
          T2) * (cos(T3 + T4 + T5) * (-2.351839871999987E-6) + cos(T2) * sin(
          T3 + T4 + T5) * 8.715126061599998E-5 + sin(T2) * sin(T3 + T4 + T5) * 8.5399200479E-6) + sin(
          T3 + T4 + T5) * (cos(T6) * 3.3E2 - sin(T6) * 3.239E3) * (sin(
          T3 + T4 + T5) * 2.542E-2 + cos(T6) * cos(T3 + T4 + T5) * 3.239E-2 + cos(
          T3 + T4 + T5) * sin(T6) * 3.3E-3) * 1.7184E-6 + cos(T2) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T6) * sin(T2) * 5.5658976E-3 + sin(
          T2) * sin(T6) * 5.67072E-4 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) * 1.0E-5 - sin(T2) * (cos(
          T3 + T4 + T5) * -2.542E3 + cos(T6) * sin(T3 + T4 + T5) * 3.239E3 + sin(
          T6) * sin(T3 + T4 + T5) * 3.3E2) * (cos(T2) * cos(T6) * 5.5658976E-3 + cos(
          T2) * sin(T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) * 1.0E-5;
      M(4, 4) =
        cos(T3 + T4 + T5) * (cos(T3 + T4 + T5) * 4.12124499284E-4 - cos(T2) * sin(
          T3 + T4 + T5) * 2.351839871999987E-6 + sin(T2) * sin(T3 + T4 + T5) * 4.817082939999985E-7) + (cos(
          T2) * cos(T6) * 3.239E-2 + cos(T2) * sin(T6) * 3.3E-3 + cos(T6) * cos(
          T3 + T4 + T5) * sin(T2) * 3.3E-3 - cos(T3 + T4 + T5) * sin(T2) * sin(
          T6) * 3.239E-2) * (cos(T2) * cos(T6) * 5.5658976E-3 + cos(T2) * sin(
          T6) * 5.67072E-4 + cos(T6) * cos(T3 + T4 + T5) * sin(T2) * 5.67072E-4 - cos(
          T3 + T4 + T5) * sin(T2) * sin(T6) * 5.5658976E-3) + (cos(T6) * sin(T2) * 3.239E-2 + sin(
          T2) * sin(T6) * 3.3E-3 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 3.3E-3 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 3.239E-2) * (cos(T6) * sin(T2) * 5.5658976E-3 + sin(
          T2) * sin(T6) * 5.67072E-4 - cos(T2) * cos(T6) * cos(T3 + T4 + T5) * 5.67072E-4 + cos(
          T2) * cos(T3 + T4 + T5) * sin(T6) * 5.5658976E-3) + pow(
          sin(T3 + T4 + T5),
          2.0) * pow(cos(T6) * 3.3E2 - sin(T6) * 3.239E3, 2.0) * 1.7184E-11 + sin(
          T2) * sin(T3 + T4 + T5) * (cos(T3 + T4 + T5) * 4.817082939999985E-7 + cos(
          T2) * sin(T3 + T4 + T5) * 8.5399200479E-6 + sin(T2) * sin(
          T3 + T4 + T5) * 3.521554994E-4) + cos(T2) * sin(T3 + T4 + T5) * (cos(
          T3 + T4 + T5) * (-2.351839871999987E-6) + cos(T2) * sin(T3 + T4 + T5) * 8.715126061599998E-5 + sin(
          T2) * sin(T3 + T4 + T5) * 8.5399200479E-6);
    }
    return M;
  }

}
#endif
