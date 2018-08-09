#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

#ifndef EE_JACOBIAN2_H
#define EE_JACOBIAN2_H

namespace KickUtils
{
  inline MatrixXf
  EEJacobian2(const unsigned& legIndex, const Vector3f& EE,
    const VectorXf& jointAngles)
  {
    Matrix<float, 3, 5> J6V;
    Matrix<float, 3, 5> J6W;
    Matrix<float, 6, 5> J;
    float x, y, z, T2, T3, T4, T5, T6;
    T2 = jointAngles[0];
    T3 = jointAngles[1];
    T4 = jointAngles[2];
    T5 = jointAngles[3];
    T6 = jointAngles[4];
    if (legIndex == CHAIN_L_LEG) {
      x = 0.02542;
      y = 0.003300;
      z = -0.03239;

      J6V(0, 0) = 0;

      J6V(0, 1) =
        cos(T3 + T4) * (-1.029E-1) - cos(T3) * (1.0 / 1.0E1) - x * sin(
          T3 + T4 + T5) + z * cos(T6) * cos(T3 + T4 + T5) + y * cos(
          T3 + T4 + T5) * sin(T6);

      J6V(0, 2) = cos(T3 + T4) * (-1.029E-1) - x * sin(T3 + T4 + T5) + z * cos(
        T6) * cos(T3 + T4 + T5) + y * cos(T3 + T4 + T5) * sin(T6);

      J6V(0, 3) =
        -x * sin(T3 + T4 + T5) + z * cos(T6) * cos(T3 + T4 + T5) + y * cos(
          T3 + T4 + T5) * sin(T6);

      J6V(0, 4) = sin(T3 + T4 + T5) * (y * cos(T6) - z * sin(T6));

      J6V(1, 0) =
        cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - y * (cos(
          T6) * sin(T2) + cos(T2) * cos(T3 + T4 + T5) * sin(T6)) + z * (sin(T2) * sin(
          T6) - cos(T2) * cos(T6) * cos(T3 + T4 + T5)) + x * cos(T2) * sin(
          T3 + T4 + T5);

      J6V(1, 1) =
        sin(T2) * (sin(T3 + T4) * -1.029E3 - sin(T3) * 1.0E3 + x * cos(
          T3 + T4 + T5) * 1.0E4 + z * cos(T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(
          T6) * sin(T3 + T4 + T5) * 1.0E4) * 1.0E-4;

      J6V(1, 2) =
        sin(T2) * (sin(T3 + T4) * -1.029E3 + x * cos(T3 + T4 + T5) * 1.0E4 + z * cos(
          T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(T6) * sin(T3 + T4 + T5) * 1.0E4) * 1.0E-4;

      J6V(1, 3) = sin(T2) * (x * cos(T3 + T4 + T5) + z * cos(T6) * sin(
        T3 + T4 + T5) + y * sin(T6) * sin(T3 + T4 + T5));

      J6V(1, 4) =
        -z * cos(T2) * cos(T6) - y * cos(T2) * sin(T6) - y * cos(T6) * cos(
          T3 + T4 + T5) * sin(T2) + z * cos(T3 + T4 + T5) * sin(T2) * sin(T6);

      J6V(2, 0) =
        sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + y * (cos(
          T2) * cos(T6) - cos(T3 + T4 + T5) * sin(T2) * sin(T6)) - z * (cos(T2) * sin(
          T6) + cos(T6) * cos(T3 + T4 + T5) * sin(T2)) + x * sin(T2) * sin(
          T3 + T4 + T5);

      J6V(2, 1) =
        cos(T2) * (sin(T3 + T4) * -1.029E3 - sin(T3) * 1.0E3 + x * cos(
          T3 + T4 + T5) * 1.0E4 + z * cos(T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(
          T6) * sin(T3 + T4 + T5) * 1.0E4) * (-1.0E-4);

      J6V(2, 2) =
        cos(T2) * (sin(T3 + T4) * -1.029E3 + x * cos(T3 + T4 + T5) * 1.0E4 + z * cos(
          T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(T6) * sin(T3 + T4 + T5) * 1.0E4) * (-1.0E-4);

      J6V(2, 3) = -cos(T2) * (x * cos(T3 + T4 + T5) + z * cos(T6) * sin(
        T3 + T4 + T5) + y * sin(T6) * sin(T3 + T4 + T5));

      J6V(2, 4) =
        -z * cos(T6) * sin(T2) - y * sin(T2) * sin(T6) + y * cos(T2) * cos(T6) * cos(
          T3 + T4 + T5) - z * cos(T2) * cos(T3 + T4 + T5) * sin(T6);

      J6W(0, 0) = 1.0;
      J6W(0, 1) = -cos(T3);
      J6W(0, 2) = 0;
      J6W(0, 3) = 0;
      J6W(0, 4) = cos(T3 + T4 + T5);
      J6W(1, 0) = 0;
      J6W(1, 1) = 0;
      J6W(1, 2) = cos(T2);
      J6W(1, 3) = cos(T2);
      J6W(1, 4) = sin(T2) * sin(T3 + T4 + T5);
      J6W(2, 0) = 0;
      J6W(2, 1) = -sin(T3);
      J6W(2, 2) = sin(T2);
      J6W(2, 3) = sin(T2);
      J6W(2, 4) = -cos(T2) * sin(T3 + T4 + T5);

      J << J6V, J6W;
    } else {
      x = 0.02540;
      y = -0.00332;
      z = -0.03239;
      J6V(0, 0) = 0;
      J6V(0, 1) =
        cos(T3 + T4) * (-1.029E-1) - cos(T3) * (1.0 / 1.0E1) - x * sin(
          T3 + T4 + T5) + z * cos(T6) * cos(T3 + T4 + T5) + y * cos(
          T3 + T4 + T5) * sin(T6);
      J6V(0, 2) = cos(T3 + T4) * (-1.029E-1) - x * sin(T3 + T4 + T5) + z * cos(
        T6) * cos(T3 + T4 + T5) + y * cos(T3 + T4 + T5) * sin(T6);
      J6V(0, 3) =
        -x * sin(T3 + T4 + T5) + z * cos(T6) * cos(T3 + T4 + T5) + y * cos(
          T3 + T4 + T5) * sin(T6);
      J6V(0, 4) = sin(T3 + T4 + T5) * (y * cos(T6) - z * sin(T6));
      J6V(1, 0) =
        cos(T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 - y * (cos(
          T6) * sin(T2) + cos(T2) * cos(T3 + T4 + T5) * sin(T6)) + z * (sin(T2) * sin(
          T6) - cos(T2) * cos(T6) * cos(T3 + T4 + T5)) + x * cos(T2) * sin(
          T3 + T4 + T5);
      J6V(1, 1) =
        sin(T2) * (sin(T3 + T4) * -1.029E3 - sin(T3) * 1.0E3 + x * cos(
          T3 + T4 + T5) * 1.0E4 + z * cos(T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(
          T6) * sin(T3 + T4 + T5) * 1.0E4) * 1.0E-4;
      J6V(1, 2) =
        sin(T2) * (sin(T3 + T4) * -1.029E3 + x * cos(T3 + T4 + T5) * 1.0E4 + z * cos(
          T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(T6) * sin(T3 + T4 + T5) * 1.0E4) * 1.0E-4;
      J6V(1, 3) = sin(T2) * (x * cos(T3 + T4 + T5) + z * cos(T6) * sin(
        T3 + T4 + T5) + y * sin(T6) * sin(T3 + T4 + T5));
      J6V(1, 4) = -y * (cos(T2) * sin(T6) + cos(T6) * cos(T3 + T4 + T5) * sin(
        T2)) - z * (cos(T2) * cos(T6) - cos(T3 + T4 + T5) * sin(T2) * sin(T6));
      J6V(2, 0) =
        sin(T2) * (cos(T3 + T4) * 1.029E3 + cos(T3) * 1.0E3) * 1.0E-4 + y * (cos(
          T2) * cos(T6) - cos(T3 + T4 + T5) * sin(T2) * sin(T6)) - z * (cos(T2) * sin(
          T6) + cos(T6) * cos(T3 + T4 + T5) * sin(T2)) + x * sin(T2) * sin(
          T3 + T4 + T5);
      J6V(2, 1) =
        cos(T2) * (sin(T3 + T4) * -1.029E3 - sin(T3) * 1.0E3 + x * cos(
          T3 + T4 + T5) * 1.0E4 + z * cos(T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(
          T6) * sin(T3 + T4 + T5) * 1.0E4) * (-1.0E-4);
      J6V(2, 2) =
        cos(T2) * (sin(T3 + T4) * -1.029E3 + x * cos(T3 + T4 + T5) * 1.0E4 + z * cos(
          T6) * sin(T3 + T4 + T5) * 1.0E4 + y * sin(T6) * sin(T3 + T4 + T5) * 1.0E4) * (-1.0E-4);
      J6V(2, 3) = -cos(T2) * (x * cos(T3 + T4 + T5) + z * cos(T6) * sin(
        T3 + T4 + T5) + y * sin(T6) * sin(T3 + T4 + T5));
      J6V(2, 4) =
        -y * (sin(T2) * sin(T6) - cos(T2) * cos(T6) * cos(T3 + T4 + T5)) - z * (cos(
          T6) * sin(T2) + cos(T2) * cos(T3 + T4 + T5) * sin(T6));

      J6W(0, 0) = 1.0;
      J6W(0, 1) = 0;
      J6W(0, 2) = 0;
      J6W(0, 3) = 0;
      J6W(0, 4) = cos(T3 + T4 + T5);
      J6W(1, 0) = 0;
      J6W(1, 1) = cos(T2);
      J6W(1, 2) = cos(T2);
      J6W(1, 3) = cos(T2);
      J6W(1, 4) = sin(T2) * sin(T3 + T4 + T5);
      J6W(2, 0) = 0;
      J6W(2, 1) = sin(T2);
      J6W(2, 2) = sin(T2);
      J6W(2, 3) = sin(T2);
      J6W(2, 4) = -cos(T2) * sin(T3 + T4 + T5);
      J << J6V, J6W;
    }
    return J;
  }
}
#endif
