/**
 * @file MotionModule/KickModule/KickUtils.h
 *
 * This file declares the utility functions used in KickModule.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017  
 */

#include "MassMatrix.h"
#include "EEJacobian.h"
#include "EEJacobian2.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/GnuPlotter.h"
#include <boost/algorithm/string.hpp>

#pragma once

namespace KickUtils
{
  inline Matrix4f
  rotX(const float& theta)
  {
    Matrix4f t;
    MathsUtils::makeRotationX(t, theta);
    return t;
  }

  inline Matrix4f
  rotY(const float& theta)
  {
    Matrix4f t;
    MathsUtils::makeRotationY(t, theta);
    return t;
  }

  inline Matrix4f
  rotZ(const float& theta)
  {
    Matrix4f t;
    MathsUtils::makeRotationZ(t, theta);
    return t;
  }

  inline bool
  fileEmpty(fstream& log)
  {
    char ch;
    while (log.get(ch)) {
      if (!std::isspace(ch)) return false;
    }
    return true;
  }

  inline bool
  checkFootContourLog(const string& logPath)
  {
    try {
      PRINT("Checking log: " << logPath)
      fstream log;
      log.open(logPath, std::fstream::in);

      if (fileEmpty(log)) {
        log.close();
        log.open(logPath, std::fstream::out | std::fstream::trunc);
        log << "# t X Y Z tX tY tZ" << endl;
        log.close();
        return true;
      }
      return false;
    } catch (const std::exception& e) {
      ERROR("Exception in KickUtils::checkFootContourLog():" << e.what())
      return false;
    }
  }

  inline void
  logContour(const float contour[][3], const unsigned& size,
    const string& logPath, const float& yOffset)
  {
    Matrix4f bezierMat;
    Matrix<float, 4, 3> contourMat;
    bezierMat << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
    Vector4f tVector;
    try {
      fstream log;
      log.open(logPath, std::fstream::out | std::fstream::app);
      size_t nCurves = size / 4;
      float start = 0.f;
      for (size_t i = 0; i < nCurves; ++i) {
        contourMat << contour[0 + 4 * i][0], contour[0 + 4 * i][1], contour[0 + 4 * i][2], contour[1 + 4 * i][0], contour[1 + 4 * i][1], contour[1 + 4 * i][2], contour[2 + 4 * i][0], contour[2 + 4 * i][1], contour[2 + 4 * i][2], contour[3 + 4 * i][0], contour[3 + 4 * i][1], contour[3 + 4 * i][2];
        contourMat = bezierMat * contourMat;
        for (float t = start; t <= 1.025f; t = t + 0.05f) {
          tVector(0, 0) = 1;
          tVector(1, 0) = t;
          tVector(2, 0) = t * t;
          tVector(3, 0) = tVector(2, 0) * t;
          Vector4f tDerivative;
          tDerivative(0, 0) = 0;
          tDerivative(1, 0) = 1;
          tDerivative(2, 0) = 2 * t;
          tDerivative(3, 0) = 3 * t * t;
          Vector3f point = (tVector.transpose() * contourMat).transpose();
          Vector3f tangent = (tDerivative.transpose() * contourMat).transpose();
          point[0] = fabsf(point[0]) < 1e-5 ? 0 : point[0];
          point[1] = fabsf(point[1]) < 1e-5 ? 0 : point[1];
          point[2] = fabsf(point[2]) < 1e-5 ? 0 : point[2];
          log << t << " " << point[0] << " " << point[1] + yOffset << " " << point[2] << " " << tangent[0] << " " << tangent[1] << " " << tangent[2] << endl;
        }
        start += 0.05f;
      }
      log.close();
    } catch (const std::exception& e) {
      ERROR("Exception in KickUtils::logContour():" << e.what())
    }
  }

  inline unsigned
  countLines(fstream& file)
  {
    string line;
    unsigned lines = 0;
    while (getline(file, line))
      ++lines;
    file.clear();
    file.seekg(0, ios::beg);
    return lines;
  }

  inline void
  drawArrow3D(Gnuplot& gp, const Vector3f& from, const Vector3f& to)
  {
    gp << "set arrow from " << from[0] << "," << from[1] << "," << from[2] << " to " << to[0] << "," << to[1] << "," << to[2] << " as 1 lc rgb 'blue'\n"; // Support frame

  }

  inline void
  drawFrame3D(Gnuplot& gp, Vector3f pos = Vector3f::Zero(), Vector3f rot =
    Vector3f::Zero())
  {
    const float arrowSize = 0.025;
    Vector3f to;
    Matrix3f rMat;
    MathsUtils::makeRotationXYZ(rMat, rot[0], rot[1], rot[2]);
    for (int i = 0; i < pos.size(); ++i) {
      to = pos;
      to[i] += arrowSize;
      to = rMat * to;
      drawArrow3D(gp, pos, to);
    }
  }
}
