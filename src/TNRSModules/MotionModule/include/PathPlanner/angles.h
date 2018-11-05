/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <algorithm>
#include <cmath>

namespace angles
{

  /*!
   * \brief Convert degrees to radians
   */

  static inline double
  fromDegrees(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  /*!
   * \brief Convert radians to degrees
   */
  static inline double
  toDegrees(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  /*!
   * \brief normalizeAnglePositive
   *
   *        Normalizes the angle to be 0 to 2*M_PI
   *        It takes and returns radians.
   */
  static inline double
  normalizeAnglePositive(double angle)
  {
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  }

  /*!
   * \brief normalize
   *
   * Normalizes the angle to be -M_PI circle to +M_PI circle
   * It takes and returns radians.
   *
   */
  static inline double
  normalizeAngle(double angle)
  {
    double a = normalizeAnglePositive(angle);
    if (a > M_PI) a -= 2.0 * M_PI;
    return a;
  }

  /*!
   * \function
   * \brief shortestAngularDistance
   *
   * Given 2 angles, this returns the shortest angular
   * difference.  The inputs and ouputs are of course radians.
   *
   * The result
   * would always be -pi <= result <= pi.  Adding the result
   * to "from" will always get you an equivelent angle to "to".
   */

  static inline double
  shortestAngularDistance(double from, double to)
  {
    return normalizeAngle(to - from);
  }

  /*!
   * \function
   *
   * \brief returns the angle in [-2*M_PI, 2*M_PI]  going the other way along the unit circle.
   * \param angle The angle to which you want to turn in the range [-2*M_PI, 2*M_PI]
   * E.g. twoPiComplement(-M_PI/4) returns 7_M_PI/4
   * twoPiComplement(M_PI/4) returns -7*M_PI/4
   *
   */
  static inline double
  twoPiComplement(double angle)
  {
    //check input conditions
    if (angle > 2 * M_PI || angle < -2.0 * M_PI) angle = fmod(
      angle,
      2.0 * M_PI);
    if (angle < 0) return (2 * M_PI + angle);
    else if (angle > 0) return (-2 * M_PI + angle);

    return (2 * M_PI);
  }

  /*!
   * \function
   *
   * \brief This function is only intended for internal use and not intended for external use. If you do use it, read the documentation very carefully. Returns the min and max amount (in radians) that can be moved from "from" angle to "leftLimit" and "rightLimit".
   * \return returns false if "from" angle does not lie in the interval [leftLimit,rightLimit]
   * \param from - "from" angle - must lie in [-M_PI, M_PI)
   * \param leftLimit - left limit of valid interval for angular position - must lie in [-M_PI, M_PI], left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
   * \param rightLimit - right limit of valid interval for angular position - must lie in [-M_PI, M_PI], left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
   * \param resultMinDelta - minimum (delta) angle (in radians) that can be moved from "from" position before hitting the joint stop
   * \param resultMaxDelta - maximum (delta) angle (in radians) that can be movedd from "from" position before hitting the joint stop
   */
  static bool
  findMinMaxDelta(double from, double leftLimit, double rightLimit,
    double &resultMinDelta, double &resultMaxDelta)
  {
    double delta[4];

    delta[0] = shortestAngularDistance(from, leftLimit);
    delta[1] = shortestAngularDistance(from, rightLimit);

    delta[2] = twoPiComplement(delta[0]);
    delta[3] = twoPiComplement(delta[1]);

    if (delta[0] == 0) {
      resultMinDelta = delta[0];
      resultMaxDelta = std::max<double>(delta[1], delta[3]);
      return true;
    }

    if (delta[1] == 0) {
      resultMaxDelta = delta[1];
      resultMinDelta = std::min<double>(delta[0], delta[2]);
      return true;
    }

    double deltaMin = delta[0];
    double deltaMin2pi = delta[2];
    if (delta[2] < deltaMin) {
      deltaMin = delta[2];
      deltaMin2pi = delta[0];
    }

    double deltaMax = delta[1];
    double deltaMax2pi = delta[3];
    if (delta[3] > deltaMax) {
      deltaMax = delta[3];
      deltaMax2pi = delta[1];
    }

    //    printf("%f %f %f %f\n",deltaMin,deltaMin2pi,deltaMax,deltaMax2pi);
    if ((deltaMin <= deltaMax2pi) || (deltaMax >= deltaMin2pi)) {
      resultMinDelta = deltaMax2pi;
      resultMaxDelta = deltaMin2pi;
      if (leftLimit == -M_PI && rightLimit == M_PI) return true;
      else return false;
    }
    resultMinDelta = deltaMin;
    resultMaxDelta = deltaMax;
    return true;
  }

  /*!
   * \function
   *
   * \brief Returns the delta from "fromAngle" to "toAngle" making sure it does not violate limits specified by leftLimit and rightLimit.
   * The valid interval of angular positions is [leftLimit,rightLimit]. E.g., [-0.25,0.25] is a 0.5 radians wide interval that contains 0.
   * But [0.25,-0.25] is a 2*M_PI-0.5 wide interval that contains M_PI (but not 0).
   * The value of shortestAngle is the angular difference between "from" and "to" that lies within the defined valid interval.
   * E.g. shortestAngularDistanceWithLimits(-0.5,0.5,0.25,-0.25,ss) evaluates ss to 2*M_PI-1.0 and returns true while
   * shortestAngularDistanceWithLimits(-0.5,0.5,-0.25,0.25,ss) returns false since -0.5 and 0.5 do not lie in the interval [-0.25,0.25]
   *
   * \return true if "from" and "to" positions are within the limit interval, false otherwise
   * \param from - "from" angle
   * \param to - "to" angle
   * \param leftLimit - left limit of valid interval for angular position, left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
   * \param rightLimit - right limit of valid interval for angular position, left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
   * \param shortestAngle - result of the shortest angle calculation
   */
  static inline bool
  shortestAngularDistanceWithLimits(double from, double to, double leftLimit,
    double rightLimit, double &shortestAngle)
  {

    double minDelta = -2 * M_PI;
    double maxDelta = 2 * M_PI;
    double minDeltaTo = -2 * M_PI;
    double maxDeltaTo = 2 * M_PI;
    bool flag = findMinMaxDelta(
      from,
      leftLimit,
      rightLimit,
      minDelta,
      maxDelta);
    double delta = shortestAngularDistance(from, to);
    double deltaMod2pi = twoPiComplement(delta);

    if (flag) //from position is within the limits
    {
      if (delta >= minDelta && delta <= maxDelta) {
        shortestAngle = delta;
        return true;
      } else if (deltaMod2pi >= minDelta && deltaMod2pi <= maxDelta) {
        shortestAngle = deltaMod2pi;
        return true;
      } else //to position is outside the limits
      {
        findMinMaxDelta(to, leftLimit, rightLimit, minDeltaTo, maxDeltaTo);
        if (fabs(minDeltaTo) < fabs(maxDeltaTo)) shortestAngle =
          std::max<double>(delta, deltaMod2pi);
        else if (fabs(minDeltaTo) > fabs(maxDeltaTo)) shortestAngle = std::min<
          double>(delta, deltaMod2pi);
        else {
          if (fabs(delta) < fabs(deltaMod2pi)) shortestAngle = delta;
          else shortestAngle = deltaMod2pi;
        }
        return false;
      }
    } else // from position is outside the limits
    {
      findMinMaxDelta(to, leftLimit, rightLimit, minDeltaTo, maxDeltaTo);

      if (fabs(minDelta) < fabs(maxDelta)) shortestAngle = std::min<double>(
        delta,
        deltaMod2pi);
      else if (fabs(minDelta) > fabs(maxDelta)) shortestAngle =
        std::max<double>(delta, deltaMod2pi);
      else {
        if (fabs(delta) < fabs(deltaMod2pi)) shortestAngle = delta;
        else shortestAngle = deltaMod2pi;
      }
      return false;
    }

    shortestAngle = delta;
    return false;
  }
}
