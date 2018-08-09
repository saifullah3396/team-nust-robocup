/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MotionModule/include/PathPlanner/Footstep.h"

namespace PathPlannerSpace
{

  Footstep::Footstep(double x, double y, double theta, double cellSize,
    int numAngleBins, int maxHashSize) :
    theta(angleState2Cell(theta, numAngleBins)), cellSize(cellSize),
      numAngleBins(numAngleBins), maxHashSize(maxHashSize),
      discSuccessorLeft(numAngleBins), discSuccessorRight(numAngleBins),
      discPredecessorLeft(numAngleBins), discPredecessorRight(numAngleBins)
  {
    init(x, y);
  }

  void
  Footstep::init(double x, double y)
  {
    int backwardAngle;
    int footstepX;
    int footstepY;

    for (int a = 0; a < numAngleBins; ++a) {
      backwardAngle = calculateForwardStep(
        RIGHT,
        a,
        x,
        y,
        &footstepX,
        &footstepY);
      discSuccessorRight[a] = footstepXY(footstepX, footstepY);
      discPredecessorLeft[backwardAngle] = footstepXY(-footstepX, -footstepY);
      backwardAngle = calculateForwardStep(
        LEFT,
        a,
        x,
        y,
        &footstepX,
        &footstepY);
      discSuccessorLeft[a] = footstepXY(footstepX, footstepY);
      discPredecessorRight[backwardAngle] = footstepXY(-footstepX, -footstepY);
    }
  }

  PlanningState
  Footstep::performMeOnThisState(const PlanningState& current) const
  {
    Leg leg;

    int x = current.getX();
    int y = current.getY();
    int theta = current.getTheta();

    if (current.getLeg() == RIGHT) {
      footstepXY xy = discSuccessorRight[theta];
      x += xy.first;
      y += xy.second;
      theta += this->theta;
      leg = LEFT;
    } else // leg == LEFT
    {
      footstepXY xy = discSuccessorLeft[theta];
      x += xy.first;
      y += xy.second;
      theta -= this->theta;
      leg = RIGHT;
    }

    // theta has to be in [0..numAngleBins)
    if (theta < 0) theta += numAngleBins;
    else if (theta >= numAngleBins) theta -= numAngleBins;

    return PlanningState(x, y, theta, leg, maxHashSize);
  }

  PlanningState
  Footstep::reverseMeOnThisState(const PlanningState& current) const
  {
    Leg leg;

    int x = current.getX();
    int y = current.getY();
    int theta = current.getTheta();

    if (current.getLeg() == LEFT) {
      footstepXY xy = discPredecessorLeft[theta];
      x += xy.first;
      y += xy.second;
      theta -= this->theta;
      leg = RIGHT;
    } else // leg == RIGHT
    {
      footstepXY xy = discPredecessorRight[theta];
      x += xy.first;
      y += xy.second;
      theta += this->theta;
      leg = LEFT;
    }
    // theta has to be in [0..numAngleBins)
    if (theta < 0) theta += numAngleBins;
    else if (theta >= numAngleBins) theta -= numAngleBins;

    return PlanningState(x, y, theta, leg, maxHashSize);
  }

  int
  Footstep::calculateForwardStep(Leg leg, int globalTheta, double x, double y,
    int* footstepX, int* footstepY) const
  {
    double contFootstepX, contFootstepY;
    double contGlobalTheta = angleCell2State(globalTheta, numAngleBins);
    double thetaCos = cos(contGlobalTheta);
    double thetaSin = sin(contGlobalTheta);
    if (leg == RIGHT) {
      contFootstepX = thetaCos * x - thetaSin * y;
      contFootstepY = thetaSin * x + thetaCos * y;

      globalTheta += this->theta;
    } else // leg == LEFT
    {
      contFootstepX = thetaCos * x + thetaSin * y;
      contFootstepY = thetaSin * x - thetaCos * y;

      globalTheta -= this->theta;
    }
    *footstepX = discVal(contFootstepX, cellSize);
    *footstepY = discVal(contFootstepY, cellSize);

    // theta has to be in [0..numAngleBins)
    if (globalTheta < 0) globalTheta += numAngleBins;
    else if (globalTheta >= numAngleBins) globalTheta -= numAngleBins;
    return globalTheta;
  }

}
;
