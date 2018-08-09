/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstepPlanner
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

#include "MotionModule/include/PathPlanner/MyHeuristic.h"

namespace PathPlannerSpace
{

  MyHeuristic::MyHeuristic(double cellSize, int numAngleBins,
    MyHeuristicType type) :
    cellSize(cellSize), numAngleBins(numAngleBins), myHeuristicType(type)
  {
  }

  MyHeuristic::~MyHeuristic()
  {
  }

  EuclideanHeuristic::EuclideanHeuristic(double cellSize, int numAngleBins) :
    MyHeuristic(cellSize, numAngleBins, EUCLIDEAN)
  {
  }

  EuclideanHeuristic::~EuclideanHeuristic()
  {
  }

  double
  EuclideanHeuristic::getHValue(const PlanningState& from,
    const PlanningState& to) const
  {
    if (from == to) return 0.0;

    // distance in cell size
    double dist = euclideanDistance(
      from.getX(),
      from.getY(),
      to.getX(),
      to.getY());
    // return distance in meter
    return contVal(dist, cellSize);
  }

  EuclStepCostHeuristic::EuclStepCostHeuristic(double cellSize,
    int numAngleBins, double stepCost, double diffAngleCost,
    double maxStepWidth) :
    MyHeuristic(cellSize, numAngleBins, EUCLIDEAN_STEPCOST), stepCost(stepCost),
      diffAngleCost(diffAngleCost), maxStepWidth(maxStepWidth)
  {
  }

  EuclStepCostHeuristic::~EuclStepCostHeuristic()
  {
  }

  double
  EuclStepCostHeuristic::getHValue(const PlanningState& from,
    const PlanningState& to) const
  {
    if (from == to) return 0.0;

    // distance in meter
    double dist = contVal(
      euclideanDistance(from.getX(), from.getY(), to.getX(), to.getY()),
      cellSize);
    double expectedSteps = dist / maxStepWidth;
    double diffAngle = 0.0;
    if (diffAngleCost > 0.0) {
      // get the number of bins between from.theta and to.theta
      int diffAngleDisc =
        (((to.getTheta() - from.getTheta()) % numAngleBins) + numAngleBins) % numAngleBins;
      // get the rotation independent from the rotation direction
      diffAngle = std::abs(
        angles::normalizeAngle(angleCell2State(diffAngleDisc, numAngleBins)));
    }

    return (dist + expectedSteps * stepCost + diffAngle * diffAngleCost);
  }

}
;
