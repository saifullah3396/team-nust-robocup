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

#include "MotionModule/include/PathPlanner/PathCostHeuristic.h"

namespace PathPlannerSpace
{

  PathCostHeuristic::PathCostHeuristic(double cellSize, int numAngleBins,
    double stepCost, double diffAngleCost, double maxStepWidth,
    double inflationRadius) :
    MyHeuristic(cellSize, numAngleBins, PATHCOST), grid(NULL),
      stepCost(stepCost), diffAngleCost(diffAngleCost),
      maxStepWidth(maxStepWidth), inflationRadius(inflationRadius), goalX(-1),
      goalY(-1)
  {
  }

  PathCostHeuristic::~PathCostHeuristic()
  {
    if (grid) resetGrid();
  }

  double
  PathCostHeuristic::getHValue(const PlanningState& current,
    const PlanningState& to) const
  {
    assert(goalX >= 0 && goalY >= 0);

    if (current == to) return 0.0;

    unsigned int fromX;
    unsigned int fromY;
    // could be removed after more testing (then use ...noBounds... again)
    mapPtr->worldToMapNoBounds(
      cell2State(current.getX(), cellSize),
      cell2State(current.getY(), cellSize),
      fromX,
      fromY);

    unsigned int toX;
    unsigned int toY;
    // could be removed after more testing (then use ...noBounds... again)
    mapPtr->worldToMapNoBounds(
      cell2State(to.getX(), cellSize),
      cell2State(to.getY(), cellSize),
      toX,
      toY);

    // cast to unsigned int is safe since goalX/goalY are checked to be >= 0
    if ((unsigned int) goalX != toX || (unsigned int) goalY != toY) {
      std::cout << "PathCostHeuristic::getHValue to a different value than " << "precomputed, heuristic values will be wrong. You need to call " << "calculateDistances() before!" << std::endl;
    }
    assert((unsigned int) goalX == toX && (unsigned int) goalY == toY);

    double dist = double(
      gridSearchPtr->getlowerboundoncostfromstart_inmm(fromX, fromY)) / 1000.0;

    double expectedSteps = dist / maxStepWidth;
    double diffAngle = 0.0;
    if (diffAngleCost > 0.0) {
      // get the number of bins between from.theta and to.theta
      int diffAngleDisc =
        (((to.getTheta() - current.getTheta()) % numAngleBins) + numAngleBins) % numAngleBins;
      // get the rotation independent from the rotation direction
      diffAngle = std::abs(
        angles::normalizeAngle(angleCell2State(diffAngleDisc, numAngleBins)));
    }

    return (dist + expectedSteps * stepCost + diffAngle * diffAngleCost);
  }

  bool
  PathCostHeuristic::calculateDistances(const PlanningState& from,
    const PlanningState& to)
  {
    assert(mapPtr);

    unsigned int fromX;
    unsigned int fromY;
    mapPtr->worldToMapNoBounds(
      cell2State(from.getX(), cellSize),
      cell2State(from.getY(), cellSize),
      fromX,
      fromY);

    unsigned int toX;
    unsigned int toY;
    mapPtr->worldToMapNoBounds(
      cell2State(to.getX(), cellSize),
      cell2State(to.getY(), cellSize),
      toX,
      toY);

    if ((int) toX != goalX || (int) toY != goalY) {
      goalX = toX;
      goalY = toY;
      gridSearchPtr->search(
        grid,
        cvObstacleThreshold,
        goalX,
        goalY,
        fromX,
        fromY,
        SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
    }

    return true;
  }

  void
  PathCostHeuristic::updateMap(GridMap2DPtr map)
  {
    mapPtr.reset();
    mapPtr = map;
    goalX = goalY = -1;
    unsigned width = mapPtr->getWidth();
    unsigned height = mapPtr->getHeight();
    if (gridSearchPtr) gridSearchPtr->destroy();
    gridSearchPtr.reset(
      new SBPL2DGridSearch(width, height, mapPtr->getResolution()));
    if (grid) resetGrid();
    grid = new unsigned char*[width];
    for (unsigned x = 0; x < width; ++x)
      grid[x] = new unsigned char[height];
    for (unsigned y = 0; y < height; ++y) {
      for (unsigned x = 0; x < width; ++x) {
        float dist = mapPtr->distanceMapAtCell(x, y);
        if (dist < 0.0f) std::cout << "Distance map at %d %d out of bounds" << x << y << std::endl;
        else if (dist <= inflationRadius) grid[x][y] = 255;
        else grid[x][y] = 0;
      }
    }
  }

  void
  PathCostHeuristic::resetGrid()
  {
    cv::Size size = mapPtr->size();
    for (int x = 0; x < size.width; ++x) {
      if (grid[x]) {
        delete[] grid[x];
        grid[x] = NULL;
      }
    }
    delete[] grid;
    grid = NULL;
  }

}
;
