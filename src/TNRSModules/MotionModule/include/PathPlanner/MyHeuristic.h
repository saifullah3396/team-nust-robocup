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

#pragma once

#include "MotionModule/include/PathPlanner/helper.h"
#include "MotionModule/include/PathPlanner/PlanningState.h"

namespace PathPlannerSpace
{

  /**
   * @brief An abstract super class providing methods necessary to be used as
   * heuristic function within the PathPlanner.
   */
  class MyHeuristic
  {
  public:
    enum MyHeuristicType
    {
      EUCLIDEAN = 0,
      EUCLIDEAN_STEPCOST = 1,
      PATHCOST = 2
    };

    MyHeuristic(double cellSize, int numAngleBins, MyHeuristicType type);
    virtual
    ~MyHeuristic();

    /**
     * @return The heuristically determined path costs to get from
     * state 'from' to state 'to' where 'to' is supposed to be the goal of
     * the planning task. (Costs are in meter.)
     */
    virtual double
    getHValue(const PlanningState& from, const PlanningState& to) const = 0;

    MyHeuristicType
    getMyHeuristicType() const
    {
      return myHeuristicType;
    }

  protected:
    double cellSize;
    int numAngleBins;

    const MyHeuristicType myHeuristicType;
  };

  /**
   * @brief Determining the heuristic value by the euclidean distance between
   * two states.
   */
  class EuclideanHeuristic : public MyHeuristic
  {
  public:
    EuclideanHeuristic(double cellSize, int numAngleBins);
    virtual
    ~EuclideanHeuristic();

    virtual double
    getHValue(const PlanningState& from, const PlanningState& to) const;
  };

  /**
   * @brief Determining the heuristic value by the euclidean distance between
   * two states, the expected step costs to get from one state to the other
   * and the difference between the orientation of the two states multiplied
   * by some cost factor. (NOTE: choosing this angular difference cost might
   * overestimate the heuristic value.)
   */
  class EuclStepCostHeuristic : public MyHeuristic
  {

  public:
    EuclStepCostHeuristic(double cellSize, int numAngleBins, double stepCost,
      double diffAngleCost, double maxStepWidth);
    virtual
    ~EuclStepCostHeuristic();

    virtual double
    getHValue(const PlanningState& from, const PlanningState& to) const;

  private:
    const double stepCost;
    const double diffAngleCost;

    /// longest step width
    const double maxStepWidth;
  };

}
;
