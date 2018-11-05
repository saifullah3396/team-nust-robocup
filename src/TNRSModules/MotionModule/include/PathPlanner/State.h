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

namespace PathPlannerSpace
{

  /**
   * @brief A class representing the robot's pose (i.e. position and
   * orientation) in the (continuous) world view. More precisely a state
   * points to the robot's supporting leg.
   */
  class State
  {
  public:
    State();
    State(double x, double y, double theta, Leg leg);
    ~State();

    void
    setX(double x)
    {
      this->x = x;
    }
    void
    setY(double y)
    {
      this->y = y;
    }
    void
    setTheta(double theta)
    {
      this->theta = theta;
    }
    void
    setLeg(Leg leg)
    {
      this->leg = leg;
    }

    double
    getX() const
    {
      return x;
    }
    double
    getY() const
    {
      return y;
    }
    ;
    double
    getTheta() const
    {
      return theta;
    }
    Leg
    getLeg() const
    {
      return leg;
    }

    /**
     * @brief Compare two states on equality of x, y, theta, leg upon
     * a certain degree of float precision.
     */
    bool
    operator ==(const State& s2) const;

    /**
     * @brief Inequality operator for two states (negates the equality
     * operator).
     */
    bool
    operator !=(const State& s2) const;

  private:
    /// The robot's position in x direction.
    double x;
    /// The robot's position in y direction.
    double y;
    /// The robot's orientation.
    double theta;
    /// The robot's supporting leg.
    Leg leg;
  };

}
;
