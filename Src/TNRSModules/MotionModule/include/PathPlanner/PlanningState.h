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

#pragma once

#include "MotionModule/include/PathPlanner/helper.h"
#include "MotionModule/include/PathPlanner/State.h"

namespace PathPlannerSpace
{
  /**
   * @brief A class representing the robot's pose (i.e. position and
   * orientation) in the underlying SBPL. More precisely a planning state
   * is a discrete representation of the robot's supporting leg.
   *
   * Since SBPL is working on discretized states the planning states are also
   * discretized positions and orientations. This is done by fitting the
   * positions into a grid and the orientations into bins.
   * (NOTE: the resolution of the planning cells is likely to differ from the
   * resolution of the grid map.)
   *
   * The SBPL can access each planning state via an unique ID. Furthermore
   * each planning state can be identified by an (ununique) hash tag generated
   * from its position, location and supporting leg.
   */
  class PlanningState
  {
  public:
    /**
     * @brief x, y and theta represent the global (continuous) position and
     * orientation of the robot's support leg.
     *
     * @param leg The supporting leg.
     * @param cellSize The size of each grid cell discretizing the
     * position.
     * @param numAngleBins The number of bins discretizing the
     * orientation.
     * @param maxHashSize
     */
    PlanningState(double x, double y, double theta, Leg leg, double cellSize,
      int numAngleBins, int maxHashSize);

    /**
     * @brief x, y and theta as discrete bin values (as used internally by
     * the planner).
     */
    PlanningState(int x, int y, int theta, Leg leg, int maxHashSize);

    /// Create a (discrete) PlanningState from a (continuous) State.
    PlanningState(const State& s, double cellSize, int numAngleBins,
      int maxHashSize);

    /// Copy constructor.
    PlanningState(const PlanningState& s);

    ~PlanningState();

    /**
     * @brief Compare two states on equality of x, y, theta, leg. Makes
     * first use of the non-unique hash tag to rule out unequal states.
     */
    bool
    operator ==(const PlanningState& s2) const;

    /**
     * @brief Compare two states on inequality of x, y, theta, leg by
     * comparing the hash tags of the states.
     */
    bool
    operator !=(const PlanningState& s2) const;

    /**
     * @brief Used to attach such an unique ID to the planning state. (This
     * cannot be done in the constructor since often such an ID is not known
     * when the planning state is created.)
     */
    void
    setId(unsigned int id)
    {
      this->id = id;
    }

    Leg
    getLeg() const
    {
      return leg;
    }
    int
    getTheta() const
    {
      return theta;
    }
    int
    getX() const
    {
      return x;
    }
    int
    getY() const
    {
      return y;
    }

    /**
     * @return The (non-unique) hash tag used to identify the planning
     * state.
     */
    unsigned int
    getHashTag() const
    {
      return hashTag;
    }

    /**
     * @return The (unique) ID used within the SBPL to access the
     * planning state.
     */
    int
    getId() const
    {
      return id;
    }

    /// @brief Computes the continuous State the PlanningState represents.
    State
    getState(double cellSize, int numAngleBins) const;

  private:
    /// Value of the grid cell the position's x value is fitted into.
    int x;
    /// Value of the grid cell the position's y value is fitted into.
    int y;
    /// Number of the bin the orientation is fitted into.
    int theta;
    /// The supporting leg.
    Leg leg;

    /// The (unique) ID of the planning state.
    int id;

    /**
     * The (non-unique) hash tag of the planning state. Different hash tags
     * imply that the states differ in x, y, theta, leg.
     */
    unsigned int hashTag;
  };

}
;
