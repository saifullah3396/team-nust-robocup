/**
 * A footstep planner for humanoid robots.
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/pathPlanner
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

#include <chrono>
#include "MotionModule/include/PathPlanner/FeetDefinitions.h"
#include "MotionModule/include/PathPlanner/PathPlannerEnvironment.h"
#include "MotionModule/include/PathPlanner/helper.h"
#include "MotionModule/include/PathPlanner/PathCostHeuristic.h"
#include "MotionModule/include/PathPlanner/PlanningStateChangeQuery.h"
#include "MotionModule/include/PathPlanner/State.h"
#include "Utils/include/MathsUtils.h"

using namespace std::chrono;

namespace PathPlannerSpace
{

  typedef vector<State>::const_iterator stateIterT;

  /**
   * @brief A class to control the interaction between ROS and the footstep
   * planner.
   */
  class PathPlanner
  {
  public:
    PathPlanner();
    virtual
    ~PathPlanner();

    /**
     * @brief Start a planning task from scratch (will delete information
     * of previous planning tasks). Map and start, goal poses need to be
     * set beforehand.
     *
     * @return Success of planning.
     */
    bool
    plan(bool forceNewPlan = true);

    /// @brief Sets start, goal poses and calls PathPlanner::plan().
    bool
    plan(float startX, float startY, float startTheta, float goalX, float goalY,
      float goalTheta);

    /**
     * @brief Starts a planning task based on previous planning information
     * (note that this method can also be used when no previous planning was
     * performed). Map and start, goal poses need to be set beforehand.
     *
     * @return Success of planning.
     */
    bool
    replan();

    /**
     * @brief Sets the goal pose as two feet (left / right)
     *
     * @return True if the two foot poses have been set successfully.
     */
    bool
    setGoal(const State& leftFoot, const State& rightFoot);

    /**
     * @brief Sets the goal pose as a robot pose centered between two feet.
     *
     * @return True if the two foot poses have been set successfully.
     */
    bool
    setGoal(const float& x, const float& y, const float& theta);

    /**
     * @brief Checks whether the goal is still valid
     *
     * @return True if the goal is valid in updated map
     */
    bool
    checkGoal();

    /**
     * @brief Sets the start pose as a robot pose centered between two feet.
     *
     * @return True if the two foot poses have been set successfully.
     */
    bool
    setStart(const float& x, const float& y, const float& theta);

    /**
     * @brief Sets the start pose as position of left and right footsteps.
     *
     * @return True if the two foot poses have been set successfully.
     */
    bool
    setStart(const State& leftFoot, const State& rightFoot);

    /**
     * @brief Sets the map pointer.
     */
    void
    setMapPtr(GridMap2DPtr mapPtr)
    {
      this->mapPtr = mapPtr;
      plannerEnvironmentPtr->setMapPtr(mapPtr);
    }

    /**
     * @brief Updates the map in the planning environment.
     */
    void
    updateMap();

    /**
     * @brief Checks whether the path is still valid on the current map
     *
     * @return True if a replanning is necessary, i.e. the old path is not valid
     * any more.
     */
    bool
    checkPathValidity();

    /// @brief Set the maximal search time.
    void
    setMaxSearchTime(int searchTime)
    {
      maxSearchTime = searchTime;
    }

    /**
     * @brief Clear the footstep path visualization from a previous planning
     * task.
     */
    void
    clearFootstepPathVis(unsigned numFootsteps = 0);

    /// @return Costs of the planned footstep path.
    double
    getPathCosts() const
    {
      return pathCost;
    }

    /// @return Number of expanded states.
    size_t
    getNumExpandedStates() const
    {
      return plannerPtr->get_n_expands();
    }

    /// @return Number of planned foot poses.
    size_t
    getNumFootPoses() const
    {
      return path.size();
    }

    stateIterT
    getPathBegin() const
    {
      return path.begin();
    }
    stateIterT
    getPathEnd() const
    {
      return path.end();
    }

    /// @return Size of the planned path.
    int
    getPathSize()
    {
      return path.size();
    }

    State
    getStartFootLeft()
    {
      return startFootLeft;
    }
    State
    getStartFootRight()
    {
      return startFootRight;
    }

    /// @brief Reset the previous planning information.
    void
    reset();

    /// @brief Reset and reinitialize the environment.
    void
    resetTotally();

    /// @return True if for the current start and goal pose a path exists.
    bool
    pathExists()
    {
      return (bool) path.size();
    }

    /// @brief Planning parameters.
    EnvironmentParams environmentParams;

  protected:
    /**
     * @return True if the newly calculated path is different from the existing
     * one (if one exists).
     */
    bool
    isNewPath(const vector<int>& newPath);

    /**
     * @brief Extracts the path (list of foot poses) from a list of state
     * IDs calculated by the SBPL.
     */
    bool
    extractPath(const vector<int>& stateIds);

    /// @brief Generates a visualization msgs for a foot pose.
    //void footPoseToMarker(const State& footstep,
    //                      visualizationMsgs::Marker* marker);

    /**
     * @brief Starts the planning task in the underlying SBPL.
     *
     * NOTE: Never call this directly. Always use either plan() or replan() to
     * invoke this method.
     */
    bool
    run();

    /// @brief Returns the foot pose of a leg for a given robot pose.
    State
    getFootPose(const State& robot, Leg side);

    /// @brief Sets the planning algorithm used by SBPL.
    void
    setPlanner();

    /// @brief Updates the environment in case of a changed map.
    void
    updateEnvironment();

    boost::shared_ptr<PathPlannerEnvironment> plannerEnvironmentPtr;
    GridMap2DPtr mapPtr;
    boost::shared_ptr<SBPLPlanner> plannerPtr;

    boost::shared_ptr<const PathCostHeuristic> pathCostHeuristicPtr;

    vector<State> path;

    State startFootLeft;
    State startFootRight;
    State goalFootLeft;
    State goalFootRight;

    double footSeparation;
    double maxStepWidth;
    int collisionCheckAccuracy;

    bool startPoseSetUp, goalPoseSetUp;
    double pathCost;
    bool searchUntilFirstSolution;
    double maxSearchTime;
    double initialEpsilon;

    /**
     * @brief If limit of changed cells is reached the planner starts a new
     * task from the scratch.
     */
    int changedCellsLimit;

    string plannerType;

    vector<int> planningStatesIds;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;

}
;
