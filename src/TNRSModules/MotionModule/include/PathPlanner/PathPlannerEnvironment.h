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

#include <boost/tr1/unordered_set.hpp>
#include <boost/functional/hash.hpp>
#include "Utils/include/ConfigMacros.h"
#include "MotionModule/include/PathPlanner/helper.h"
#include "MotionModule/include/PathPlanner/PathCostHeuristic.h"
#include "MotionModule/include/PathPlanner/MyHeuristic.h"
#include "MotionModule/include/PathPlanner/Footstep.h"
#include "MotionModule/include/PathPlanner/PlanningState.h"
#include "MotionModule/include/PathPlanner/State.h"
#include "SbplLib/include/headers.h"
#include "Utils/include/MathsUtils.h"

using namespace Utils;

namespace PathPlannerSpace
{

  struct EnvironmentParams
  {
    vector<Footstep> footstepSet;
    boost::shared_ptr<MyHeuristic> myHeuristic;

    /// Defines the area of performable (discrete) steps.
    vector<pair<int, int> > stepRange;

    double footSizeX, footSizeY, footSizeZ;
    double footOriginShiftX, footOriginShiftY;
    double footMaxStepX, footMaxStepY, footMaxStepTheta;
    double footMaxStepInvX, footMaxStepInvY, footMaxStepInvTheta;
    double stepCost;
    int collisionCheckAccuracy;
    int hashTableSize;
    double cellSize;
    int numAngleBins;
    bool forwardSearch;
    double maxStepWidth;
    int numRandomNodes;
    double randomNodeDistance;
    double heuristicScale;
  };

  /**
   * @brief A class defining a footstep planner environment for humanoid
   * robots used by the SBPL to perform planning tasks.
   *
   * The environment keeps track of all the planning states expanded during
   * the search. Each planning state can be accessed via its Id. Furthermore
   */
  class PathPlannerEnvironment : public DiscreteSpaceInformation
  {
  public:
    // specialization of hash<int,int>, similar to standard boost::hash on pairs?
    struct IntPairHash
    {
    public:
      size_t
      operator()(pair<int, int> x) const throw ()
      {
        size_t seed = boost::hash<int>()(x.first);
        return boost::hash<int>()(x.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
    };

    typedef vector<int> expStatesT;
    typedef expStatesT::const_iterator expStatesIterT;
    typedef tr1::unordered_set<pair<int, int>, IntPairHash> expStates2dT;
    typedef expStates2dT::const_iterator expStates2dIterT;

    /**
     * @param footstepSet The set of footsteps used for the path planning.
     * @param heuristic The heuristic used by the planner.
     * @param footsizeX Size of the foot in x direction.
     * @param footsizeY Size of the foot in y direction.
     * @param originFootShiftX Shift in x direction from the foot's
     * center.
     * @param originFootShiftY Shift in y direction from the foot's
     * center.
     * @param footMaxStepX The maximal translation in x direction
     * performable by the robot.
     * @param footMaxStepY The maximal translation in y direction
     * performable by the robot.
     * @param footMaxStepTheta The maximal rotation performable by the
     * robot.
     * @param maxInverseFootstepX The minimal translation in x direction
     * performable by the robot.
     * @param maxInverseFootstepY The minimal translation in y direction
     * performable by the robot.
     * @param maxInverseFootstepTheta The minimal rotation performable by
     * the robot.
     * @param stepCost The costs for each step.
     * @param collisionCheckAccuracy Whether to check just the foot's
     * circumcircle (0), the incircle (1) or recursively the circumcircle
     * and the incircle for the whole foot (2) for collision.
     * @param hashTableSize Size of the hash table storing the planning
     * states expanded during the search.
     * @param cellSize The size of each grid cell used to discretize the
     * robot positions.
     * @param numAngleBins The number of bins used to discretize the
     * robot orientations.
     * @param forwardSearch Whether to use forward search (1) or backward
     * search (0).
     */
    PathPlannerEnvironment(const EnvironmentParams& params);

    virtual
    ~PathPlannerEnvironment();

    /**
     * @brief Update the robot's feet poses in the goal state.
     * @return The new Ids (left, right) of the planning state representing the
     * feet.
     */
    pair<int, int>
    updateGoal(const State& footLeft, const State& footRight);

    /**
     * @brief Update the robot's feet poses in the start state.
     * @return The new Ids (left, right) of the planning states representing the
     * feet.
     */
    pair<int, int>
    updateStart(const State& footLeft, const State& rightRight);

    void
    setMapPtr(const GridMap2DPtr& mapPtr)
    {
      this->mapPtr = mapPtr;
    }
    void
    updateMap();

    /**
     * @return True iff the foot in State s is colliding with an
     * obstacle.
     */
    bool
    occupied(const State& s);

    /**
     * @return True iff the foot in State s is colliding with an
     * obstacle.
     */
    bool
    occupied(const State& s, const bool& mapCheck);

    /**
     * @brief Try to receive a state with a certain Id.
     *
     * @return True iff there is a state with such an Id.
     */
    bool
    getState(unsigned int id, State* s);

    /**
     * @brief Resets the current planning task (i.e. the start and goal
     * poses).
     */
    void
    reset();

    /// @return The number of expanded states during the search.
    int
    getNumExpandedStates()
    {
      return numExpandedStates;
    }

    expStates2dIterT
    GetExpandedStatesStart()
    {
      return expandedStates.begin();
    }

    expStates2dIterT
    GetExpandedStatesEnd()
    {
      return expandedStates.end();
    }

    expStatesIterT
    GetRandomStatesStart()
    {
      return randomStates.begin();
    }

    expStatesIterT
    GetRandomStatesEnd()
    {
      return randomStates.end();
    }

    /**
     * @return The costs (in mm, truncated as int) to reach the
     * planning state toStateId from within planning state fromStateId.
     */
    int
    GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * @return The heuristic value to reach the goal from within the
     * planning state stateId (used for forward planning).
     */
    int
    GetGoalHeuristic(int stateID);

    /**
     * @return The heuristic value to reach the start from within
     * the planning state stateId. (Used for backward planning.)
     */
    int
    GetStartHeuristic(int stateID);

    /**
     * @brief Calculates the successor states and the corresponding costs
     * when performing the footstep set on the planning state SourceStateId.
     * (Used for forward planning.)
     */
    void
    GetSuccs(int SourceStateId, vector<int> *SuccIDV, vector<int> *CostV);

    /**
     * @brief Calculates the predecessor states and the corresponding costs
     * when reversing the footstep set on the planning state TargetStateId.
     * (Used for backward planning.)
     */
    void
    GetPreds(int TargetStateId, vector<int> *PredIDV, vector<int> *CostV);

    /**
     * @brief Used for RStar: generate succs/preds at some
     * domain-dependent distance. The number of generated succs/preds is up
     * to the environment.
     */
    virtual void
    GetRandomSuccsatDistance(int sourceStateId, vector<int>* succIdv,
      vector<int>* cLowV);

    /**
     * @brief Used for RStar: generate succs/preds at some
     * domain-dependent distance. The number of generated succs/preds is up
     * to the environment.
     */
    virtual void
    GetRandomPredsatDistance(int targetStateId, vector<int>* predIdv,
      vector<int>* cLowV);

    /// Testing, for R*
    void
    getSuccsTo(int sourceStateId, int goalStateId, vector<int> *succIdv,
      vector<int> *costV);

    /// @return True if two states meet the same condition. Used for R*.
    bool
    AreEquivalent(int stateId1, int stateId2);

    bool
    InitializeEnv(const char *sEnvFile);

    bool
    InitializeMDPCfg(MDPConfig *MDPCfg);

    void
    PrintEnv_Config(FILE *fOut);

    void
    PrintState(int stateId, bool bVerbose, FILE *fOut);

    void
    SetAllActionsandAllOutcomes(CMDPSTATE *state);

    void
    SetAllPreds(CMDPSTATE *state);

    int
    SizeofCreatedEnv();

    /**
     * @return True iff 'to' can be reached by an arbitrary footstep that
     * can be performed by the robot from within 'from'. (This method is
     * used to check whether the goal/start can be reached from within the
     * current state.)
     */
    bool
    reachable(const PlanningState& from, const PlanningState& to);

    void
    getPredsOfGridCells(const vector<State>& changedStates,
      vector<int>* predIds);

    void
    getSuccsOfGridCells(const vector<State>& changedStates,
      vector<int>* succIds);

    /**
     * @brief Update the heuristic values (e.g. after the map has changed).
     * The environment takes care that the update is only done when it is
     * necessary.
     */
    void
    updateHeuristicValues();

    /// Used to scale continuous values in meter to discrete values in mm.
    static const int cvMmScale = 1000;

  protected:
    /**
     * @return The costs (in mm, truncated as int) to reach the
     * planning state ToStateId from within planning state FromStateId.
     */
    int
    getFromToHeuristic(const PlanningState& from, const PlanningState& to);

    /// @return The step cost for reaching 'b' from within 'a'.
    int
    getStepCost(const PlanningState& a, const PlanningState& b);

    /**
     * @return True iff the foot in 's' is colliding with an obstacle.
     */
    bool
    occupied(const PlanningState& s, const bool& mapCheck);

    void
    getRandomNeighs(const PlanningState* currentState, vector<int>* neighIdv,
      vector<int>* cLowV, int nNumofNeighs, int nDistC, bool bSuccs);

    void
    setStateArea(const PlanningState& left, const PlanningState& right);

    /// Wrapper for PathPlannerEnvironment::createNewHashEntry(PlanningState).
    const PlanningState*
    createNewHashEntry(const State& s);

    /**
     * @brief Creates a new planning state for 's' and inserts it into the
     * maps (PlanningState::stateId2State,
     * PlanningState::stateHash2State)
     *
     * @return A pointer to the newly created PlanningState.
     */
    const PlanningState*
    createNewHashEntry(const PlanningState& s);

    /// Wrapper for PathPlannerEnvironment::getHashEntry(PlanningState).
    const PlanningState*
    getHashEntry(const State& s);

    /**
     * @return The pointer to the planning state 's' stored in
     * PathPlannerEnvironment::stateHash2State.
     */
    const PlanningState*
    getHashEntry(const PlanningState& s);

    const PlanningState*
    createHashEntryIfNotExists(const PlanningState& s);

    /**
     * @return True iff 'goal' can be reached by an arbitrary footstep.
     * (Used for forward planning.)
     */
    bool
    closeToGoal(const PlanningState& from);

    /**
     * @return True iff 'start' can be reached by an arbitrary footstep.
     * (Used for backward planning.)
     */
    bool
    closeToStart(const PlanningState& from);

    /// < operator for planning states.
    struct less
    {
      bool
      operator ()(const PlanningState* a, const PlanningState* b) const;
    };

    /**
     * @brief Id of the planning goal, i.e. dependent on the planning direction
     * (forward/backward) this Id is used to map to the goal/start poses.
     */
    int idPlanningGoal;

    /// Id of the start pose of the left foot.
    int idStartFootLeft;
    /// Id of the start pose of the right foot.
    int idStartFootRight;
    /// Id of the goal pose of the left foot.
    int idGoalFootLeft;
    /// Id of the goal pose of the right foot.
    int idGoalFootRight;

    vector<int> stateArea;

    /**
     * @brief Maps from an Id to the corresponding PlanningState. (Used in
     * the SBPL to access a certain PlanningState.)
     */
    vector<const PlanningState*> stateId2State;

    /**
     * @brief Maps from a hash tag to a list of corresponding planning
     * states. (Used in PathPlannerEnvironment to identify a certain
     * PlanningState.)
     */
    vector<const PlanningState*>* stateHash2State;

    /// The set of footsteps used for the path planning.
    const vector<Footstep>& footstepSet;

    /// The heuristic function used by the planner.
    const boost::shared_ptr<MyHeuristic> myHeuristicConstPtr;

    /// Size of the foot in x direction.
    const double footsizeX;
    /// Size of the foot in y direction.
    const double footsizeY;

    /// Shift in x direction from the foot's center.
    const double originFootShiftX;
    /// Shift in y direction from the foot's center.
    const double originFootShiftY;

    /// The maximal translation in x direction (discretized in cell size).
    const int footMaxStepX;
    /// The maximal translation in y direction (discretized in cell size).
    const int footMaxStepY;
    /// The maximal rotation (discretized into bins).
    int footMaxStepTheta;

    /// The minimal translation in x direction (discretized in cell size).
    const int footMaxStepInvX;
    /// The minimal translation in y direction (discretized in cell size).
    const int footMaxStepInvY;
    /// The minimal rotation (discretized into bins).
    int footMaxStepInvTheta;

    /**
     * @brief The costs for each step (discretized with the help of
     * cvMmScale).
     */
    const int stepCost;

    /**
     * @brief Whether to check just the foot's inner circle (0), the hole
     * outer circle (1) or exactly the foot's bounding box (2) for
     * collision.
     */
    const int collisionCheckAccuracy;

    /**
     * @brief Size of the hash table storing the planning states expanded
     * during the search. (Also referred to by maxHashSize.)
     */
    const int hashTableSize;

    /// The size of each grid cell used to discretize the robot positions.
    const double cellSize;
    /// The number of bins used to discretize the robot orientations.
    const int numAngleBins;

    /// Whether to use forward search (1) or backward search (0).
    const bool forwardSearch;

    double maxStepWidth;

    /// number of random neighbors for R*
    const int numRandomNodes;
    /// distance of random neighbors for R* (discretized in cells)
    const int randomNodeDist;

    /**
     * Scaling factor of heuristic, in case it underestimates by a constant
     * factor.
     */
    double heuristicScale;

    /// Indicates if heuristic has to be updated.
    bool heuristicExpired;

    /// Pointer to the map.
    GridMap2DPtr mapPtr;

    expStates2dT expandedStates;
    expStatesT randomStates; ///< random intermediate states for R*
    size_t numExpandedStates;

    bool* stepRange;
  };

}
;
