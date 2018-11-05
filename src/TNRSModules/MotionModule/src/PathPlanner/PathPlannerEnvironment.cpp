/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://!www.ros.org/wiki/footstepPlanner
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
 * along with this program.  If not, see <http://!www.gnu.org/licenses/>.
 */

#include "MotionModule/include/PathPlanner/PathPlannerEnvironment.h"

namespace PathPlannerSpace
{

  PathPlannerEnvironment::PathPlannerEnvironment(
    const EnvironmentParams& params) :
    DiscreteSpaceInformation(), idPlanningGoal(-1), idStartFootLeft(-1),
      idStartFootRight(-1), idGoalFootLeft(-1), idGoalFootRight(-1),
      stateHash2State(new vector<const PlanningState*> [params.hashTableSize]),
      footstepSet(params.footstepSet), myHeuristicConstPtr(params.myHeuristic),
      footsizeX(params.footSizeX), footsizeY(params.footSizeY),
      originFootShiftX(params.footOriginShiftX),
      originFootShiftY(params.footOriginShiftY),
      footMaxStepX(discVal(params.footMaxStepX, params.cellSize)),
      footMaxStepY(discVal(params.footMaxStepY, params.cellSize)),
      footMaxStepTheta(
        angleState2Cell(params.footMaxStepTheta, params.numAngleBins)),
      footMaxStepInvX(discVal(params.footMaxStepInvX, params.cellSize)),
      footMaxStepInvY(discVal(params.footMaxStepInvY, params.cellSize)),
      footMaxStepInvTheta(
        angleState2Cell(params.footMaxStepInvTheta, params.numAngleBins)),
      stepCost(cvMmScale * params.stepCost),
      collisionCheckAccuracy(params.collisionCheckAccuracy),
      hashTableSize(params.hashTableSize), cellSize(params.cellSize),
      numAngleBins(params.numAngleBins), forwardSearch(params.forwardSearch),
      maxStepWidth(double(discVal(params.maxStepWidth, params.cellSize))),
      numRandomNodes(params.numRandomNodes),
      randomNodeDist(params.randomNodeDistance / cellSize),
      heuristicScale(params.heuristicScale), heuristicExpired(true),
      numExpandedStates(0)
  {
    int numAngleBinsHalf = numAngleBins / 2;

    if (footMaxStepTheta >= numAngleBinsHalf) footMaxStepTheta -= numAngleBins;
    if (footMaxStepInvTheta >= numAngleBinsHalf) footMaxStepInvTheta -=
      numAngleBins;

    int numX = footMaxStepX - footMaxStepInvX + 1;
    stepRange = new bool[numX * (footMaxStepY - footMaxStepInvY + 1)];
    //! determine whether a (x,y) translation can be performed by the robot by
    //! checking if it is within a certain area of performable steps
    for (int j = footMaxStepInvY; j <= footMaxStepY; ++j) {
      for (int i = footMaxStepInvX; i <= footMaxStepX; ++i) {
        stepRange[(j - footMaxStepInvY) * numX + (i - footMaxStepInvX)] =
          pointWithinPolygon(i, j, params.stepRange);
      }
    }
  }

  PathPlannerEnvironment::~PathPlannerEnvironment()
  {
    reset();
    if (stateHash2State) {
      delete[] stateHash2State;
      stateHash2State = NULL;
    }
    if (stepRange) {
      delete[] stepRange;
      stepRange = NULL;
    }
  }

  pair<int, int>
  PathPlannerEnvironment::updateGoal(const State& footLeft,
    const State& footRight)
  {
    //! keep the old Ids
    int goalFootIdLeft = idGoalFootLeft;
    int goalFootIdRight = idGoalFootRight;

    //! update the states for both feet (if necessary)
    const PlanningState* pFootLeft = getHashEntry(footLeft);
    if (pFootLeft == NULL) pFootLeft = createNewHashEntry(footLeft);
    const PlanningState* pFootRight = getHashEntry(footRight);
    if (pFootRight == NULL) pFootRight = createNewHashEntry(footRight);
    idGoalFootLeft = pFootLeft->getId();
    idGoalFootRight = pFootRight->getId();

    //! check if everything has been set correctly
    ASSERT(idGoalFootLeft != -1);
    ASSERT(idGoalFootRight != -1);

    //! if using the forward search a change of the goal states involves an
    //! update of the heuristic
    if (forwardSearch) {
      //! check if the goal states have been changed
      if (goalFootIdLeft != idGoalFootLeft && goalFootIdRight != idGoalFootRight) {
        heuristicExpired = true;
        setStateArea(*pFootLeft, *pFootRight);
      }
    }
    return pair<int, int>(idGoalFootLeft, idGoalFootRight);
  }

  pair<int, int>
  PathPlannerEnvironment::updateStart(const State& footLeft,
    const State& footRight)
  {
    //! keep the old Ids
    int startFootIdLeft = idStartFootLeft;
    int startFootIdRight = idStartFootRight;

    //! update the states for both feet (if necessary)
    const PlanningState* pFootLeft = getHashEntry(footLeft);
    if (pFootLeft == NULL) pFootLeft = createNewHashEntry(footLeft);
    const PlanningState* pFootRight = getHashEntry(footRight);
    if (pFootRight == NULL) pFootRight = createNewHashEntry(footRight);
    idStartFootLeft = pFootLeft->getId();
    idStartFootRight = pFootRight->getId();
    //! check if everything has been set correctly
    ASSERT(idStartFootLeft != -1);
    ASSERT(idStartFootRight != -1);

    //auto t1 = high_resolution_clock::now();
    //! if using the backward search a change of the start states involves an
    //! update of the heuristic
    if (!forwardSearch) {
      //! check if the start states have been changed
      if (startFootIdLeft != idStartFootLeft || startFootIdRight != idStartFootRight) {
        heuristicExpired = true;
        setStateArea(*pFootLeft, *pFootRight);
      }
    }
    //auto t2 = high_resolution_clock::now();
    //duration<double> time_span1 = t2 - t1;
    return pair<int, int>(idStartFootLeft, idStartFootRight);
  }

  const PlanningState*
  PathPlannerEnvironment::createNewHashEntry(const State& s)
  {
    PlanningState tmp(s, cellSize, numAngleBins, hashTableSize);
    return createNewHashEntry(tmp);
  }

  const PlanningState*
  PathPlannerEnvironment::createNewHashEntry(const PlanningState& s)
  {
    unsigned int stateHash = s.getHashTag();
    PlanningState* newState = new PlanningState(s);

    size_t stateId = stateId2State.size();
    ASSERT(stateId < (size_t)numeric_limits<int>::max());

    //! insert the Id of the new state into the corresponding map
    newState->setId(stateId);
    stateId2State.push_back(newState);

    //! insert the new state into the hash map at the corresponding position
    stateHash2State[stateHash].push_back(newState);

    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
      StateID2IndexMapping[stateId][i] = -1;
    }

    ASSERT(StateID2IndexMapping.size() - 1 == stateId);
    return newState;
  }

  const PlanningState*
  PathPlannerEnvironment::getHashEntry(const State& s)
  {
    PlanningState tmp(s, cellSize, numAngleBins, hashTableSize);
    return getHashEntry(tmp);
  }

  const PlanningState*
  PathPlannerEnvironment::getHashEntry(const PlanningState& s)
  {
    unsigned int stateHash = s.getHashTag();
    vector<const PlanningState*>::const_iterator state_iter;
    for (state_iter = stateHash2State[stateHash].begin();
      state_iter != stateHash2State[stateHash].end(); ++state_iter) {
      if (*(*state_iter) == s) return *state_iter;
    }
    return NULL;
  }

  const PlanningState*
  PathPlannerEnvironment::createHashEntryIfNotExists(const PlanningState& s)
  {
    const PlanningState* hashEntry = getHashEntry(s);
    if (hashEntry == NULL) hashEntry = createNewHashEntry(s);
    return hashEntry;
  }

  int
  PathPlannerEnvironment::getStepCost(const PlanningState& a,
    const PlanningState& b)
  {
    if (a == b) return 0;

    //! NOTE: instead of using contVal() the calculation is done directly
    //! here because contVal() truncates the input length to int
    double dist =
      euclideanDistance(a.getX(), a.getY(), b.getX(), b.getY()) * cellSize;
    return int(cvMmScale * dist) + stepCost;
  }

  bool
  PathPlannerEnvironment::occupied(const State& s)
  {
    return occupied(
      PlanningState(s, cellSize, numAngleBins, hashTableSize),
      false);
  }

  bool
  PathPlannerEnvironment::occupied(const State& s, const bool& mapCheck)
  {
    return occupied(
      PlanningState(s, cellSize, numAngleBins, hashTableSize),
      mapCheck);
  }

  bool
  PathPlannerEnvironment::occupied(const PlanningState& s, const bool& mapCheck)
  {
    double x = cell2State(s.getX(), cellSize);
    double y = cell2State(s.getY(), cellSize);
    //! collision check for the planning state
    if (mapPtr->isOccupiedAt(x, y)) {
      return true;
    }

    if (!mapCheck) {
      double theta = angleCell2State(s.getTheta(), numAngleBins);
      double thetaCos = cos(theta);
      double thetaSin = sin(theta);

      //! transform the planning state to the foot center
      x += thetaCos * originFootShiftX - thetaSin * originFootShiftY;
      if (s.getLeg() == LEFT) y +=
        thetaSin * originFootShiftX + thetaCos * originFootShiftY;
      else //! leg == RLEG
      y += thetaSin * originFootShiftX - thetaCos * originFootShiftY;

      //! collision check for the foot center
      return collisionCheck(
        x,
        y,
        theta,
        footsizeX,
        footsizeY,
        collisionCheckAccuracy,
        *mapPtr);
    }

    return false;
  }

  bool
  PathPlannerEnvironment::getState(unsigned int id, State* s)
  {
    if (id >= stateId2State.size()) return false;

    const PlanningState* planningState = stateId2State[id];

    s->setX(cell2State(planningState->getX(), cellSize));
    s->setY(cell2State(planningState->getY(), cellSize));
    s->setTheta(
      angles::normalizeAngle(
        angleCell2State(planningState->getTheta(), numAngleBins)));
    s->setLeg(planningState->getLeg());

    /*cout << "state:" << endl;
     cout << "id: " << id << endl;
     cout << "sX: " << s->getX() << endl;
     cout << "sY: " << s->getY() << endl;
     cout << "sT: " << s->getTheta() << endl;
     cout << "sT: " << s->getLeg() << endl;*/

    return true;
  }

  void
  PathPlannerEnvironment::updateMap()
  {
    //cout << "updating env1" << endl;
    if (myHeuristicConstPtr->getMyHeuristicType() == MyHeuristic::PATHCOST) {
      //  cout << "updating env2" << endl;
      boost::shared_ptr<PathCostHeuristic> h =
        boost::dynamic_pointer_cast < PathCostHeuristic > (myHeuristicConstPtr);
      h->updateMap(mapPtr);
      heuristicExpired = true;
      //   cout << "updating env3" << endl;
    }
    // cout << "updating env4" << endl;
  }

  void
  PathPlannerEnvironment::updateHeuristicValues()
  {
    //! check if start and goal have been set
    ASSERT(idGoalFootLeft != -1 && idGoalFootRight != -1);
    ASSERT(idStartFootLeft != -1 && idStartFootRight != -1);
    if (!heuristicExpired) return;
    //cout << "Updating the heuristic values." << endl;
    if (myHeuristicConstPtr->getMyHeuristicType() == MyHeuristic::PATHCOST) {
      boost::shared_ptr<PathCostHeuristic> h =
        boost::dynamic_pointer_cast < PathCostHeuristic > (myHeuristicConstPtr);
      MDPConfig mdpCfg;
      InitializeMDPCfg(&mdpCfg);
      const PlanningState* start = stateId2State[mdpCfg.startstateid];
      const PlanningState* goal = stateId2State[mdpCfg.goalstateid];

      //! NOTE: start/goal state are set to left leg
      bool success;
      if (forwardSearch) success = h->calculateDistances(*start, *goal);
      else success = h->calculateDistances(*goal, *start);
      if (!success) {
        //cout << "Failed to calculate path cost heuristic." << endl;
        exit(1);
      }
    }
    //cout << "Finished updating the heuristic values." << endl;
    heuristicExpired = false;
  }

  void
  PathPlannerEnvironment::reset()
  {
    for (unsigned int i = 0; i < stateId2State.size(); ++i) {
      if (stateId2State[i]) {
        delete stateId2State[i];
      }
    }
    stateId2State.clear();
    if (stateHash2State) {
      for (int i = 0; i < hashTableSize; ++i)
        stateHash2State[i].clear();
    }
    StateID2IndexMapping.clear();
    expandedStates.clear();
    numExpandedStates = 0;
    randomStates.clear();
    idPlanningGoal = -1;
    idGoalFootLeft = -1;
    idGoalFootRight = -1;
    idStartFootLeft = -1;
    idStartFootRight = -1;
    heuristicExpired = true;
  }

  bool
  PathPlannerEnvironment::closeToStart(const PlanningState& from)
  {
    //! NOTE: "goal check" for backward planning
    const PlanningState* start;
    if (from.getLeg() == RIGHT) start = stateId2State[idStartFootLeft];
    else start = stateId2State[idStartFootRight];
    return reachable(*start, from);
  }

  bool
  PathPlannerEnvironment::closeToGoal(const PlanningState& from)
  {
    //! NOTE: "goal check" for forward planning
    const PlanningState* goal;
    if (from.getLeg() == RIGHT) goal = stateId2State[idGoalFootLeft];
    else goal = stateId2State[idGoalFootRight];
    //! TODO: check step if reachable == True
    return reachable(from, *goal);
  }

  bool
  PathPlannerEnvironment::reachable(const PlanningState& from,
    const PlanningState& to)
  {
    if (euclideanDistance(from.getX(), from.getY(), to.getX(), to.getY()) > maxStepWidth) {
      return false;
    }
    Matrix3d rotation;
    MathsUtils::makeRotationZ(
      rotation,
      angleCell2State(from.getTheta(), numAngleBins));
    Matrix4d step;
    step = MathsUtils::makeTransformation(
      rotation,
      cell2State(from.getX(), cellSize),
      cell2State(from.getY(), cellSize),
      0.0).inverse() * MathsUtils::makeTransformation(
      rotation,
      cell2State(to.getX(), cellSize),
      cell2State(to.getY(), cellSize),
      0.0);
    int footstepX = discVal(step(0, 3), cellSize);
    int footstepY = discVal(step(1, 3), cellSize);

    //! calculate the footstep rotation
    int footstepTheta = to.getTheta() - from.getTheta();

    //! transform the value into [-numAngleBins/2..numAngleBins/2)
    int numAngleBinsHalf = numAngleBins / 2;
    if (footstepTheta >= numAngleBinsHalf) footstepTheta -= numAngleBins;
    else if (footstepTheta < -numAngleBinsHalf) footstepTheta += numAngleBins;

    //! adjust for the left foot
    if (from.getLeg() == LEFT) {
      footstepY = -footstepY;
      footstepTheta = -footstepTheta;
    }

    //! check if footstepX is not within the executable range
    if (footstepX > footMaxStepX || footstepX < footMaxStepInvX) return false;
    //! check if footstepY is not within the executable range
    if (footstepY > footMaxStepY || footstepY < footMaxStepInvY) return false;
    //! check if footstepTheta is not within the executable range
    if (footstepTheta > footMaxStepTheta || footstepTheta < footMaxStepInvTheta) return false;
    return stepRange[(footstepY - footMaxStepInvY) * (footMaxStepX - footMaxStepInvX + 1) + (footstepX - footMaxStepInvX)];

//!  //! get the (continuous) orientation of state 'from'
//!  double orient = -(angleCell2State(from.getTheta(), numAngleBins));
//!  double orientCos = cos(orient);
//!  double orientSin = sin(orient);
//!
//!  //! calculate the footstep shift and rotate it into the 'from'-view
//!  int footstepX = to.getX() - from.getX();
//!  int footstepY = to.getY() - from.getY();
//!  double shiftX = footstepX * orientCos - footstepY * orientSin;
//!  double shiftY = footstepX * orientSin + footstepY * orientCos;
//!  footstepX = round(shift_x);
//!  footstepY = round(shift_y);
//!
//!  //! calculate the footstep rotation
//!  int footstepTheta = to.getTheta() - from.getTheta();
//!
//!  //! transform the value into [-numAngleBins/2..numAngleBins/2)
//!  int numAngleBinsHalf = numAngleBins / 2;
//!  if (footstepTheta >= numAngleBinsHalf)
//!    footstepTheta -= numAngleBins;
//!  else if (footstepTheta < -numAngleBinsHalf)
//!    footstepTheta += numAngleBins;
//!
//!  //! adjust for the left foot
//!  if (from.getLeg() == LEFT)
//!  {
//!    footstepY = -footstepY;
//!    footstepTheta = -footstepTheta;
//!  }
//!
//!  return (footstepX <= footMaxStepX &&
//!          footstepX >= footMaxStepInvX &&
//!          footstepY <= footMaxStepY &&
//!          footstepY >= footMaxStepInvY &&
//!          footstepTheta <= footMaxStepTheta &&
//!          footstepTheta >= footMaxStepInvTheta);
  }

  void
  PathPlannerEnvironment::getPredsOfGridCells(
    const vector<State>& changedStates, vector<int>* predIds)
  {
    predIds->clear();

    vector<State>::const_iterator state_iter;
    for (state_iter = changedStates.begin(); state_iter != changedStates.end();
      ++state_iter) {
      PlanningState s(*state_iter, cellSize, numAngleBins, hashTableSize);
      //! generate predecessor planning states
      vector<Footstep>::const_iterator footstepSet_iter;
      for (footstepSet_iter = footstepSet.begin();
        footstepSet_iter != footstepSet.end(); ++footstepSet_iter) {
        PlanningState pred = footstepSet_iter->reverseMeOnThisState(s);
        //! check if predecessor exists
        const PlanningState* predHashEntry = getHashEntry(pred);
        if (predHashEntry == NULL) continue;
        predIds->push_back(predHashEntry->getId());
      }
    }
  }

  void
  PathPlannerEnvironment::getSuccsOfGridCells(
    const vector<State>& changedStates, vector<int>* succIds)
  {
    succIds->clear();
    vector<State>::const_iterator state_iter;
    for (state_iter = changedStates.begin(); state_iter != changedStates.end();
      ++state_iter) {
      PlanningState s(*state_iter, cellSize, numAngleBins, hashTableSize);
      //! generate successors
      vector<Footstep>::const_iterator footstepSet_iter;
      for (footstepSet_iter = footstepSet.begin();
        footstepSet_iter != footstepSet.end(); ++footstepSet_iter) {
        PlanningState succ = footstepSet_iter->performMeOnThisState(s);
        //! check if successor exists
        const PlanningState* succHashEntry = getHashEntry(succ);
        if (succHashEntry == NULL) continue;
        succIds->push_back(succHashEntry->getId());
      }
    }
  }

  int
  PathPlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID)
  {
    ASSERT(
      FromStateID >= 0 && (unsigned int) FromStateID < stateId2State.size());
    ASSERT(ToStateID >= 0 && (unsigned int) ToStateID < stateId2State.size());
    if ((FromStateID == idGoalFootLeft && ToStateID == idGoalFootRight) || (FromStateID == idGoalFootRight && ToStateID == idGoalFootLeft)) {
      return 0;
    }
    const PlanningState* from = stateId2State[FromStateID];
    const PlanningState* to = stateId2State[ToStateID];
    //!      if (myHeuristicConstPtr->getMyHeuristicType() == MyHeuristic::PATHCOST){
    //!        boost::shared_ptr<PathCostHeuristic> pathCostHeuristic = boost::dynamic_pointer_cast<PathCostHeuristic>(myHeuristicConstPtr);
    //!        pathCostHeuristic->calculateDistances(*from, *to);
    //!      }
    return getFromToHeuristic(*from, *to);
  }

  int
  PathPlannerEnvironment::getFromToHeuristic(const PlanningState& from,
    const PlanningState& to)
  {
    return cvMmScale * heuristicScale * myHeuristicConstPtr->getHValue(from, to);
  }

  int
  PathPlannerEnvironment::GetGoalHeuristic(int stateID)
  {
    return GetFromToHeuristic(stateID, idGoalFootLeft);
  }

  void
  PathPlannerEnvironment::GetPreds(int targetStateId, vector<int> *PredIDV,
    vector<int> *CostV)
  {
    PredIDV->clear();
    CostV->clear();
    ASSERT(
      targetStateId >= 0 && (unsigned int) targetStateId < stateId2State.size());
    //! make goal state absorbing (only left!)
    if (targetStateId == idStartFootLeft) return;
    //! add cheap transition from right to left, so right becomes an equivalent
    //! goal
    if (targetStateId == idStartFootRight) {
      PredIDV->push_back(idStartFootLeft);
      CostV->push_back(0.0);
      return;
    }
    const PlanningState* current = stateId2State[targetStateId];
    //! make sure goal state transitions are consistent with
    //! getSuccs(someState, goal_state) where goal_state is reachable by an
    //! arbitrary step from someState
    if (forwardSearch) {
      if (targetStateId == idGoalFootLeft || targetStateId == idGoalFootRight) {
        const PlanningState* s;
        int cost;
        vector<int>::const_iterator stateIdIter;
        for (stateIdIter = stateArea.begin(); stateIdIter != stateArea.end();
          ++stateIdIter) {
          s = stateId2State[*stateIdIter];
          cost = getStepCost(*current, *s);
          PredIDV->push_back(s->getId());
          CostV->push_back(cost);
        }
        return;
      }
    }
    expandedStates.insert(pair<int, int>(current->getX(), current->getY()));
    /*cout << "expandedStates id: " << current->getId() << endl;
     cout << "expandedStates X: " << current->getX() << endl;
     cout << "expandedStates Y: " << current->getY() << endl;
     cout << "expandedStates Theta: " << current->getTheta() << endl;*/
    ++numExpandedStates;
    if (closeToStart(*current)) {
      //! map to the start state id
      PredIDV->push_back(idStartFootLeft);
      //! get actual costs (dependent on whether the start foot is left or right)
      int startId;
      if (current->getLeg() == RIGHT) startId = idStartFootLeft;
      else startId = idStartFootRight;
      CostV->push_back(getStepCost(*current, *stateId2State[startId]));

      return;
    }
    PredIDV->reserve(footstepSet.size());
    CostV->reserve(footstepSet.size());
    vector<Footstep>::const_iterator footstepSetIter;
    for (footstepSetIter = footstepSet.begin();
      footstepSetIter != footstepSet.end(); ++footstepSetIter) {
      const PlanningState predecessor = footstepSetIter->reverseMeOnThisState(
        *current);
      if (occupied(predecessor, false)) continue;
      const PlanningState* predecessorHash = createHashEntryIfNotExists(
        predecessor);
      int cost = getStepCost(*current, *predecessorHash);
      PredIDV->push_back(predecessorHash->getId());
      CostV->push_back(cost);
    }
  }

  int
  PathPlannerEnvironment::GetStartHeuristic(int stateID)
  {
    return GetFromToHeuristic(stateID, idStartFootLeft);
  }

  void
  PathPlannerEnvironment::GetSuccs(int sourceStateId, vector<int> *SuccIDV,
    vector<int> *CostV)
  {
    SuccIDV->clear();
    CostV->clear();
    ASSERT(
      sourceStateId >= 0 && unsigned(sourceStateId) < stateId2State.size());
    //! make goal state absorbing (only left!)
    if (sourceStateId == idGoalFootLeft) return;
    //! add cheap transition from right to left, so right becomes an
    //! equivalent goal
    if (sourceStateId == idGoalFootRight) {
      SuccIDV->push_back(idGoalFootLeft);
      CostV->push_back(0.0);
      return;
    }
    const PlanningState* current = stateId2State[sourceStateId];
    //! make sure start state transitions are consistent with
    //! getPreds(someState, start_state) where some_state is reachable by an
    //! arbitrary step from startState
    if (!forwardSearch) {
      if (sourceStateId == idStartFootLeft || sourceStateId == idStartFootRight) {
        const PlanningState* s;
        int cost;
        vector<int>::const_iterator stateIdIter;
        for (stateIdIter = stateArea.begin(); stateIdIter != stateArea.end();
          ++stateIdIter) {
          s = stateId2State[*stateIdIter];
          cost = getStepCost(*current, *s);
          SuccIDV->push_back(s->getId());
          CostV->push_back(cost);
        }
        return;
      }
    }
    expandedStates.insert(pair<int, int>(current->getX(), current->getY()));
    ++numExpandedStates;
    if (closeToGoal(*current)) {
      int goal_id;
      ASSERT(current->getLeg() != NOLEG);
      if (current->getLeg() == RIGHT) goal_id = idGoalFootLeft;
      else goal_id = idGoalFootRight;
      const PlanningState* goal = stateId2State[goal_id];
      SuccIDV->push_back(goal_id);
      CostV->push_back(getStepCost(*current, *goal));
      return;
    }
    SuccIDV->reserve(footstepSet.size());
    CostV->reserve(footstepSet.size());
    vector<Footstep>::const_iterator footstepSetIter;
    for (footstepSetIter = footstepSet.begin();
      footstepSetIter != footstepSet.end(); ++footstepSetIter) {
      PlanningState successor = footstepSetIter->performMeOnThisState(*current);
      if (occupied(successor, false)) continue;

      const PlanningState* successorHashEntry = createHashEntryIfNotExists(
        successor);

      int cost = getStepCost(*current, *successorHashEntry);
      SuccIDV->push_back(successorHashEntry->getId());
      CostV->push_back(cost);
    }
  }

  void
  PathPlannerEnvironment::getSuccsTo(int sourceStateId, int goalStateId,
    vector<int> *SuccIDV, vector<int> *CostV)
  {
    //!return getSuccs(sourceStateId, SuccIDV, CostV);
    SuccIDV->clear();
    CostV->clear();
    ASSERT(
      sourceStateId >= 0 && unsigned(sourceStateId) < stateId2State.size());
    //! make goal state absorbing
    if (sourceStateId == idGoalFootLeft) return;

    const PlanningState* current = stateId2State[sourceStateId];
    expandedStates.insert(pair<int, int>(current->getX(), current->getY()));
    ++numExpandedStates;
    //! add cheap transition from right to left, so right becomes an equivalent goal
    if (goalStateId == idGoalFootLeft && sourceStateId == idGoalFootRight && current->getLeg() == RIGHT) {
      SuccIDV->push_back(idGoalFootLeft);
      CostV->push_back(stepCost);
      return;
    }
    if (closeToGoal(*current)) {
      int goal_id;
      ASSERT(current->getLeg() != NOLEG);
      if (current->getLeg() == RIGHT) {
        goal_id = idGoalFootLeft;
      } else {
        goal_id = idGoalFootRight;
      }
      const PlanningState* goal = stateId2State[goal_id];
      int cost = getStepCost(*current, *goal);
      SuccIDV->push_back(goal_id);
      CostV->push_back(cost);
      return;
    }
    //! intermediate goal reachable (R*)?
    ASSERT(goalStateId >= 0 && unsigned(goalStateId) < stateId2State.size());
    const PlanningState* randomGoal = stateId2State[goalStateId];
    if (randomGoal->getLeg() != current->getLeg() && reachable(
      *current,
      *randomGoal)) {
      int cost = getStepCost(*current, *randomGoal);
      SuccIDV->push_back(goalStateId);
      CostV->push_back(cost);
    }

    SuccIDV->reserve(footstepSet.size());
    CostV->reserve(footstepSet.size());
    vector<Footstep>::const_iterator footstepSetIter;
    for (footstepSetIter = footstepSet.begin();
      footstepSetIter != footstepSet.end(); ++footstepSetIter) {
      PlanningState successor = footstepSetIter->performMeOnThisState(*current);
      if (occupied(successor, false)) continue;
      const PlanningState* successorHash = createHashEntryIfNotExists(
        successor);
      int cost = getStepCost(*current, *successorHash);
      SuccIDV->push_back(successorHash->getId());
      CostV->push_back(cost);
    }
  }

  void
  PathPlannerEnvironment::GetRandomSuccsatDistance(int sourceStateId,
    vector<int>* SuccIDV, vector<int>* cLowV)
  {
    ASSERT(
      sourceStateId >= 0 && unsigned(sourceStateId) < stateId2State.size());
    //!goal state should be absorbing
    if (sourceStateId == idGoalFootLeft || sourceStateId == idGoalFootRight) return;
    const PlanningState* currentState = stateId2State[sourceStateId];
    //! TODO: closeToGoal?
    //!
    //!      if (closeToGoal(*currentState))
    //!        return;

    //!get the successors
    getRandomNeighs(
      currentState,
      SuccIDV,
      cLowV,
      numRandomNodes,
      randomNodeDist,
      true);
  }

  void
  PathPlannerEnvironment::GetRandomPredsatDistance(int targetStateId,
    vector<int>* PredIDV, vector<int>* cLowV)
  {
    ASSERT(
      targetStateId >= 0 && unsigned(targetStateId) < stateId2State.size());
    //! start state should be absorbing
    if (targetStateId == idStartFootLeft || targetStateId == idStartFootRight) return;
    const PlanningState* currentState = stateId2State[targetStateId];
    //! TODO: ???
    //!      if(closeToStart(*currentState))
    //!        return;

    //!get the predecessors
    getRandomNeighs(
      currentState,
      PredIDV,
      cLowV,
      numRandomNodes,
      randomNodeDist,
      false);
  }

//!generates nNumofNeighs random neighbors of cell <X,Y> at distance nDistC (measured in cells)
//!it will also generate goal if within this distance as an additional neighbor
//!bSuccs is set to true if we are computing successor states, otherwise it is Preds
//! (see fct. implemented in environmentNav2D)
  void
  PathPlannerEnvironment::getRandomNeighs(const PlanningState* currentState,
    vector<int>* neighIDV, vector<int>* cLowV, int nNumofNeighs, int nDistC,
    bool bSuccs)
  {
    //!clear the successor array
    neighIDV->clear();
    cLowV->clear();
    //!get X, Y for the states
    int X = currentState->getX();
    int Y = currentState->getY();
    //!int theta = currentState->getTheta();
    //!see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
    //! NOTE: "goal check" for backward planning
    const PlanningState* goalLeft = NULL;
    const PlanningState* goalRight = NULL;
    if (bSuccs) {
      goalLeft = stateId2State[idGoalFootLeft];
      goalRight = stateId2State[idGoalFootRight];
    } else {
      goalLeft = stateId2State[idStartFootLeft];
      goalRight = stateId2State[idStartFootRight];
    }
    int nDistSq = nDistC * nDistC;
    //!add left if within the distance
    if (euclideanDistanceSq(X, Y, goalLeft->getX(), goalLeft->getY()) <= nDistSq) {
      //!compute clow
      int clow;
      if (bSuccs) clow = getFromToHeuristic(*currentState, *goalLeft);
      else clow = getFromToHeuristic(*goalLeft, *currentState);

      neighIDV->push_back(goalLeft->getId());
      cLowV->push_back(clow);
      randomStates.push_back(goalLeft->getId());
    }
    //!add right if within the distance
    if (euclideanDistanceSq(X, Y, goalRight->getX(), goalRight->getY()) <= nDistSq) {
      //!compute clow
      int clow;
      if (bSuccs) clow = getFromToHeuristic(*currentState, *goalRight);
      else clow = getFromToHeuristic(*goalRight, *currentState);
      neighIDV->push_back(goalRight->getId());
      cLowV->push_back(clow);
      randomStates.push_back(goalRight->getId());
    }
    //!iterate through random actions
    int nAttempts = 0;
    for (int i = 0; i < nNumofNeighs && nAttempts < 5 * nNumofNeighs;
      ++i, ++nAttempts) {
      //! pick goal in random direction
      float fDir = (float) (TWO_PI * (((double) rand()) / RAND_MAX));
      int dX = (int) (nDistC * cos(fDir));
      int dY = (int) (nDistC * sin(fDir));
      //!get the coords of the state
      int newX = X + dX;
      int newY = Y + dY;
      //! TODO / FIXME x,y, can be negative! need offset
      //! check if outside of map:
      //!        if (newX < 0 || newY < 0 || unsigned(newX) >= mapPtr->getInfo().width || unsigned(newY) >= mapPtr->getInfo().height){
      //!          i--;
      //!          ROS_INFO("Outside of map: %d %d", newX, newY);
      //!          continue;
      //!        }
      //! direction of random exploration (facing forward):
      int newTheta = angleState2Cell(fDir, numAngleBins);
      //! random left/right
      Leg newLeg = Leg(rand() % 2);
      PlanningState randomState(newX, newY, newTheta, newLeg, hashTableSize);
      //! add both left and right if available:
      //!        int sep = discVal(0.07, cellSize);
      //!        int ddX = int(-sin(fDir) * sep);
      //!        int ddY = int(cos(fDir) * sep);
      //!        PlanningState randomState(newX+ddX, newY+ddY, newTheta, LEFT, hashTableSize);
      //!
      //!        PlanningState randomStateR(newX-ddX, newY-ddY, newTheta, RIGHT, hashTableSize);
      if (!occupied(randomState, false)) {
        const PlanningState* randomHashEntry = getHashEntry(randomState);
        if (randomHashEntry == NULL) {
          randomHashEntry = createNewHashEntry(randomState);
          randomStates.push_back(randomHashEntry->getId());
        }

        //!compute clow
        int clow;
        if (bSuccs) clow = GetFromToHeuristic(
          currentState->getId(),
          randomHashEntry->getId());
        else clow = GetFromToHeuristic(
          randomHashEntry->getId(),
          currentState->getId());
        neighIDV->push_back(randomHashEntry->getId());
        cLowV->push_back(clow);
      } else {
        i--;
      }
      //!        if(!occupied(randomStateR))
      //!        {
      //!          const PlanningState* randomHashEntry = getHashEntry(randomStateR);
      //!          if (randomHashEntry == NULL){
      //!            randomHashEntry = createNewHashEntry(randomStateR);
      //!            randomStates.push_back(randomHashEntry->getId());
      //!          }
      //!
      //!          //!compute clow
      //!          int clow;
      //!          if(bSuccs)
      //!            clow = getFromToHeuristic(currentState->getId(), randomHashEntry->getId());
      //!          else
      //!            clow = getFromToHeuristic(randomHashEntry->getId(), currentState->getId());
      //!
      //!          neighIDV->push_back(randomHashEntry->getId());
      //!          CLowV->push_back(clow);
      //!
      //!        }else{
      //!          i--;
      //!        }
    }
    if (neighIDV->size() == 0) {
      cout << "Could not create any random neighbor nodes." << " Attempts: " << nAttempts << " from id: " << currentState->getId() << " (" << X << ", " << Y << ")" << endl;
    } else cout << "Created " << neighIDV->size() << " random neighbors (" << nAttempts << " attempts) from id " << " from id: " << currentState->getId() << " (" << X << ", " << Y << ")" << endl;
  }

  bool
  PathPlannerEnvironment::AreEquivalent(int stateId1, int stateId2)
  {
    ASSERT(
      stateId1 >= 0 && stateId2 >= 0 && unsigned(stateId1) < stateId2State.size() && unsigned(stateId2) < stateId2State.size());
    if (stateId1 == stateId2) return true;
    const PlanningState* s1 = stateId2State[stateId1];
    const PlanningState* s2 = stateId2State[stateId2];
    //!    //! approximately compare, ignore theta:
    return (abs(s1->getX() - s2->getX()) < 1 && abs(s1->getY() - s2->getY()) < 1
    //!                      && abs(s1->getTheta() - s2->getTheta()) < 3
    && s1->getLeg() == s2->getLeg());
//!  compare the actual values (exact comparison)
//!  return (*s1 == *s2);
  }

  bool
  PathPlannerEnvironment::InitializeEnv(const char *sEnvFile)
  {
    return true;
  }

  bool
  PathPlannerEnvironment::InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    //! NOTE: The internal start and goal ids are set here to the left foot
    //! (this affects the calculation of the heuristic values)
    MDPCfg->goalstateid = idGoalFootLeft;
    MDPCfg->startstateid = idStartFootLeft;
    ASSERT(idGoalFootLeft != -1);
    ASSERT(idStartFootLeft != -1);
    return true;
  }

  void
  PathPlannerEnvironment::PrintEnv_Config(FILE *fOut)
  {
    //! NOTE: implement this if the planner needs to print out configurations
    cout << "PathPlanerEnvironment::PrintEnvConfig: Hit "
      "unimplemented function. Check this!" << endl;
  }

  void
  PathPlannerEnvironment::PrintState(int stateId, bool bVerbose, FILE *fOut)
  {
    if (fOut == NULL) {
      fOut = stdout;
    }
    if (stateId == idGoalFootLeft && bVerbose) {
      SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }
    const PlanningState* s = stateId2State[stateId];
    if (bVerbose) {
      SBPL_FPRINTF(fOut, "X=%i Y=%i THETA=%i FOOT=%i\n",
        s->getX(), s->getY(), s->getTheta(), s->getLeg());
    } else {
      SBPL_FPRINTF(fOut, "%i %i %i %i\n",
        s->getX(), s->getY(), s->getTheta(), s->getLeg());
    }
  }

  void
  PathPlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE *state)
  {
    cout << "PathPlannerEnvironment::SetAllActionsandAllOutcomes: Hit"
      " unimplemented function. Check this!" << endl;
  }

  void
  PathPlannerEnvironment::SetAllPreds(CMDPSTATE *state)
  {
    cout << "PathPlannerEnvironment::SetAllPreds: Hit unimplemented "
      "function. Check this!" << endl;
  }

  int
  PathPlannerEnvironment::SizeofCreatedEnv()
  {
    return stateId2State.size();
  }

  void
  PathPlannerEnvironment::setStateArea(const PlanningState& left,
    const PlanningState& right)
  {
    stateArea.clear();
    const PlanningState* pState = getHashEntry(right);
    stateArea.push_back(pState->getId());
    double contStepX, contStepY, contStepTheta;
    for (int stepY = footMaxStepInvY; stepY <= footMaxStepY; ++stepY) {
      for (int stepX = footMaxStepInvX; stepX <= footMaxStepX; ++stepX) {
        for (int stepTheta = footMaxStepInvTheta; stepTheta <= footMaxStepTheta;
          ++stepTheta) {
          contStepX = contVal(stepX, cellSize);
          contStepY = contVal(stepY, cellSize);
          contStepTheta = angleCell2State(stepTheta, numAngleBins);
          Footstep step(
            contStepX,
            contStepY,
            contStepTheta,
            cellSize,
            numAngleBins,
            hashTableSize);
          if (forwardSearch) {
            PlanningState pred = step.reverseMeOnThisState(left);
            if (occupied(pred, false) || !reachable(pred, left)) continue;
            pState = createHashEntryIfNotExists(pred);
            stateArea.push_back(pState->getId());

            pred = step.reverseMeOnThisState(right);
            if (occupied(pred, false) || !reachable(pred, right)) continue;
            pState = createHashEntryIfNotExists(pred);
            stateArea.push_back(pState->getId());
          } else {
            PlanningState succ = step.performMeOnThisState(left);
            if (occupied(succ, false) || !reachable(left, succ)) continue;
            pState = createHashEntryIfNotExists(succ);
            stateArea.push_back(pState->getId());

            succ = step.performMeOnThisState(right);
            if (occupied(succ, false) || !reachable(right, succ)) continue;
            pState = createHashEntryIfNotExists(succ);
            stateArea.push_back(pState->getId());
          }
        }
      }
    }
  }

  bool
  PathPlannerEnvironment::less::operator ()(const PlanningState* a,
    const PlanningState* b) const
  {
    if (a->getX() < b->getX()) return true;
    else if (a->getY() < b->getY()) return true;
    else return false;
  }

}
;
