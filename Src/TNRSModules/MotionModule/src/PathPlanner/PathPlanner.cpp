/**
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/pathPlanner
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

#include "MotionModule/include/PathPlanner/PathPlanner.h"
#include "MotionModule/include/PathPlanner/FeetDefinitions.h"

namespace PathPlannerSpace
{

  PathPlanner::PathPlanner() :
    startPoseSetUp(false), goalPoseSetUp(false), pathCost(0)
  {
    string heuristicType;
    double diffAngleCost;
    GET_CONFIG(
      "PathPlanner",
      (string, PathPlanner.heuristicType, heuristicType), (double, PathPlanner.heuristicScale, environmentParams.heuristicScale), (int, PathPlanner.maxHashSize, environmentParams.hashTableSize), (int, PathPlanner.collisionCheckAccuracy, environmentParams.collisionCheckAccuracy), (double, Map.cellSize, environmentParams.cellSize), (int, PathPlanner.numAngleBins, environmentParams.numAngleBins), (double, PathPlanner.stepCost, environmentParams.stepCost), (double, PathPlanner.diffAngleCost, diffAngleCost), (string, PathPlanner.plannerType, plannerType), (bool, PathPlanner.searchUntilFirstSolution, searchUntilFirstSolution), (double, PathPlanner.allocatedTime, maxSearchTime), (bool, PathPlanner.forwardSearch, environmentParams.forwardSearch), (double, PathPlanner.initialEpsilon, initialEpsilon), (int, PathPlanner.changedCellsLimit, changedCellsLimit), (int, PathPlanner.numRandomNodes, environmentParams.numRandomNodes), (double, PathPlanner.randomNodeDist, environmentParams.randomNodeDistance), (double, Foot.sizeX, environmentParams.footSizeX), (double, Foot.sizeY, environmentParams.footSizeY), (double, Foot.sizeZ, environmentParams.footSizeZ), (double, Foot.separation, footSeparation), (double, Foot.originShiftX, environmentParams.footOriginShiftX), (double, Foot.originShiftY, environmentParams.footOriginShiftY), (double, PathPlanner.maxStepX, environmentParams.footMaxStepX), (double, PathPlanner.maxStepY, environmentParams.footMaxStepY), (double, PathPlanner.maxStepTheta, environmentParams.footMaxStepTheta), (double, PathPlanner.maxStepInvX, environmentParams.footMaxStepInvX), (double, PathPlanner.maxStepInvY, environmentParams.footMaxStepInvY), (double, PathPlanner.maxStepInvTheta, environmentParams.footMaxStepInvTheta), )

    vector<double> fsX =
      vector<double>(
        fsParameterized[0],
        fsParameterized[0] + sizeof(fsParameterized[0]) / sizeof(fsParameterized[0][0]));
    vector<double> fsY =
      vector<double>(
        fsParameterized[1],
        fsParameterized[1] + sizeof(fsParameterized[1]) / sizeof(fsParameterized[1][0]));
    vector<double> fsTheta =
      vector<double>(
        fsParameterized[2],
        fsParameterized[2] + sizeof(fsParameterized[2]) / sizeof(fsParameterized[2][0]));

    ASSERT(fsX.size() == fsY.size() && fsY.size() == fsTheta.size());

    environmentParams.footstepSet.clear();
    double maxStepWidth = 0;
    for (int i = 0; i < fsX.size(); ++i) {
      double x = (double) fsX[i];
      double y = (double) fsY[i];
      double theta = (double) fsTheta[i];
      Footstep f(
        x,
        y,
        theta,
        environmentParams.cellSize,
        environmentParams.numAngleBins,
        environmentParams.hashTableSize);
      environmentParams.footstepSet.push_back(f);
      double curStepWidth = sqrt(x * x + y * y);
      if (curStepWidth > maxStepWidth) maxStepWidth = curStepWidth;
    }
    vector<double> stepRangeX = vector<double>(
      stepRanges[0],
      stepRanges[0] + sizeof(stepRanges[0]) / sizeof(stepRanges[0][0]));
    vector<double> stepRangeY = vector<double>(
      stepRanges[1],
      stepRanges[1] + sizeof(stepRanges[1]) / sizeof(stepRanges[1][0]));
    //! Create step range
    environmentParams.stepRange.clear();
    environmentParams.stepRange.reserve(stepRangeX.size());
    double x, y;
    double maxX = 0.0;
    double maxY = 0.0;
    double cellSize = environmentParams.cellSize;
    for (int i = 0; i < stepRangeX.size(); ++i) {
      x = (double) stepRangeX[i];
      y = (double) stepRangeY[i];
      if (fabs(x) > maxX) maxX = fabs(x);
      if (fabs(y) > maxY) maxY = fabs(y);
      environmentParams.stepRange.push_back(
        pair<int, int>(discVal(x, cellSize), discVal(y, cellSize)));
    }

    //! Insert first point again at the end!
    environmentParams.stepRange.push_back(environmentParams.stepRange[0]);
    environmentParams.maxStepWidth = sqrt(maxX * maxX + maxY * maxY) * 1.5;

    //! Initialize the heuristic
    boost::shared_ptr<MyHeuristic> h;
    //cout << "Initializing Heuristic" << endl;
    if (heuristicType == "EuclideanHeuristic") {
      h.reset(
        new EuclideanHeuristic(
          environmentParams.cellSize,
          environmentParams.numAngleBins));
    } else if (heuristicType == "EuclStepCostHeuristic") {
      h.reset(
        new EuclStepCostHeuristic(
          environmentParams.cellSize,
          environmentParams.numAngleBins,
          environmentParams.stepCost,
          diffAngleCost,
          maxStepWidth));
    } else if (heuristicType == "PathCostHeuristic") {
      //! For heuristic inflation
      double footIncircle = min(
        (environmentParams.footSizeX / 2.0 - abs(
          environmentParams.footOriginShiftX)),
        (environmentParams.footSizeY / 2.0 - abs(
          environmentParams.footOriginShiftY)));
      ASSERT(footIncircle > 0.0);

      h.reset(
        new PathCostHeuristic(
          environmentParams.cellSize,
          environmentParams.numAngleBins,
          environmentParams.stepCost,
          diffAngleCost,
          maxStepWidth,
          footIncircle));
      //! Keep a local ptr for visualization
      pathCostHeuristicPtr =
        boost::dynamic_pointer_cast < PathCostHeuristic > (h);
    } else {
      //cout << "Heuristic not available." << endl;
      exit(1);
    }
    //cout << "Reseting environment" << endl;
    environmentParams.myHeuristic = h;

    //! Initialize the planner environment
    plannerEnvironmentPtr.reset(new PathPlannerEnvironment(environmentParams));
    //cout << "Resetted environment" << endl;
    //! Set up planner
    if (plannerType == "ARAPlanner" || plannerType == "ADPlanner" || plannerType == "RSTARPlanner") {
      //cout << "Planning with " << plannerType << endl;
    } else {
      //cout << "Planner "<< plannerType << " not available" << endl;
      exit(1);
    }

    //if (environmentParams.forwardSearch)
    //cout << "Search direction: forward planning" << endl;
    //else
    // cout << "Search direction: backward planning" << endl;
    setPlanner();
  }

  PathPlanner::~PathPlanner()
  {
  }

  void
  PathPlanner::setPlanner()
  {
    if (plannerType == "ARAPlanner") {
      plannerPtr.reset(
        new ARAPlanner(
          plannerEnvironmentPtr.get(),
          environmentParams.forwardSearch));
    } else if (plannerType == "ADPlanner") {
      plannerPtr.reset(
        new ADPlanner(
          plannerEnvironmentPtr.get(),
          environmentParams.forwardSearch));
    } else if (plannerType == "RSTARPlanner") {
      plannerPtr.reset(
        new RSTARPlanner(
          plannerEnvironmentPtr.get(),
          environmentParams.forwardSearch));
    }
  }

  bool
  PathPlanner::run()
  {
    bool pathExisted = (bool) path.size();
    int ret = 0;
    MDPConfig mdpConfig;
    vector<int> solutionStateIds;
    //! Commit start/goal poses to the environment
    //auto t1 = high_resolution_clock::now();
    plannerEnvironmentPtr->updateStart(startFootLeft, startFootRight);
    //auto t2 = high_resolution_clock::now();
    //duration<double> time_span1 = t2 - t1;
    //cout << "time1: " << time_span1.count() << endl;
    plannerEnvironmentPtr->updateGoal(goalFootLeft, goalFootRight);
    //auto t3 = high_resolution_clock::now();
    //duration<double> time_span2 = t3 - t2;
    //cout << "time2: " << time_span2.count() << endl;
    plannerEnvironmentPtr->updateHeuristicValues();
    plannerEnvironmentPtr->InitializeEnv(NULL);
    plannerEnvironmentPtr->InitializeMDPCfg(&mdpConfig);

    //! Inform AD planner about changed (start) states for replanning
    if (pathExisted && !environmentParams.forwardSearch && plannerType == "ADPlanner") {
      vector<int> changedEdges;
      changedEdges.push_back(mdpConfig.startstateid);
      //! update the AD planner
      boost::shared_ptr<ADPlanner> adPlanner =
        boost::dynamic_pointer_cast < ADPlanner > (plannerPtr);
      adPlanner->update_preds_of_changededges(&changedEdges);
    }

    //! set up SBPL
    if (plannerPtr->set_start(mdpConfig.startstateid) == 0) {
      //cout << "Failed to set start state." << endl;
      return false;
    }

    if (plannerPtr->set_goal(mdpConfig.goalstateid) == 0) {
      //cout << "Failed to set goal state." << endl;
      return false;
    }

    plannerPtr->set_initialsolution_eps(initialEpsilon);
    plannerPtr->set_search_mode(searchUntilFirstSolution);

    //auto t4 = high_resolution_clock::now();
    //duration<double> time_span3 = t4 - t3;
    //cout << "time3: " << time_span3.count() << endl;

    /*cout << "Start planning (max time: "
     <<  maxSearchTime
     << ", initial eps: "
     << initialEpsilon
     << "("
     << plannerPtr->get_initial_eps()
     << ")"
     << endl;*/
    //int pathCost;
    high_resolution_clock::time_point tStart = high_resolution_clock::now();
    try {
      //ret = plannerPtr->replan(maxSearchTime, &solutionStateIds, &pathCost);
      ret = plannerPtr->replan(maxSearchTime, &solutionStateIds);
    } catch (const exception& e) {
      cout << "SBPL planning failed:" << e.what();
      return false;
    }
    //high_resolution_clock::time_point tEnd = 
    //  high_resolution_clock::now();
    //duration<double> time_span = tEnd - tStart;
    /*cout << "Solution of size "
     << solutionStateIds.size()
     << "found after "
     << DataUtils::varToString(time_span.count())
     << "secs."
     << endl;*/
    //pathCost = double(pathCost) / PathPlannerEnvironment::cvMmScale;
    bool pathIsNew = isNewPath(solutionStateIds);
    if (ret && solutionStateIds.size() > 0) {
      if (!pathIsNew) cout << "Solution found by SBPL is the same as the old solution. " << "This could indicate that replanning failed." << endl;

      if (extractPath(solutionStateIds)) {
        /*cout << "Expanded states: "
         << plannerEnvironmentPtr->getNumExpandedStates()
         << " total / "
         << plannerPtr->get_n_expands()
         << " new."
         << endl;
         cout << "Final eps: " << plannerPtr->get_final_epsilon() << endl;*/
        //cout << "Path cost: " << pathCost << endl;
        planningStatesIds = solutionStateIds;
        return true;
      } else {
        //cout << "Extracting path failed." << endl;
        return false;
      }
    } else {
      //cout << "No solution found" << endl;
      return false;
    }
  }

  bool
  PathPlanner::extractPath(const vector<int>& stateIds)
  {
    path.clear();
    State s;
    State startLeft;
    vector<int>::const_iterator stateIdsIter = stateIds.begin();

    //! first state is always the robot's left foot
    if (!plannerEnvironmentPtr->getState(*stateIdsIter, &startLeft)) {
      path.clear();
      return false;
    }
    ++stateIdsIter;
    if (!plannerEnvironmentPtr->getState(*stateIdsIter, &s)) {
      path.clear();
      return false;
    }
    ++stateIdsIter;

    //! check if the robot's left foot can be ommited as first state in the path,
    //! i.e. the robot's right foot is appended first to the path
    if (s.getLeg() == LEFT) path.push_back(startFootRight);
    else path.push_back(startLeft);
    path.push_back(s);

    for (; stateIdsIter < stateIds.end(); ++stateIdsIter) {
      if (!plannerEnvironmentPtr->getState(*stateIdsIter, &s)) {
        path.clear();
        return false;
      }
      path.push_back(s);
    }

    //! add last neutral step
    if (path.back().getLeg() == RIGHT) path.push_back(goalFootLeft);
    else //! lastLeg == LEFT
    path.push_back(goalFootRight);

    return true;
  }

  void
  PathPlanner::reset()
  {
    //cout << "Resetting planner" << endl;
    //! reset the previously calculated paths
    path.clear();
    planningStatesIds.clear();
    //! reset the planner
    //! INFO: forcePlanningFromScratch was not working properly the last time
    //! checked; therefore instead of using this function the planner is manually
    //! reset
    //!plannerPtr->forcePlanningFromScratch();
    plannerEnvironmentPtr->reset();
    setPlanner();
    //cout << "resetted planner" << endl;
  }

  void
  PathPlanner::resetTotally()
  {
    //cout << "Resetting planner and environment" << endl;
    //! reset the previously calculated paths
    path.clear();
    planningStatesIds.clear();
    //! reinitialize the planner environment
    plannerEnvironmentPtr.reset(new PathPlannerEnvironment(environmentParams));
    setPlanner();
  }

  bool
  PathPlanner::plan(bool forceNewPlan)
  {
    if (!mapPtr) {
      //cout << "PathPlanner has no map for planning yet." << endl;
      return false;
    }
    if (!goalPoseSetUp || !startPoseSetUp) {
      /*cout << "PathPlanner has not set the start and/or goal pose "
       << "yet."
       << endl;*/
      return false;
    }
    if (forceNewPlan || plannerType == "RSTARPlanner" || plannerType == "ARAPlanner") {
      reset();
    }
    //! start the planning and return success
    return run();
  }

  bool
  PathPlanner::replan()
  {
    return plan(false);
  }

  bool
  PathPlanner::plan(float startX, float startY, float startTheta, float goalX,
    float goalY, float goalTheta)
  {
    if (!(setStart(startX, startY, startTheta) && setGoal(
      goalX,
      goalY,
      goalTheta))) {
      return false;
    }
    return plan(false);
  }

  bool
  PathPlanner::setGoal(const float& x, const float& y, const float& theta)
  {
    if (!mapPtr) {
      //cout << "Distance map hasn't been initialized yet." << endl;
      return false;
    }
    State goal(x, y, theta, NOLEG);
    State footLeft = getFootPose(goal, LEFT);
    State footRight = getFootPose(goal, RIGHT);
    if (plannerEnvironmentPtr->occupied(footLeft) || plannerEnvironmentPtr->occupied(
      footRight)) {
      /* cout << "Goal pose at ("
       << x << ", "
       << y << ", "
       << theta << ")"
       <<" not accessible." << x << ", " << y << ", " << theta << endl;*/
      goalPoseSetUp = false;
      return false;
    }
    goalFootLeft = footLeft;
    goalFootRight = footRight;
    goalPoseSetUp = true;
    /* cout << "Goal pose set to ("
     << x << ", "
     << y << ", "
     << theta << ")"
     << endl;*/
    return true;
  }

  bool
  PathPlanner::setGoal(const State& leftFoot, const State& rightFoot)
  {
    if (plannerEnvironmentPtr->occupied(leftFoot) || plannerEnvironmentPtr->occupied(
      rightFoot)) {
      goalPoseSetUp = false;
      return false;
    }
    goalFootLeft = leftFoot;
    goalFootRight = rightFoot;
    goalPoseSetUp = true;
    return true;
  }

  bool
  PathPlanner::checkGoal()
  {
    if (plannerEnvironmentPtr->occupied(goalFootLeft) || plannerEnvironmentPtr->occupied(
      goalFootRight)) {
      goalPoseSetUp = false;
      return false;
    }
    return true;
  }

  bool
  PathPlanner::setStart(const State& leftFoot, const State& rightFoot)
  {
    if (plannerEnvironmentPtr->occupied(leftFoot) || plannerEnvironmentPtr->occupied(
      rightFoot)) {
      startPoseSetUp = false;
      return false;
    }
    startFootLeft = leftFoot;
    startFootRight = rightFoot;
    startPoseSetUp = true;
    return true;
  }

  bool
  PathPlanner::setStart(const float& x, const float& y, const float& theta)
  {
    if (!mapPtr) {
      //cout << "Distance map hasn't been initialized yet." << endl;
      return false;
    }

    State start(x, y, theta, NOLEG);
    State footLeft = getFootPose(start, LEFT);
    State footRight = getFootPose(start, RIGHT);

    bool success = setStart(footLeft, footRight);
    //if (success)
    /*cout << "Start pose set to ("
     << x << ", "
     << y << ", "
     << theta << ")"
     << endl;*/
    //else
    /*cout << "Start pose at ("
     << x << ", "
     << y << ", "
     << theta << ")"
     <<" not accessible." << x << ", " << y << ", " << theta << endl;*/
    return success;
  }

  void
  PathPlanner::updateMap()
  {
    //! check if the path is still valid on updated map.
    mapPtr->updateDistanceMap();
    plannerEnvironmentPtr->updateMap();
  }

  bool
  PathPlanner::checkPathValidity()
  {
    stateIterT pathStatesIter = path.begin();
    int i = 0;
    for (; pathStatesIter < path.end(); ++pathStatesIter) {
      if (plannerEnvironmentPtr->occupied(*pathStatesIter, true)) {
        //cout << "State: " << i << endl;
        return false;
      }
      ++i;
    }
    return true;
  }

  void
  PathPlanner::updateEnvironment()
  {
    //cout << "Reseting the planning environment." << endl;
    resetTotally();
    plannerEnvironmentPtr->updateMap();
  }

  State
  PathPlanner::getFootPose(const State& robot, Leg leg)
  {
    double shiftX = -sin(robot.getTheta()) * footSeparation / 2.0;
    double shiftY = cos(robot.getTheta()) * footSeparation / 2.0;
    double sign = -1.0;
    if (leg == LEFT) sign = 1.0;
    return State(
      robot.getX() + sign * shiftX,
      robot.getY() + sign * shiftY,
      robot.getTheta(),
      leg);
  }

  bool
  PathPlanner::isNewPath(const vector<int>& newPath)
  {
    if (newPath.size() != planningStatesIds.size()) return true;
    bool unequal = true;
    for (unsigned i = 0; i < newPath.size(); ++i)
      unequal = newPath[i] != planningStatesIds[i] && unequal;
    return unequal;
  }

}
;
