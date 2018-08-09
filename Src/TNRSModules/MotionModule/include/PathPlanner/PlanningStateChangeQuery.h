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

#include <vector>
#include "Resources/SbplLib/include/headers.h"

using namespace std;

namespace PathPlannerSpace
{

  class PlanningStateChangeQuery : public StateChangeQuery
  {
  public:
    PlanningStateChangeQuery(const vector<int>& neighbors);
    virtual
    ~PlanningStateChangeQuery();

    const vector<int>*
    getPredecessors() const;
    const vector<int>*
    getSuccessors() const;

  private:
    const vector<int>& neighbors;
  };

}
;
