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

#include "MotionModule/include/PathPlanner/State.h"

namespace PathPlannerSpace
{

  State::State() :
    x(0.0), y(0.0), theta(0.0), leg(NOLEG)
  {
  }

  State::State(double x, double y, double theta, Leg leg) :
    x(x), y(y), theta(theta), leg(leg)
  {
  }

  State::~State()
  {
  }

  bool
  State::operator ==(const State& s2) const
  {
    return (fabs(x - s2.getX()) < FLOAT_CMP_THR && fabs(y - s2.getY()) < FLOAT_CMP_THR && fabs(
      angles::shortestAngularDistance(theta, s2.getTheta())) < FLOAT_CMP_THR && leg == s2.getLeg());
  }

  bool
  State::operator !=(const State& s2) const
  {
    return not (*this == s2);
  }

}
;
