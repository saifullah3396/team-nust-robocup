/**
 * @file PlanningBehaviors/KickSequence/MotionEquationSolver.cpp
 *
 * This file implements the class MotionEquationSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/MotionModule.h" 
#include "Utils/include/Solvers/MotionEquationSolver.h"

double
MotionEquationSolver::costFunction(const vector<double>& vars, vector<double>& grad,
  void *data)
{
  //cout << "Finding f.. " << endl;
  double f = 0;
  auto pos = solvePosition(vars[0]);
  //cout << "time: " << vars[0] << endl;
  //cout << "pos: " << pos << endl;
  f = (pos - target).norm();
  //cout << "f: " << f << endl;
  return f;
}

void
MotionEquationSolver::optDef()
{
  //!Objective function to minimize the distance between target
  //!Hessian for this objective function is unknown.
  //!Gradient for this function is unknown.
  //!1 variable; time it will take to reach the desired target
  nlopt::opt opt(nlopt::LN_COBYLA, 1);
  vector<double> lb(1), ub(1), var0, constraintTols;
  lb[0] = 0.0; //! Lower bound for time
  ub[0] = 10.0; //! Upper bound for time
  for (int i = 0; i < lb.size(); ++i)
    var0.push_back(lb[i]);

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(MotionEquationSolver::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  //cout << "Starting optimization... " << endl;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
   // cout << "nlopt failed!" << endl;
  } else {
    timeToReach = var0[0];
    distFromTarget = minf;
    endPosition = solvePosition(timeToReach);
    endVelocity = solveVelocity(timeToReach);
    success = true;
    //cout << "Found minimum at time:" << var0[0] << endl << "with dist:" << endl << minf << endl;
  }
}

