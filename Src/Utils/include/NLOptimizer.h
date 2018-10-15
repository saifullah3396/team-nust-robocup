/**
 * @file Utils/include/NLOptimizer.h
 *
 * This file declares the class NLOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include <numeric>
#include <vector>
#include <nlopt/nlopt.hpp>

using namespace std;

/**
 * @class NLOptimizer
 * @brief The base class for defining an optimization problem using 
 *   nlopt
 */
class NLOptimizer
{
public:
  /**
   * Constructor 
   */
  NLOptimizer() : success(false) {}
  
  /**
   * Destructor
   */
  virtual ~NLOptimizer()
  {
  }
  
  /**
   * The function that defines and carries out the optimization process
   */ 
  virtual void optDef() = 0;
  
  /**
   * Returns true if a solution is found
   */ 
 bool getSuccess() { return success; }
 
protected:
  /**
   * NLOPT based function for defining cost function
   */
  virtual double
  costFunction(
    const vector<double>& vars, 
    vector<double>& grad,
    void *data) = 0;
  
  /**
   * NLOPT based function for defining inequality constraints
   */
  virtual void
  ineqConstraints(unsigned nCons, double *result, unsigned nVars,
    const double* vars, double* grad, void* data) {}
  
  /**
   * Wraps to NLOPT based function solver function within the class.
   */
  static double
  objWrapper(const vector<double> &vars, vector<double> &grad,
    void *data)
  {
    NLOptimizer *obj = static_cast<NLOptimizer*>(data);
    return obj->costFunction(vars, grad, data);
  }
  
  /**
   * Wraps to NLOPT based function ineq constraints defining function
   * within the class.
   */
  static void
  ineqWrapper(unsigned nCons, double *result, unsigned nVars,
    const double* vars, double* grad, void* data)
  {
    NLOptimizer *obj = static_cast<NLOptimizer*>(data);
    return obj->ineqConstraints(nCons, result, nVars, vars, grad, data);
  }
  
  //! Whether the optimization has been finished and succeeded or not
  bool success;
};
