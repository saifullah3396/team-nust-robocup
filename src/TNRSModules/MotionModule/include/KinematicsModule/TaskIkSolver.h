/**
 * @file MotionModule/include/KinematicsModule/TaskIkSolver.h
 *
 * This file declares the class TaskIkSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/MathsUtils.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"

using namespace std;

namespace qpOASES {
  class SQProblem;
}
template <typename Scalar>
class KinematicsModule;

/**
 * @class TaskIkSolver
 * @brief A class to solve for the best end-effector for a kick with 
 *   maximimum product of virtual mass and possible velocity hence 
 *   momentum.
 */
template <typename Scalar>
class TaskIkSolver
{
public:
  /**
   * Constructor
   * 
   * @param km: Pointer to kinematics module 
   */
   TaskIkSolver(
     const boost::shared_ptr<KinematicsModule<Scalar> >& km,
     const unsigned& maxIterations,
     const vector<bool>& activeJoints,
     const Scalar& dt = 5e-3,
     const Scalar& costThres = 1e-10,
     const Scalar& costVarThres = 1e-5);
  
  /**
   * Destructor
   */
  ~TaskIkSolver();

  /**
   * Solve the ik problem using qpoases
   *
   * @return joint values
   */
  Eigen::Matrix<Scalar, Dynamic, 1> solve();
  
  /**
   * Update step for the ik solver
   *
   */ 
  Eigen::Matrix<Scalar, Dynamic, 1> step();
  
  /**
   * Computes the total integrated cost for all the tasks and returns it
   * 
   * @return Scalar
   */ 
  Scalar computeCost();
  
  /**
   * Adds a task to the vector of tasks to be solved
   * 
   * @param task: Added task
   */ 
  void addTask(const MotionTaskPtr& taskPtr)
    { this->tasks.push_back(taskPtr); }

  /**
   * Set active joints
   *
   * @param activeJoints
   */
  void setActiveJoints(const vector<bool>& activeJoints)
    { this->activeJoints = activeJoints; }
  
private:
  //! Pointer to SQP problem
  qpOASES::SQProblem* qp;
  
  //! QP Problem matrices
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> P; // 1/2 x' P x
  Eigen::Matrix<Scalar, Dynamic, 1> v; // -K x'v
  
  //! QP Constraint matrices
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> G; // Gx <= h
  Eigen::Matrix<Scalar, Dynamic, 1> h; // Gx <= h

  //! Velocity constraints for each dof
  Eigen::Matrix<Scalar, Dynamic, 1> maxV, minV;

  //! Type of joints to be used for dof computations
  JointStateType type;

  //! Number of active joints
  unsigned nDof;
  
  //! Contains a list of booleans telling if the joint of given index is active or not
  vector<bool> activeJoints;

  //! Minmax dof positional limit gain
  double dofPLimitGain;

  //! Time step size for solving the ik
  Scalar dt;

  //! Maximum number of iterations to solve ik
  unsigned maxIterations;
  
  //! Whether the solver has been initiated once
  bool initiated;

  //! If cost is below this value then stop
  Scalar costThres;
  
  //! Threshold for checking if cost variation is below this then stop
  Scalar costVarThres;
  
  //! A vector of tasks to be solved
  vector<MotionTaskPtr> tasks;
  
  //! Kinematics module
  boost::shared_ptr<KinematicsModule<Scalar> > km;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
