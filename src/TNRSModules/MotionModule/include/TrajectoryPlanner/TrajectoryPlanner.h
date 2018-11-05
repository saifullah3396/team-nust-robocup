/**
 * @file MotionModule/TrajectoryPlanner/TrajectoryPlanner.h
 *
 * This file declares a class for creating smooth trajectories for the 
 * joints based on requried conditions.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Aug 2017  
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/HardwareIds.h"

class MotionModule;

/** 
 * @class TrajectoryPlanner
 * @brief The class for creating smooth trajectories for the 
 *   joints based on requried conditions.
 */
template <typename Scalar>
class TrajectoryPlanner
{
public:
  /**
   * Default constructor for this class
   * 
   * @param motionModule: base class.
   */
  TrajectoryPlanner(MotionModule* motionModule);

  /**
   * Default destructor for this class.
   */
  ~TrajectoryPlanner()
  {
  }

  /**
   * Setting up the trajectory planner for a certain type.
   * 
   * @param chainIndex: The chain for which trajectories 
   * 	 are being planned. 
   * @param endEffector: End effector transformation for 
   *   that chain from its previous endpoint. 
   * @param cPoses: The cartesian poses of the 
   *   end effector needed in given sequence (1,2,...., N).
   * @param cVels: The cartesian initial and final 
   *   velocities of the end effector needed.
   * 
   * @return bool: planning possible or not?
   */
  bool
  cartesianPlanner(vector<vector<Scalar> >& traj, const unsigned& chainIndex,
    const Matrix<Scalar, 4, 4> & endEffector, const vector<Matrix<Scalar, 4, 4> >& cPoses,
    const vector<Matrix<Scalar, Dynamic, 1> >& cBoundVels, const bool& solveInitPose = false,
    const bool& timeOpt = false);

  /**
   * Setting up the trajectory planner for a certain type.
   * 
   * @param chainIndex: The chain for which trajectories 
   * 	 are being planned. 
   * @param jPoses: The joint positions of the 
   *   end effector needed.
   */
  void
  jointsPlanner(vector<vector<Scalar> >& traj, const unsigned& chainIndex,
    const Matrix<Scalar, Dynamic, Dynamic> & jointPositions, const Matrix<Scalar, Dynamic, Dynamic> & jointBoundVels,
    const Matrix<Scalar, Dynamic, 1> & knots);

  KinematicsModulePtr
  getKinematicsModule()
  {
    return kM;
  }

  /*bool step(Vector3f &splinePosition, Vector3f &splineVelocity, Vector3f &splineAcceleration);
   void setStepSize(Scalar stepSize){this->stepSize = stepSize;}	
   void setTrajectoryTime(Scalar trajectoryTime);
   void setReferencePoints(vector<Vector3f> trajectoryRefPoints);
   void setTrajectoryKnots();
   void defineViaPoints(Vector3f initialVelocity, Vector3f finalVelocity);
   Scalar getTrajStep() {return trajStep;}*/
private:
  //! Size of the timestep.
  Scalar stepSize;
  /*
   vector<Vector3f> trajectoryRefPoints;
   fstream controlPointsLog;*/

  //! Kinematics module object.
  KinematicsModulePtr kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<TrajectoryPlanner<MType> > TrajectoryPlannerPtr;
