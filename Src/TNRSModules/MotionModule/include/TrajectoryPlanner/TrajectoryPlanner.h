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

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/TrajectoryPlanner/CubicSpline.h"
#include "Utils/include/HardwareIds.h"

/** 
 * @class TrajectoryPlanner
 * @brief The class for creating smooth trajectories for the 
 *   joints based on requried conditions.
 */
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
  ;

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
  cartesianPlanner(vector<vector<float> >& traj, const unsigned& chainIndex,
    const Matrix4f& endEffector, const vector<Matrix4f>& cPoses,
    const vector<VectorXf>& cBoundVels, const bool& solveInitPose = false,
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
  jointsPlanner(vector<vector<float> >& traj, const unsigned& chainIndex,
    const MatrixXf& jointPositions, const MatrixXf& jointBoundVels,
    const VectorXf& knots);

  boost::shared_ptr<KinematicsModule>
  getKinematicsModule()
  {
    return kM;
  }

  /*bool step(Vector3f &splinePosition, Vector3f &splineVelocity, Vector3f &splineAcceleration);
   void setStepSize(float stepSize){this->stepSize = stepSize;}	
   void setTrajectoryTime(float trajectoryTime);
   void setReferencePoints(vector<Vector3f> trajectoryRefPoints);
   void setTrajectoryKnots();
   void defineViaPoints(Vector3f initialVelocity, Vector3f finalVelocity);
   float getTrajStep() {return trajStep;}*/
private:
  //! Size of the timestep.
  float stepSize;
  /*
   vector<Vector3f> trajectoryRefPoints;
   fstream controlPointsLog;*/

  //! Kinematics module object.
  KinematicsModulePtr kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<TrajectoryPlanner> TrajectoryPlannerPtr;
