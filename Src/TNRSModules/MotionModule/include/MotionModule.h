/**
 * @file MotionModule/include/MotionModule.h
 *
 * This file declares the class MotionModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <alproxies/almotionproxy.h>
#include "BehaviorManager/include/BehaviorManager.h"
#include "MotionModule/include/MBManager.h"
#include "MotionModule/include/MotionRequest.h"
#include "MotionModule/include/MTypeHeader.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "BehaviorManager/include/BehaviorInfo.h"
//#include "Utils/include/ActuatorRequests.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/BallInfo.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/OccupancyMap.h"

//! Forward declarations
template <typename Scalar> class FallDetector;
typedef boost::shared_ptr<FallDetector<MType> > FallDetectorPtr;
template <typename Scalar> class KinematicsModule;
typedef boost::shared_ptr<KinematicsModule<MType> > KinematicsModulePtr;
template <typename Scalar> class TrajectoryPlanner;
typedef boost::shared_ptr<TrajectoryPlanner<MType> > TrajectoryPlannerPtr;
namespace PathPlannerSpace
{
  class PathPlanner;
  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;
}
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;

/**
 * @class MotionModule
 * @brief A class for handling all activities related to the robot 
 *   kinematics, dynamics, motion.
 */
class MotionModule : public BaseModule
{
public:
  /**
   * Definition of input connector and variables for this module
   */ 
  CREATE_INPUT_CONNECTOR(MotionInput,
    (int, motionThreadPeriod),
    (vector<float>, jointPositionSensors),
    (vector<float>, jointStiffnessSensors),
    (vector<float>, inertialSensors),
    (vector<float>, fsrSensors),
    (vector<float>, touchSensors),
    (BallInfo, ballInfo),
    (GoalInfo, goalInfo),
    (RobotPose2D<float>, robotPose2D),
    (OccupancyMap, occupancyMap),
    (int, nFootsteps),
    (bool, landmarksFound),
  )

  /**
   * Definition of output connector and variables for this module
   */ 
  CREATE_OUTPUT_CONNECTOR(MotionOutput,
    (Matrix4f, upperCamInFeet),
    (Matrix4f, lowerCamInFeet),
    (Matrix4f, lFootOnGround),
    (Matrix4f, rFootOnGround),
    (PostureState, postureState),
    (bool, robotFallen),
    (RobotPose2D<float>, moveTarget),
    (Point2f, kickTarget),
    (vector<RotatedRect>, footRects),
    (bool, robotInMotion),
    (BehaviorInfo, mBehaviorInfo),
  )

  /**
   * Constructor
   *
   * @param parent: parent: Pointer to parent module
   * @param motionProxy: Pointer to NaoQi's motion proxy
   */
  MotionModule(void* parent, ALMotionProxyPtr motionProxy);

  /**
   * Destructor
   */
  ~MotionModule()
  {
		delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();

  /**
   * Gets the kinematics module
   *
   * @return KinematicsModulePtr
   */
  KinematicsModulePtr getKinematicsModule();

  /**
   * Gets the trajectory planner module
   *
   * @return TrajectoryPlannerPtr
   */
  TrajectoryPlannerPtr getTrajectoryPlanner();
  
  /**
   * Gets the motion proxy module
   *
   * @return MotionProxyPtr
   */
  ALMotionProxyPtr getSharedMotionProxy();

  /**
   * Gets the pointer to path planner
   *
   * @return PathPlannerPtr
   */
  PathPlannerSpace::PathPlannerPtr getPathPlanner();

private:
	//! Motion behaviors manager shared object
  MBManagerPtr mbManager;

  //! Fall detector module object
  FallDetectorPtr fallDetector;

  //! Kinematics module object
  KinematicsModulePtr kinematicsModule;

  //! Trajectory planner module object
  TrajectoryPlannerPtr trajectoryPlanner;

  //! Pointer to NaoQi internal motion class
  ALMotionProxyPtr motionProxy;

  //! Path planner object
  PathPlannerSpace::PathPlannerPtr pathPlanner;
};
