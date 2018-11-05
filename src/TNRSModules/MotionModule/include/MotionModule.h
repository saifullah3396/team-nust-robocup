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
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "BehaviorManager/include/BehaviorManager.h"
#include "BehaviorManager/include/BehaviorInfo.h"
#include "ControlModule/include/HardwareLayer.h"
#include "MotionModule/include/MBManager.h"
#include "MotionModule/include/MotionRequest.h"
#include "MotionModule/include/MTypeHeader.h"
#include "TNRSBase/include/BaseIncludes.h"
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
template <typename Scalar> class MotionGenerator;
typedef boost::shared_ptr<MotionGenerator<MType> > MotionGeneratorPtr;
template <typename Scalar> class TrajectoryPlanner;
typedef boost::shared_ptr<TrajectoryPlanner<MType> > TrajectoryPlannerPtr;
namespace PathPlannerSpace
{
  class PathPlanner;
  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;
}
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

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
    (vector<float>, jointStiffnessSensors),
    (vector<float>, touchSensors),
    (BallInfo, ballInfo),
    (GoalInfo, goalInfo),
    (RobotPose2D<float>, robotPose2D),
    (OccupancyMap, occupancyMap),
    (bool, landmarksFound),
  )

  /**
   * Definition of output connector and variables for this module
   */ 
  CREATE_OUTPUT_CONNECTOR(MotionOutput,
    (vector<float>, jointPositionSensors),
    (vector<float>, inertialSensors),
    (vector<float>, fsrSensors),
    (int, nFootsteps),
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
   * @param memoryProxy: Pointer to NaoQi's memory proxy
   * @param dcmProxy: Pointer to NaoQi's DCM proxy
   * @param motionProxy: Pointer to NaoQi's motion proxy
   */
  MotionModule(
   void* parent,
   const ALMemoryProxyPtr& memoryProxy,
   const ALDCMProxyPtr& dcmProxy,
   const ALMotionProxyPtr& motionProxy);

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
   * Gets the motion generator module
   *
   * @return MotionGeneratorPtr
   */
  MotionGeneratorPtr getMotionGenerator();

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
  /**
   * Sets up the motion sensors
   */
  void setupSensors();

  /**
   * Sets up the motion actuators
   */
  void setupActuators();

  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   */
  void sensorsUpdate();

  /**
   * Sends the requested actuator commands to NaoQi DCM for execution
   */
  void actuatorsUpdate();

	//! Motion behaviors manager shared object
  MBManagerPtr mbManager;

  //! Fall detector module object
  FallDetectorPtr fallDetector;

  //! Kinematics module object
  KinematicsModulePtr kinematicsModule;

  //! Motion generator module object
  MotionGeneratorPtr motionGenerator;

  //! Trajectory planner module object
  TrajectoryPlannerPtr trajectoryPlanner;

  //! Pointer to NaoQi internal motion class
  ALMotionProxyPtr motionProxy;

  //! Path planner object
  PathPlannerSpace::PathPlannerPtr pathPlanner;

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Vector of pointer to ActuatorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers;

  //! Pointer to NaoQi internal memory proxy
  ALMemoryProxyPtr memoryProxy;

  //! Pointer to NaoQi internal dcm proxy
  ALDCMProxyPtr dcmProxy;

  enum class MotionSensors : unsigned {
    JOINT_POSITION,
    INERTIAL,
    FSR,
    COUNT
  };

  enum class MotionActuators : unsigned {
    JOINT_ACTUATORS,
    COUNT
  };
};
