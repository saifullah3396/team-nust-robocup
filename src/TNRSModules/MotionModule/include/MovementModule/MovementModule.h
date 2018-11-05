/**
 * @file MotionModule/include/MovementModule/MovementModule.h
 *
 * This file declares the class MovementModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#pragma once

#include "TNRSBase/include/DebugBase.h"
#include "CommModule/include/CommRequest.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "MotionModule/include/PathPlanner/PathPlanner.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBMovementConfig.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/PositionInput.h"

using namespace Utils;
using namespace PathPlannerSpace;

/** 
 * @class MovementModule
 * @brief The base class for defining robot navigation
 */
template <typename Scalar>
class MovementModule : public MotionBehavior<Scalar>, public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to enable any kind of debugging.
    (int, debug, 1),
    //! Option to send planned footsteps to debugger.
    (int, sendFootsteps, 1),
  )
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  MovementModule(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "MovementModule");

  /**
   * Default destructor for this class
   */
  virtual ~MovementModule()
  {
  }

  /**
   * Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<MovementModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
    
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void reinitiate(const BehaviorConfigPtr& cfg);
  void update();
  void finish();  
    
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig();
  
  
private:
  /**
	 * Returns the cast of config to MBBallThrowConfigPtr
	 */
  MBMovementConfigPtr getBehaviorCast();

  /**
   * Finds the starting feet placement and sets the starting pose
   * in path planner
   * 
   * @return true if starting pose is set up
   */
  bool setStart();
  
  /**
   * Sets the starting pose in path planner
   * 
   * @param left: Left foot state in the environment
   * @param right: Right foot state in the environment
   * 
   * @return true if starting pose is set up
   */
  bool setStart(const State& left, const State& right);

  /**
   * Sets the goal position to reach in the environment
   * 
   * @param goal: Desired goal robot pose
   * 
   * @return true if goal is successfully set up
   */ 
  bool setGoal(const RobotPose2D<Scalar>& goal);
  
  /**
   * Finds the closest possible goal in the direction from initial pose
   * to goal pose
   * 
   * @param goal: Desired goal robot pose
   * 
   * @return true if a possible goal is found
   */ 
  bool findPossibleGoal(RobotPose2D<Scalar>& goal);

  /**
   * Gets the feet transformation frames in environment from robot pose
   * and kinematics info.
   * 
   * @param foot: Transformation frame to be updated
   * @param id: Foot id defined in path planner
   * 
   * @return true if a valid transformation matrix is found
   */ 
  bool getFootTransform(Matrix<Scalar, 4, 4>& foot, const unsigned& id);
  
  /**
   * Checks whether the current path is no more valid and replans if true.
   */ 
  void replanIfPathInvalid();

  /**
   * Wrapper for pathPlanner->plan()
   */ 
  void plan();
  
  /**
   * Wrapper for pathPlanner->replan()
   */ 
  void replan();

  /**
   * Uses the current planned path 
   */ 
  void executeWalk();
  
  /**
   * Sets up the robot walk using NaoQi's motion proxy setFootSteps()
   */ 
  void setupNaoQiWalk();
  
  /**
   * Send planned footsteps to clients through the comm module
   */ 
  void sendFootStepsData();
  
  /// Search direction
  bool forwardSearch;

  //! Footsteps image matrix
  Mat footstepsImage;

  //! Footsteps rects to be drawn in the image matrix
  vector<RotatedRect> footRects;

  //! Path is planned or not
  bool pathPlanned;
  
  //! Projected position updates if a step is executed
  vector<PositionInput<float> > stepPositionUpdates;
  
  //! Transformation frame of the right foot in previous cycle
  Matrix<Scalar, 4, 4> prevRFoot;
  
  //! Transformation frame of the left foot in previous cycle
  Matrix<Scalar, 4, 4> prevLFoot;
  
  //! Current goal pose
  RobotPose2D<Scalar> goal;
  
  //! If robot is in walk
  bool inWalk;
  
  //! Time taken by one step
  Scalar footStepTime;
  
  //! The current footstep start time
  Scalar footStepStartTime;
  
  //! The start footstep id for tracking footsteps
  int startStep;

  //! Current step index in planned footsteps
  unsigned currentStep;

  //! The previous footstep id for tracking footsteps
  int prevStep;
  
  //! The path planner object
  PathPlannerPtr pathPlanner;
  
  //! Path states iterator
  StateIterT toPlanned;
  
  //! Current state of this behavior
  unsigned behaviorState;
  
  /**
   * States of this behavior
   * 
   * @enum BehaviorState
   */ 
  enum BehaviorState
  {
    setPosture,
    move,
    stop
  };
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<MovementModule<MType> > MovementModulePtr;
