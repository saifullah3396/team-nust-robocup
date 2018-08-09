/**
 * @file MotionModule/MovementModule/MovementModule.h
 *
 * This file declares the class for the robot navigation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#pragma once

#include "DebugModule/include/DebugBase.h"
#include "CommModule/include/CommModule.h"
#include "MotionModule/include/PathPlanner/PathPlanner.h"
#include "MotionModule/include/PostureModule/PostureModule.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/HeadTracking/HeadTracking.h"
#include "Utils/include/PostureState.h"
#include "MotionModule/include/MotionConfigs/MBMovementConfig.h"

using namespace Utils;
using namespace PathPlannerSpace;

class MovementModule : public MotionBehavior, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to enable any kind of debugging.
  (int, debug, 1),
  //! Option to send planned footsteps to debugger.
  (int, sendFootsteps, 1),
)
public:
  /**
   * Default constructor for this class
   *
   * @param motionModule: pointer to parent
   */
  MovementModule(MotionModule* motionModule);

  /**
   * Default destructor for this class
   */
  ~MovementModule()
  {
  }

  /**
   * Derived from class Behavior
   */
  void
  initiate();

  /**
   * Derived from class Behavior
   */
  void
  reinitiate();

  /**
   * Derived from class Behavior
   */
  void
  update();

  /**
   * Derived from class Behavior
   */
  void
  finishBehaviorSafely();

  /**
   * Derived from class Behavior
   */
  void
  loadStaticConfig();

  /**
   * Derived from class Behavior
   */
  void
  loadInitialConfig();

  /**
   * Performs a static cast on the BehaviorConfig parent to get the 
   * configuration of this module
   */
  void
  setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);
  boost::shared_ptr<MBMovementConfig>
  getBehaviorCast();

private:

  /// @brief Wrapper for PathPlanner::setGoal.
  bool
  setGoal(const RobotPose2D<float>& goal);
  bool
  findRandomCloseGoal(RobotPose2D<float>& goal);

  /// @brief Wrapper for PathPlanner::setStart.
  bool
  setStart();
  bool
  setStart(const State& left, const State& right);
  void
  checkPathValidity();
  void
  plan();
  void
  replan();

  void
  updateWalk();
  bool
  getFootTransform(Matrix4d& foot, unsigned id);
  bool
  getFootstep(const Matrix4d& from, const State& fromPlanned, const State& to,
    State& footstep);
  void
  getFootstep(const State& from, const State& to, State& footstep);

  bool
  performable(const State& footstep);
  bool
  performable(float stepX, float stepY);

  double accuracyX;
  double accuracyY;
  double accuracyTheta;
  double cellSize;
  int numAngleBins;

  /// Search direction.
  bool forwardSearch;

  //! The start footstep id for tracking footsteps.
  int startStep;

  //! The previous footstep id for tracking footsteps.
  int prevStep;

  //! The current footstep time.
  float startFSTime;

  //! Time taken by one footstep.
  float footStepTime;

  int currentStep;
  float currentFeetMid;
  Vector3f diffFeetMidP;
  Vector3f prevFeetMidP;
  Vector3f currentFeetMidP;
  float prevStepTime;

  bool pathPlanned;
  stateIterT toPlanned;

  double footMaxStepX;
  double footMaxStepY;
  double footMaxStepTheta;
  double footMaxStepInvX;
  double footMaxStepInvY;
  double footMaxStepInvTheta;
  Matrix4d startFrameT;
  Matrix4d prevRFoot;
  Matrix4d prevLFoot;
  vector<pair<double, double> > stepRange;

  PathPlannerPtr pathPlanner;

  //! Pointer to NaoQi internal motion class
  boost::shared_ptr<AL::ALMotionProxy> motionProxy;

  RobotPose2D<float> goal;

  Mat footstepsImage;

  bool proceedToFinish;
  /**
   * Head tracking module object
   * 
   * @var Pointer to head tracking module.
   */
  boost::shared_ptr<HeadTracking> htModule;

  /**
   * Posture module object
   * 
   * @var Pointer to Posture module.
   */
  boost::shared_ptr<PostureModule> postureModule;

  vector<RotatedRect> footRects;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<MovementModule> MovementModulePtr;
