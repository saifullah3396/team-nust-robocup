/**
 * @file PlanningModule/include/PlanningBehavior.h
 *
 * This file declares the class PlanningBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "BehaviorManager/include/Behavior.h"
#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PlanningConfigs.h"
#include "SBModule/include/SBConfigs.h"
#include "MotionModule/include/MotionConfigs/MBConfig.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/StiffnessState.h"

/**
 * @class PlanningBehavior
 * @brief A base class for all types of planning behaviors
 */
class PlanningBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param pModule: Pointer to the parent planning module
   * @param config: Behavior configuration
   * @param name: Behavior name
   */
  PlanningBehavior(
    PlanningModule* pModule,
    const BehaviorConfigPtr& config,
    const string& name = "Not assigned.") :
      Behavior(config, name),
      MemoryBase(pModule), 
      pModule(pModule), 
      sRequestTime(0.f),
      mRequestTime(0.f) 
    //penaliseMotion(false), 
    //waitForUnpenalise(false)
  {
    cycleTime = pModule->getPeriodMinMS() / 1000.f;
  }
  
  /**
   * Destructor
   */
  virtual
  ~PlanningBehavior()
  {
  }
  
protected:
  /**
   * Updates the current posture and stiffness state for this behavior
   */ 
  void updatePostureAndStiffness();
  
  /**
   * Sends the requests to change robot posture and stiffness states to
   * the given ones
   * 
   * @param desPosture: Desired posture
   * @param desPosture: Desired stiffness
   * 
   * @return true if both are set successfully else false
   */
  bool setPostureAndStiffness(
    const PostureState& desPosture, 
    const StiffnessState& desStiffness, 
    const float& postureTime = 2.f);
  
  /**
   * Sends the request to kill the running static behavior
   */ 
  void killStaticBehavior();
  
  /**
   * Sends the request to kill the running motion behavior
   */ 
  void killMotionBehavior();
  
  /**
   * Returns true if a static behavior is running
   * 
   * @return bool
   */ 
  bool sbInProgress();
  
  /**
   * Returns true if a motion behavior is running
   * 
   * @return bool
   */ 
  bool mbInProgress();
  
  /**
   * Sets up a request to call a static behavior
   * 
   * @param config: Configuration of the required static behavior
   */ 
  void setupSBRequest(const SBConfigPtr& config);
  
  /**
   * Sets up a request to call a motion behavior
   * 
   * @param config: Configuration of the required motion behavior
   */ 
  void setupMBRequest(const MBConfigPtr& config);

  /**
   * Returns true if a static or motion behavior requests has been
   * sent but not accepted
   * 
   * @return bool
   */ 
  bool requestInProgress();

  //! Base PlanningModule object pointer.
  PlanningModule* pModule;
  
  //! Configuration of the currently running motion behavior
  BehaviorConfigPtr mbRunning;

  //! Configuratoin of the currently running static behavior
  BehaviorConfigPtr sbRunning;

  //! Last motion behaviors requested
  BehaviorRequestPtr lastMotionRequest;

  //! Last static behaviors requested
  BehaviorRequestPtr lastStaticRequest;

  //! Running cycle time of this behavior
  float cycleTime;
  
  //! Current posture state
  PostureState posture;

  //! Current stiffness state
  StiffnessState stiffness;

private:
  /**
   * Checks whether the behavior is still in progress based on incoming
   * info
   * 
   * @param info: The behavior info
   * 
   * @return true if behavior is still running
   */ 
  bool behaviorInProgress(const BehaviorInfo& info);

  /**
   * Returns true if the given request has not been received. If 
   * received and accepted, the acceptedBehavior is assigned the config 
   * of the given request. In case of rejection, the request is deleted
   * 
   * @param req: Request to be checked
   * @param acceptedBehavior: The configuration of the accepted behavior
   * @param info: The feed back information about behavior
   * @param requestStartTime: The time this request was sent at
   * 
   * @return bools
   */ 
  bool requestInProgress(
    BehaviorRequestPtr& req,
    BehaviorConfigPtr& acceptedBehavior,
    const BehaviorInfo& feedback,
    const float& requestStartTime);

  //! Time taken by the motion request in progress
  float mRequestTime;

  //! Time taken by the static behavior request in progress
  float sRequestTime;
  
  //! Maximum allowed time for a motion or behavior request to be 
  //! processed
  static constexpr float maxRequestTimeout = 2.f;

  /**
   * Resets the robot localization
   * 
   * @return true if the req is already set.
   */ 
  //bool reqRelocalization();

  //bool goToTarget(const RobotPose2D<float>& target, const float& positionTolX,
  //  const float& positionTolY, const float& positionTolTheta,
  //  const bool& trackBall = false);
  //bool penalisedRobot();

  //! Whether robot is supposed to get into penalised posture
 // bool penaliseMotion;

  //! Whether to wait to get unpenalised
//  bool waitForUnpenalise;

  //! Time to wait until robot is unpenalised
  //float timeTillUnpenalised;

  //! Penalisation starting time
  //float timePenalised;
};
