/**
 * @file PlanningBehaviors/RobotStartup/Types/RequestBehavior.h
 *
 * This file declares the class RequestBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"

/** 
 * @class RequestBehavior
 * @brief The class for requesting a particular planning behavior at
 *   startup
 */
class RequestBehavior : public RobotStartup
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  RequestBehavior(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    RobotStartup(planningModule, config, "RequestBehavior")
  {
  }

  /**
   * Destructor
   */
  ~RequestBehavior()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  void loadExternalConfig();
  
private:
  /**
   * Returns the config casted as RequestBehaviorConfigPtr
   */ 
  RequestBehaviorConfigPtr getBehaviorCast();
  
  /**
   * Sets the start posture from configuration. Sets start posture to 
   * crouch in case of an error.
   * 
   * @throws BehaviorException if the given posture in configuration file
   *   is invalid
   */ 
  void setStartPosture() throw (BehaviorException);

  //! Whether the startup has been done
  bool started;
  
  //! Posture to reach on startup before requesting the desired behavior
  PostureState startPosture;
  
  //! The start behavior request which is assigned by the configuration
  //! file PlanningBehaviors.ini
  static unsigned pbRequest;
  
  //! The start posture request which is assigned by the configuration
  //! file PlanningBehaviors.ini
  static string postureRequest;
};

typedef boost::shared_ptr<RequestBehavior> RequestBehaviorPtr;
