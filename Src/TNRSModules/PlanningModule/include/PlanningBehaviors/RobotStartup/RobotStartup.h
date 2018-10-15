/**
 * @file PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h
 *
 * This file declares the class RobotStartup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "Utils/include/ConfigMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"

static const int BUTTON_DELAY = 30;

/** 
 * @class RobotStartup
 * @brief The class for defining the stratup behavior of the robot when 
 *   the code first initiates on the robot.
 */
class RobotStartup : public PlanningBehavior
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   * @param name: Name of this behavior
   */
  RobotStartup(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config,
    const string& name = "RobotStartup") :
    PlanningBehavior(planningModule, config, name)
  {
  }

  /**
   * Destructor
   */
  ~RobotStartup()
  {
  }
  
  /**
   * Returns its own child based on the given type
   * 
   * @param planningModule: Pointer to base planning module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<RobotStartup> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& cfg);
    
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}

private:
  /**
   * Returns the cast of config as PBStartupConfigPtr
   */ 
  PBStartupConfigPtr getBehaviorCast();
};

typedef boost::shared_ptr<RobotStartup> RobotStartupPtr;
