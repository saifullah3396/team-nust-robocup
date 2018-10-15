/**
 * @file PlanningModule/include/PlanningBehaviors/ExternalInterface.h
 *
 * This file declares the class ExternalInterface
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/**
 * @class ExternalInterface
 * @brief The class for defining a interface to interact with an external
 *   cognition module
 */
class ExternalInterface : public PlanningBehavior
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   * @param name: Name of this behavior
   */
  ExternalInterface(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config,
    const string& name = "ExternalInterface") :
    PlanningBehavior(planningModule, config, name)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~ExternalInterface()
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
  static boost::shared_ptr<ExternalInterface> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& type);

  //! Child type may or may not use the same behavior config as parent
  virtual void loadExternalConfig() {}
  
private:
  /**
   * Returns the config casted as PBExternalInterfaceConfig
   */ 
  boost::shared_ptr<PBExternalInterfaceConfig> getBehaviorCast();
};

typedef boost::shared_ptr<ExternalInterface> ExternalInterfacePtr;
