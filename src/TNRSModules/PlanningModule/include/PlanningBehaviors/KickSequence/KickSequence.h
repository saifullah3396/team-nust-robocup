/**
 * @file PlanningModule/include/PlanningBehaviors/KickSequence.h
 *
 * This file declares the class KickSequence
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/**
 * @class KickSequence
 * @brief The class for defining different kinds of kick sequences 
 *   possible
 */
class KickSequence : public PlanningBehavior
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   * @param name: Name of this behavior
   */
  KickSequence(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config,
    const string& name = "KickSequence") :
    PlanningBehavior(planningModule, config, name)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~KickSequence()
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
  static boost::shared_ptr<KickSequence> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& type);

  //! Child type may or may not use the same behavior config as parent
  virtual void loadExternalConfig() {}

private:
  /**
   * Returns the config casted as PBKickSequenceConfig
   */ 
  boost::shared_ptr<PBKickSequenceConfig> getBehaviorCast();
};

typedef boost::shared_ptr<KickSequence> KickSequencePtr;
