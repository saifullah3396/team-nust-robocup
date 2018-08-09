/**
 * @file PlanningModule/include/PBManager.h
 *
 * This file declares the class PBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#pragma once

#include "BehaviorManager/include/BehaviorManager.h"

//! Forward declaration
class PlanningModule;

/**
 * @class PBManager 
 * @brief Handles the initiation and execution of planning behaviors
 */
class PBManager : public BehaviorManager
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: Pointer to the base planning module
   */
  PBManager(PlanningModule* planningModule);

  /**
   * Destructor
   */
  ~PBManager() {}
private:
  /**
   * Derived from BehaviorManager
   */
  bool
  makeBehavior(BehaviorPtr& behavior, const BehaviorConfigPtr& cfg);
  
  //! Pointer to the base planning behaviors module
  PlanningModule* planningModule;
};
typedef boost::shared_ptr<PBManager> PBManagerPtr;
