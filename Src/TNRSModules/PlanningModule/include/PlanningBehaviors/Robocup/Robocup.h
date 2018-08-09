/**
 * @file PlanningModule/PlanningBehaviors/Robocup.h
 *
 * This file declares the class Robocup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "PlanningModule/include/PlanningBehavior.h"

/** 
 * @class Robocup
 * @brief Class for defining the robot behavior for robocup gameplay.
 */
class Robocup : public PlanningBehavior
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  Robocup(PlanningModule* planningModule,
    const string& name = "Robocup") :
    PlanningBehavior(planningModule, name)
  {
    fallRecovery = false;
    readyToGetup = false;
    wasPenalised = false;
    lastKickTarget = Point2f(0, 0);
  }

  /**
   * Default destructor for this class.
   */
  virtual
  ~Robocup()
  {
  }
  
  static boost::shared_ptr<Robocup> getType(
    PlanningModule* planningModule, const unsigned& type);

  //! Must be defined for specific type
  virtual void initiate() = 0;
  virtual void update() = 0;
  virtual void finishBehaviorSafely() = 0;
  
  //! Child type may or may not use the same behavior config as parent
  virtual void loadExternalConfig() {}
  virtual void setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);
private:
  boost::shared_ptr<PBRobocupConfig> getBehaviorCast();
protected:
  void setRobotIntention();
  void setRobotSuggestions();
  void fallenRobotAction();
  void fallRecoveryAction();
  void getupFront();
  void getupBack();
  void getupSit();

  bool wasPenalised;
  bool fallRecovery;
  bool readyToGetup;
  Point2f lastKickTarget;
};

typedef boost::shared_ptr<Robocup> RobocupPtr;
