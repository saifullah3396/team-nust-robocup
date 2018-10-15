/**
 * @file PlanningModule/include/PlanningConfigs.h
 *
 * This file defines all the planning behavior configurations
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "BehaviorManager/include/BehaviorConfig.h"

/**
 * @struct PBConfig
 * @brief Base planning behavior configuration
 */
struct PBConfig : BehaviorConfig 
{
  /**
   * Constructor
   * 
   * @param id: Id of the behavior
   * @param maxRunTime: Maximum allowed time for the behavior
   * @param type: Type of the behavior
   */ 
  PBConfig(
    const PBIds& id, 
    const float& maxRunTime,
    const int& type) : 
  BehaviorConfig((unsigned)id, BaseBehaviorType::PLANNING, maxRunTime, type)
  {}
};
typedef boost::shared_ptr<PBConfig> PBConfigPtr;

/**
 * @struct PBStartupConfig
 * @brief The software startup behavior configuration
 */
struct PBStartupConfig : PBConfig
{
  /**
   * Constructor
   * 
   * @param type: Startup behavior type
   */ 
  PBStartupConfig(const PBStartupTypes& type) :
    PBConfig(PBIds::STARTUP, 10.f, (int)type)
  {
  }
};
typedef boost::shared_ptr<PBStartupConfig> PBStartupConfigPtr;

/**
 * @struct RequestBehaviorConfig
 * @brief The config for RequestBehavior startup type behavior
 */
struct RequestBehaviorConfig : PBStartupConfig
{
  /**
   * Constructor
   */ 
  RequestBehaviorConfig() : 
    PBStartupConfig(PBStartupTypes::REQUEST_BEHAVIOR)
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<RequestBehaviorConfig> RequestBehaviorConfigPtr;

/**
 * @struct PBRobocupConfig
 * @brief The robocup gameplay behavior configuration
 */
struct PBRobocupConfig : PBConfig
{
  /**
   * Constructor
   * 
   * @param type: Robocup behavior type
   */ 
  PBRobocupConfig(const PBRobocupTypes& type) :
    PBConfig(PBIds::ROBOCUP, 3600, (int)type)
  {
  }
};
typedef boost::shared_ptr<PBRobocupConfig> PBRobocupConfigPtr;

/**
 * @struct RobocupSetupConfig
 * @brief The robocup gameplay setup configuration
 */
struct RobocupSetupConfig : PBRobocupConfig
{
  /**
   * Constructor
   */ 
  RobocupSetupConfig() : 
    PBRobocupConfig(PBRobocupTypes::ROBOCUP_SETUP) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<RobocupSetupConfig> RobocupSetupConfigPtr;

/**
 * @struct GoalKeeperConfig
 * @brief The goal keeper configuration
 */
struct GoalKeeperConfig : PBRobocupConfig
{
  /**
   * Constructor
   */ 
  GoalKeeperConfig() :
    PBRobocupConfig(PBRobocupTypes::GOAL_KEEPER) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<GoalKeeperConfig> GoalKeeperConfigPtr;

/**
 * @struct DefenderConfig
 * @brief The defender configuration
 */
struct DefenderConfig : PBRobocupConfig
{
  /**
   * Constructor
   */ 
  DefenderConfig() :
    PBRobocupConfig(PBRobocupTypes::DEFENDER) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<DefenderConfig> DefenderConfigPtr;

/**
 * @struct AttackerConfig
 * @brief The attacker configuration
 */
struct AttackerConfig : PBRobocupConfig
{
  /**
   * Constructor
   */ 
  AttackerConfig() :
    PBRobocupConfig(PBRobocupTypes::ATTACKER) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<AttackerConfig> AttackerConfigPtr;

/**
 * @struct PenaltiesConfig
 * @brief The robocup penalties behavior configuration
 */
struct PenaltiesConfig : PBRobocupConfig
{
  /**
   * Constructor
   */ 
  PenaltiesConfig() :
    PBRobocupConfig(PBRobocupTypes::PENALTIES) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<PenaltiesConfig> PenaltiesConfigPtr;

/**
 * @struct PBKickSequenceConfig
 * @brief The kick sequence configuration
 */
struct PBKickSequenceConfig : PBConfig
{
  /**
   * Constructor
   * 
   * @param type: Kick sequence behavior type
   */ 
  PBKickSequenceConfig(
    const PBKickSequenceTypes& type = 
      PBKickSequenceTypes::BALL_INTERCEPT) :
    PBConfig(PBIds::KICK_SEQUENCE, 9999.f, (int) type) // to be fixed
  {
  }
};
typedef boost::shared_ptr<PBKickSequenceConfig> PBKickSequenceConfigPtr;

/**
 * @struct BallInterceptConfig
 * @brief The ball intercepting behavior configuration
 */
struct BallInterceptConfig : PBKickSequenceConfig
{
  /**
   * Constructor
   */ 
  BallInterceptConfig() :
    PBKickSequenceConfig(PBKickSequenceTypes::BALL_INTERCEPT) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<BallInterceptConfig> BallInterceptConfigPtr;

/**
 * @struct FindAndKickConfig
 * @brief The ball finding and kicking behavior configuration
 */
struct FindAndKickConfig : PBKickSequenceConfig
{
  /**
   * Constructor
   */ 
  FindAndKickConfig() :
    PBKickSequenceConfig(PBKickSequenceTypes::FIND_AND_KICK) 
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<FindAndKickConfig> FindAndKickConfigPtr;


/**
 * @struct PBExternalInterfaceConfig
 * @brief The cognition module configuration
 */
struct PBExternalInterfaceConfig : PBConfig
{
  /**
   * Constructor
   * 
   * @param type: Startup behavior type
   */ 
  PBExternalInterfaceConfig(const PBExternalInterfaceTypes& type) :
    PBConfig(PBIds::EXTERNAL_INTERFACE, 99999.f, (int)type)
  {
  }
};
typedef boost::shared_ptr<PBExternalInterfaceConfig> PBExternalInterfaceConfigPtr;

/**
 * @struct NIHACognitionConfig
 * @brief The config for NIHA cognition interface
 */
struct NIHACognitionConfig : PBExternalInterfaceConfig
{
  /**
   * Constructor
   */ 
  NIHACognitionConfig() : 
    PBExternalInterfaceConfig(PBExternalInterfaceTypes::NIHA_COGNITION)
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<NIHACognitionConfig> NIHACognitionConfigPtr;

/**
 * @struct UserRequestsHandlerConfig
 * @brief The config for user requests handler interface
 */
struct UserRequestsHandlerConfig : PBExternalInterfaceConfig
{
  /**
   * Constructor
   */ 
  UserRequestsHandlerConfig() : 
    PBExternalInterfaceConfig(PBExternalInterfaceTypes::USER_REQ_HANDLER)
  {
  }
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException) {}
};
typedef boost::shared_ptr<UserRequestsHandlerConfig> UserRequestsHandlerConfigPtr;
