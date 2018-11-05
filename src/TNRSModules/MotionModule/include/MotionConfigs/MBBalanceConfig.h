/**
 * @file MotionModule/include/MotionConfigs/MBBalanceConfig.h
 *
 * This file declares the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"

/**
 * @struct MBBalanceConfig
 * @brief Balance behavior configuration
 */
struct MBBalanceConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the balance behavior
   * @param supportLeg: The support leg to shift the balance to
   */ 
  MBBalanceConfig(
    const MBBalanceTypes& type,
    const unsigned& supportLeg);
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBBalanceConfig> 
    makeFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  //! The support leg to shift the balance to
  unsigned supportLeg;
};
typedef boost::shared_ptr<MBBalanceConfig> MBBalanceConfigPtr;

/**
 * @struct MPComControlConfig
 * @brief Configuration for motion primitive based center of mass 
 *  control behavior
 */
struct MPComControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param timeToReachB: Time to shift the balance to support leg
   */ 
  MPComControlConfig(
    const unsigned& supportLeg = 0,
    const float& timeToReachB = 0.f);
  
  /**
   * @derived
   */  
  void validate() throw (BConfigException);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  //! Time to shift the balance to support leg
  float timeToReachB;
};
typedef boost::shared_ptr<MPComControlConfig> MPComControlConfigPtr;

/**
 * @struct PIDComControlConfig
 * @brief Configuration for pid based center of mass 
 *  control behavior
 */
struct PIDComControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   */ 
  PIDComControlConfig(const unsigned& supportLeg = 0);
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
};
typedef boost::shared_ptr<PIDComControlConfig> PIDComControlConfigPtr;

/**
 * @struct ZmpControlConfig
 * @brief Configuration for zmp controller based balance
 */
struct ZmpControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param refGenerator: The zmp reference values generator
   */ 
  ZmpControlConfig(
    const unsigned& supportLeg = 0,
    const bool& keepOtherLegContact = true,
    const bool& regularizePosture = true,
    const bool& keepTorsoUpright = false);
  
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param refGenerator: The zmp reference values generator
   */ 
  ZmpControlConfig(
    const unsigned& supportLeg,
    const ZmpRefGeneratorPtr& refGenerator,
    const bool& keepOtherLegContact = true,
    const bool& regularizePosture = true,
    const bool& keepTorsoUpright = false);

  /**
   * @derived
   */ 
  void validate() throw (BConfigException);
  
   /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();

  //! Whether to keep torso upright while balancing
  bool keepTorsoUpright;

  //! Whether to track a target posture while balancing
  bool regularizePosture;

  //! Whether to keep the other leg fixed on ground while balancing
  bool keepOtherLegContact;

  //! Pointer to the zmp references generator
  ZmpRefGeneratorPtr refGenerator;
};
typedef boost::shared_ptr<ZmpControlConfig> ZmpControlConfigPtr;
