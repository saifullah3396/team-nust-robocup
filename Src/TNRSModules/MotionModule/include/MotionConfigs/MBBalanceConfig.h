/**
 * @file MotionModule/include/MotionConfigs/MBBalanceConfig.h
 *
 * This file defines the structs MBBalanceConfig, MPComControlConfig,
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
    const unsigned& supportLeg) :
    MBConfig(MBIds::BALANCE, 45.f, (int)type), // Basically a lot of time
    supportLeg(supportLeg)
  {
  }
  
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
    const unsigned& supportLeg,
    const float& timeToReachB) :
    MBBalanceConfig(MBBalanceTypes::MP_COM_CONTROL, supportLeg),
    timeToReachB(timeToReachB)
  {
  }
  
  /**
   * Returns true if the configuration is valid
   * 
   * @return bool
   */ 
  bool isValid() {
    try {
      if (
        supportLeg != CHAIN_L_LEG &&
        supportLeg != CHAIN_R_LEG || 
        timeToReachB <= 0.5f) 
      { 
        throw 
          BConfigException(
            this, 
            "Invalid behavior configuration parameters passed.", 
            false, 
            EXC_INVALID_BCONFIG_PARAMETERS
          );
      }
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }
  
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
  PIDComControlConfig(const unsigned& supportLeg) :
    MBBalanceConfig(MBBalanceTypes::PID_COM_CONTROL, supportLeg)
  {
  }
  
  /**
   * Returns true if the configuration is valid
   * 
   * @return bool
   */ 
  bool isValid() {
    try {
      if (
        supportLeg != CHAIN_L_LEG &&
        supportLeg != CHAIN_R_LEG) 
      { 
        throw 
          BConfigException(
            this, 
            "Invalid behavior configuration parameters passed.", 
            false, 
            EXC_INVALID_BCONFIG_PARAMETERS
          );
      }
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }
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
    const unsigned& supportLeg,
    const ZmpRefGeneratorPtr& refGenerator) :
    MBBalanceConfig(MBBalanceTypes::ZMP_CONTROL, supportLeg),
    refGenerator(refGenerator)
  {
  }

  /**
   * Returns true if the configuration is valid
   * 
   * @return bool
   */ 
  bool isValid() {
    try {
      if (!refGenerator) 
      { 
        throw 
          BConfigException(
            this, 
            "Invalid behavior configuration parameters passed.", 
            false, 
            EXC_INVALID_BCONFIG_PARAMETERS
          );
      }
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }
  
  //! Pointer to the zmp references generator
  ZmpRefGeneratorPtr refGenerator;
};
typedef boost::shared_ptr<ZmpControlConfig> ZmpControlConfigPtr;
